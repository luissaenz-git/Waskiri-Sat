/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>     // snprintf
#include <stdlib.h>    // malloc, free (si aún los quieres)
#include <string.h>    // memset, memcpy, memcmp, strlen, etc.
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(const char *str);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t col, uint8_t row);

static void LCD_PulseEnable(void);
static void LCD_Send4Bits(uint8_t data);

void UART_ReadLine(char *buffer, uint16_t maxLen);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//------------------------------------------------------------------------------
// Registers for Flash Memory
//------------------------------------------------------------------------------

	#define AT25_CMD_WREN   0x06  // 0000 0110
	#define AT25_CMD_WRDI   0x04
	#define AT25_CMD_RDSR   0x05
	#define AT25_CMD_WRSR   0x01
	#define AT25_CMD_READ   0x03
	#define AT25_CMD_PP     0x02  // Page Program

	#define AT25_FLASH_SIZE   524288U   // 4 Mbit = 512 KiB
	#define AT25_PAGE_SIZE    256U      // 256-byte pages

	#define AT25_CMD_WREN   0x06
	#define AT25_CMD_RDSR   0x05
	#define AT25_CMD_PP     0x02
	#define AT25_CMD_READ   0x03
//---------------------------------------------
// MEMORY_MAP_SATELLITE
//---------------------------------------------

// SECTOR_SIZE
#define SECTOR_SIZE 4096U //4KB

//BASIC ADRESSES
#define ADDR_FLAG			0x001000 //Sector 1: Satellite Flags
#define ADDR_ADDRESSES		0x000000 //Sector 0: Direction Pointer

//FLAGS SIZE DATA
#define FLAG_INTO_SIZE		16		// 16 bytes to save all flags
#define ADDRESS_DATA_SIZE	41		// 41 bytes: 9 pointers(36) + BC_flag(1) + counter(4)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//----------------------------------------------
// SATELLITE FLAGS VARIABLES (Data that needs to be saved)
//----------------------------------------------

//Individual Flags
uint8_t BC_ATTEMPT_FLAG = 0; //Beacon attempts
uint16_t PASSED_DAYS = 0; //Days in orbit (0-65535)
uint8_t ANT_DEP_STATUS = 0; //Antenna Deployed? (0=No , 1=Yes)
uint8_t KILL_FLAG = 0; //Emergency shutdown flag
uint8_t RESET_COUNT = 0; //Reset Counter

//Buffer to read/write flags (16 bytes)
uint8_t flag_info_buffer[FLAG_INTO_SIZE];

//----------------------------------------------
// ADDRESS POINTERS (indicates where to write next data in Flash)
//----------------------------------------------
uint32_t FLAG_DATA_ADDRESS = 0; //Flag info stirage location
uint32_t RSV_DATA_ADDRESS = 0; //Reserved data address
uint32_t SAT_LOG_ADDRESS = 0; //Satellite event log
uint32_t CAM_ADDRESS = 0; //Camera data storage
uint32_t FAB_HK_ADDRESS = 0; // Housekeeping data
uint32_t FAB_CW_ADDRESS = 0; // Continuous Wave data
uint32_t ADCS_SENSOR_ADDRESS = 0; //ADCS sensor data
uint32_t DC_STATUS_ADDRESS = 0; // Desployement status and critical events
uint32_t HIGH_SAMP_HK_ADDRESS = 0; // High sample rate housekeeping data

uint32_t ADDRESS_WRITTING_COUNTER = 0; // How many times address data was written 

//Buffer to read/write address pointers (41 bytes)
uint8_t address_data_buffer[ADDRESS_DATA_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* ========== UART LINE READER ========== */
/* Reads until '\n', stores string without '\n', adds '\0' */
	void UART_ReadLine(char *buffer, uint16_t maxLen){
		uint16_t idx = 0;
		uint8_t ch;

		while (idx < (maxLen - 1)){
			HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
			if (ch == '\n')
				break;
			buffer[idx++] = (char)ch;
		}
		buffer[idx] = '\0';
	}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//------------------------------------------------------------------------------
// Functions for Flash Memory
//------------------------------------------------------------------------------

static inline void AT25_CS_Low(void)
{
    HAL_GPIO_WritePin(CS_STM_GPIO_Port, CS_STM_Pin, GPIO_PIN_RESET);
}

static inline void AT25_CS_High(void)
{
    HAL_GPIO_WritePin(CS_STM_GPIO_Port, CS_STM_Pin, GPIO_PIN_SET);
}

void AT25_WriteEnable(void)
{
    uint8_t cmd = AT25_CMD_WREN;

    AT25_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    AT25_CS_High();
}

/* Write Status Register (para desbloquear protección) */
void AT25_WriteStatusRegister(uint8_t value)
{
    uint8_t cmd[2];
    
    // 1) Habilitar escritura
    AT25_WriteEnable();
    
    // 2) Escribir Status Register
    cmd[0] = AT25_CMD_WRSR;  // 0x01
    cmd[1] = value;
    
    AT25_CS_Low();
    HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);
    AT25_CS_High();
    
    HAL_Delay(50);  // Esperar a que se complete
}

/* Desbloquear toda la Flash (quitar protección de sectores) */
void AT25_UnlockFlash(void)
{
    // Escribir 0x00 al Status Register para quitar BP0, BP1, BP2
    AT25_WriteStatusRegister(0x00);
}

/* Read status register (1 byte) */
uint8_t AT25_ReadStatus(void)
{
    uint8_t cmd = AT25_CMD_RDSR;
    uint8_t status = 0;

    AT25_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    AT25_CS_High();

    return status;
}

/* Wait until RDY bit (bit0) == 0 → device ready */
void AT25_WaitBusy(void)
{
    while (AT25_ReadStatus() & 0x01)   // RDY = 1 → busy
    {
        HAL_Delay(1);/* small delay or just spin */
    }
}


//------------------------------------------------------------------------------
// AT25_ByteProgram: Escribe UN SOLO byte (AT25F4096 no soporta Page Program)
// Cada byte requiere: WREN + comando 0x02 + direccion + 1 byte + wait
//------------------------------------------------------------------------------
void AT25_ByteProgram(uint32_t addr, uint8_t data)
{
    uint8_t cmd[5];

    /* 1) Enable write */
    AT25_WriteEnable();

    /* 2) Send BYTE PROGRAM command + 3-byte address + 1 byte data */
    cmd[0] = AT25_CMD_PP;           // 0x02
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8)  & 0xFF;
    cmd[3] = (addr)       & 0xFF;
    cmd[4] = data;

    AT25_CS_Low();
    HAL_SPI_Transmit(&hspi1, cmd, 5, HAL_MAX_DELAY);
    AT25_CS_High();

    /* 3) Wait for internal program cycle to finish */
    AT25_WaitBusy();
}

//------------------------------------------------------------------------------
// AT25_WriteBuffer: Escribe múltiples bytes (byte por byte para AT25F4096)
//------------------------------------------------------------------------------
void AT25_WriteBuffer(uint32_t addr, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        AT25_ByteProgram(addr + i, data[i]);
    }
}

void AT25_ReadData(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t header[4];

    header[0] = AT25_CMD_READ;
    header[1] = (addr >> 16) & 0xFF;
    header[2] = (addr >> 8)  & 0xFF;
    header[3] = (addr)       & 0xFF;

    AT25_CS_Low();
    HAL_SPI_Transmit(&hspi1, header, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    AT25_CS_High();
}

bool AT25_WriteBuffer_Verified(uint32_t addr, const uint8_t *data, uint32_t len)
{
    // Limitar a un tamaño razonable por seguridad
    if (len == 0 || len > 64) {
        return false;
    }

    uint8_t verify_buf[64];

    // 1) Escribir
    AT25_WriteBuffer(addr, data, len);

    // 2) Leer de vuelta
    AT25_ReadData(addr, verify_buf, len);

    // 3) Comparar
    return (memcmp(data, verify_buf, len) == 0);
}

//------------------------------------------------------------------------------
// AT25_SectorErase: Borra un sector de 4KB (comando 0x20)
// Direcciones de sectores: 0x000000, 0x001000, 0x002000, ...
//------------------------------------------------------------------------------
#define AT25_CMD_SECTOR_ERASE  0x52  // Sector Erase (4KB)
#define AT25_SECTOR_SIZE       4096U // 4KB por sector

void AT25_SectorErase(uint32_t sector_addr)
{
    uint8_t cmd[4];

    // Alinear dirección al inicio del sector (múltiplo de 4KB)
    sector_addr = sector_addr & ~(AT25_SECTOR_SIZE - 1);

    // 1) Habilitar escritura (requerido antes de erase)
    AT25_WriteEnable();

    // 2) Enviar comando Sector Erase + dirección de 24 bits
    cmd[0] = AT25_CMD_SECTOR_ERASE;
    cmd[1] = (sector_addr >> 16) & 0xFF;  // A23-A16
    cmd[2] = (sector_addr >> 8)  & 0xFF;  // A15-A8
    cmd[3] = (sector_addr)       & 0xFF;  // A7-A0

    AT25_CS_Low();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    AT25_CS_High();

    // 3) Esperar a que termine el borrado (típico ~200ms para sector)
    AT25_WaitBusy();
}
//--------------------------------------------------
//MEMORY OPERATIONS FUNCTIONS - FLAGS
//-----------------------------------------------

/**
 * @brief This saves satellite Flags to the Flash Memory
 *
 * Packs all important variables into a bufeer
 * and writes them to the FLAGS sector (0x001000)
 */
void STORE_FLAG_INFO(void)
{
	// == This will pack flags into buffer ===
	flag_info_buffer[0] = BC_ATTEMPT_FLAG;		//Byte 0: Beacon Attempts
	flag_info_buffer[1] = (PASSED_DAYS >> 8) & 0xFF; // Byte 1: Days (high byte)
	flag_info_buffer[2] = PASSED_DAYS & 0xFF;        // Byte 2: Days (low byte)
	flag_info_buffer[3] = ANT_DEP_STATUS;            // Byte 3: Antenna deployed
	flag_info_buffer[4] = KILL_FLAG;                 // Byte 4: Kill switch
	flag_info_buffer[5] = RESET_COUNT;               // Byte 5: Reset counter
	// Bytes 6-15: Reserved for future use

	// == This will erase sector before writing ===
	AT25_SectorErase(ADDR_FLAG);
	HAL_Delay(100); //A delay to wait for erase to complete

	// == This will write buffer to the Flash
	AT25_WriteBuffer(ADDR_FLAG, flag_info_buffer, FLAG_INTO_SIZE);
}

/**
 * @brief This reads satellite FLAGS from Flash Memory
 *
 * Reads the FLAGS sector and restores variables
 * If Flash is empty (0xFF), uses default values
 *
 * @return 1 if valid data found and 0 if flash was empty
 */
uint8_t CHECK_FLAG_INFO(void)
{
    // === This reads buffer from Flash ===
    AT25_ReadData(ADDR_FLAG, flag_info_buffer, FLAG_INTO_SIZE);

    // === This checks if data is valid ===
    // If first bytes are 0xFF, Flash is empty/erased
    if (flag_info_buffer[0] == 0xFF && flag_info_buffer[1] == 0xFF)
    {
        // Flash empty - use default values
        BC_ATTEMPT_FLAG = 0;
        PASSED_DAYS = 0;
        ANT_DEP_STATUS = 0;
        KILL_FLAG = 0;
        RESET_COUNT = 0;

        return 0;  // No data found
    }

    // This unpacks buffer to variables ===
    BC_ATTEMPT_FLAG = flag_info_buffer[0];
    PASSED_DAYS = (flag_info_buffer[1] << 8) | flag_info_buffer[2];  // Combine 2 bytes (it is a uint16 value)
    ANT_DEP_STATUS = flag_info_buffer[3];
    KILL_FLAG = flag_info_buffer[4];
    RESET_COUNT = flag_info_buffer[5];

    return 1;  // Data recovered successfully
}

//--------------------------------------------------
// MEMORY OPERATIONS FUNCTIONS - ADDRESS POINTERS
//--------------------------------------------------

/**
 * @brief Saves address pointers to Flash Memory (Sector 0)
 * 
 * Packs all write pointers into a buffer and saves them
 * to ADDR_ADDRESSES (0x000000) so they survive resets
 * Empaca todos los punteros en el buffer y los guarda en Flash
 */

void SAVE_ADDRESS_DATA(void)
{
    // === Pack 9 pointers into buffer (4 bytes each) ===
    // Pointer 1: FLAG_DATA_ADDRESS (bytes 0-3)
    address_data_buffer[0] = (FLAG_DATA_ADDRESS >> 24) & 0xFF; // Byte 0: FLAG_DATA_ADDRESS (high byte)
    address_data_buffer[1] = (FLAG_DATA_ADDRESS >> 16) & 0xFF;
    address_data_buffer[2] = (FLAG_DATA_ADDRESS >> 8) & 0xFF;
    address_data_buffer[3] = FLAG_DATA_ADDRESS & 0xFF; // Byte 3: FLAG_DATA_ADDRESS (low byte)
    
    // Pointer 2: RSV_DATA_ADDRESS (bytes 4-7)
    address_data_buffer[4] = (RSV_DATA_ADDRESS >> 24) & 0xFF;
    address_data_buffer[5] = (RSV_DATA_ADDRESS >> 16) & 0xFF;
    address_data_buffer[6] = (RSV_DATA_ADDRESS >> 8) & 0xFF;
    address_data_buffer[7] = RSV_DATA_ADDRESS & 0xFF;
    
    // Pointer 3: SAT_LOG_ADDRESS (bytes 8-11)
    address_data_buffer[8] = (SAT_LOG_ADDRESS >> 24) & 0xFF;
    address_data_buffer[9] = (SAT_LOG_ADDRESS >> 16) & 0xFF;
    address_data_buffer[10] = (SAT_LOG_ADDRESS >> 8) & 0xFF;
    address_data_buffer[11] = SAT_LOG_ADDRESS & 0xFF;
    
    // Pointer 4: CAM_ADDRESS (bytes 12-15)
    address_data_buffer[12] = (CAM_ADDRESS >> 24) & 0xFF;
    address_data_buffer[13] = (CAM_ADDRESS >> 16) & 0xFF;
    address_data_buffer[14] = (CAM_ADDRESS >> 8) & 0xFF;
    address_data_buffer[15] = CAM_ADDRESS & 0xFF;
    
    // Pointer 5: FAB_HK_ADDRESS (bytes 16-19)
    address_data_buffer[16] = (FAB_HK_ADDRESS >> 24) & 0xFF;
    address_data_buffer[17] = (FAB_HK_ADDRESS >> 16) & 0xFF;
    address_data_buffer[18] = (FAB_HK_ADDRESS >> 8) & 0xFF;
    address_data_buffer[19] = FAB_HK_ADDRESS & 0xFF;
    
    // Pointer 6: FAB_CW_ADDRESS (bytes 20-23)
    address_data_buffer[20] = (FAB_CW_ADDRESS >> 24) & 0xFF;
    address_data_buffer[21] = (FAB_CW_ADDRESS >> 16) & 0xFF;
    address_data_buffer[22] = (FAB_CW_ADDRESS >> 8) & 0xFF;
    address_data_buffer[23] = FAB_CW_ADDRESS & 0xFF;
    
    // Pointer 7: ADCS_SENSOR_ADDRESS (bytes 24-27)
    address_data_buffer[24] = (ADCS_SENSOR_ADDRESS >> 24) & 0xFF;
    address_data_buffer[25] = (ADCS_SENSOR_ADDRESS >> 16) & 0xFF;
    address_data_buffer[26] = (ADCS_SENSOR_ADDRESS >> 8) & 0xFF;
    address_data_buffer[27] = ADCS_SENSOR_ADDRESS & 0xFF;
    
    // Pointer 8: DC_STATUS_ADDRESS (bytes 28-31)
    address_data_buffer[28] = (DC_STATUS_ADDRESS >> 24) & 0xFF;
    address_data_buffer[29] = (DC_STATUS_ADDRESS >> 16) & 0xFF;
    address_data_buffer[30] = (DC_STATUS_ADDRESS >> 8) & 0xFF;
    address_data_buffer[31] = DC_STATUS_ADDRESS & 0xFF;
    
    // Pointer 9: HIGH_SAMP_HK_ADDRESS (bytes 32-35)
    address_data_buffer[32] = (HIGH_SAMP_HK_ADDRESS >> 24) & 0xFF;
    address_data_buffer[33] = (HIGH_SAMP_HK_ADDRESS >> 16) & 0xFF;
    address_data_buffer[34] = (HIGH_SAMP_HK_ADDRESS >> 8) & 0xFF;
    address_data_buffer[35] = HIGH_SAMP_HK_ADDRESS & 0xFF;
    
    // BC_ATTEMPT_FLAG (byte 36)
    address_data_buffer[36] = BC_ATTEMPT_FLAG;
    
    // ADDRESS_WRITTING_COUNTER (bytes 37-40)
    address_data_buffer[37] = (ADDRESS_WRITTING_COUNTER >> 24) & 0xFF;
    address_data_buffer[38] = (ADDRESS_WRITTING_COUNTER >> 16) & 0xFF;
    address_data_buffer[39] = (ADDRESS_WRITTING_COUNTER >> 8) & 0xFF;
    address_data_buffer[40] = ADDRESS_WRITTING_COUNTER & 0xFF;
    
    // === Erase Sector 0 before writing ===
    AT25_SectorErase(ADDR_ADDRESSES);
    HAL_Delay(100);  // Wait for erase to complete
    
    // === Write buffer to Flash ===
    AT25_WriteBuffer(ADDR_ADDRESSES, address_data_buffer, ADDRESS_DATA_SIZE);
}

/**
 * @brief Reads address pointers from Flash Memory (Sector 0)
 * Lee los 41 bytes desde la Flash y los desempaca a variables
 * Lee los bytes, los verifica (si están en 0xFF, Flash vacía) y los convierte de vuela a 9 variables
 * Reads the buffer from Sector 0 and restores all pointers
 * If Flash is empty (0xFF), uses default values
 * 
 * @return 1 if valid data found, 0 if Flash was empty
 */
uint8_t LOAD_ADDRESS_POINTERS(void)
{
    // === Read buffer from Flash (Sector 0) ===
    AT25_ReadData(ADDR_ADDRESSES, address_data_buffer, ADDRESS_DATA_SIZE);
    
    // === Check if data is valid ===
    // If last 4 bytes (counter) are all 0xFF, Flash is empty/erased
    if (address_data_buffer[37] == 0xFF && 
        address_data_buffer[38] == 0xFF && 
        address_data_buffer[39] == 0xFF && 
        address_data_buffer[40] == 0xFF)
    {
        // Flash empty - use default values (all zeros)
        FLAG_DATA_ADDRESS = 0;
        RSV_DATA_ADDRESS = 0;
        SAT_LOG_ADDRESS = 0;
        CAM_ADDRESS = 0;
        FAB_HK_ADDRESS = 0;
        FAB_CW_ADDRESS = 0;
        ADCS_SENSOR_ADDRESS = 0;
        DC_STATUS_ADDRESS = 0;
        HIGH_SAMP_HK_ADDRESS = 0;
        ADDRESS_WRITTING_COUNTER = 0;
        
        return 0;  // No data found
    }
    
    // === Unpack buffer to variables ===
    // Pointer 1: FLAG_DATA_ADDRESS (bytes 0-3)
    FLAG_DATA_ADDRESS = ((uint32_t)address_data_buffer[0] << 24) |
                        ((uint32_t)address_data_buffer[1] << 16) |
                        ((uint32_t)address_data_buffer[2] << 8)  |
                        ((uint32_t)address_data_buffer[3]);
    
    // Pointer 2: RSV_DATA_ADDRESS (bytes 4-7)
    RSV_DATA_ADDRESS = ((uint32_t)address_data_buffer[4] << 24) |
                       ((uint32_t)address_data_buffer[5] << 16) |
                       ((uint32_t)address_data_buffer[6] << 8)  |
                       ((uint32_t)address_data_buffer[7]);
    
    // Pointer 3: SAT_LOG_ADDRESS (bytes 8-11)
    SAT_LOG_ADDRESS = ((uint32_t)address_data_buffer[8] << 24) |
                      ((uint32_t)address_data_buffer[9] << 16) |
                      ((uint32_t)address_data_buffer[10] << 8) |
                      ((uint32_t)address_data_buffer[11]);
    
    // Pointer 4: CAM_ADDRESS (bytes 12-15)
    CAM_ADDRESS = ((uint32_t)address_data_buffer[12] << 24) |
                  ((uint32_t)address_data_buffer[13] << 16) |
                  ((uint32_t)address_data_buffer[14] << 8)  |
                  ((uint32_t)address_data_buffer[15]);
    
    // Pointer 5: FAB_HK_ADDRESS (bytes 16-19)
    FAB_HK_ADDRESS = ((uint32_t)address_data_buffer[16] << 24) |
                     ((uint32_t)address_data_buffer[17] << 16) |
                     ((uint32_t)address_data_buffer[18] << 8)  |
                     ((uint32_t)address_data_buffer[19]);
    
    // Pointer 6: FAB_CW_ADDRESS (bytes 20-23)
    FAB_CW_ADDRESS = ((uint32_t)address_data_buffer[20] << 24) |
                     ((uint32_t)address_data_buffer[21] << 16) |
                     ((uint32_t)address_data_buffer[22] << 8)  |
                     ((uint32_t)address_data_buffer[23]);
    
    // Pointer 7: ADCS_SENSOR_ADDRESS (bytes 24-27)
    ADCS_SENSOR_ADDRESS = ((uint32_t)address_data_buffer[24] << 24) |
                          ((uint32_t)address_data_buffer[25] << 16) |
                          ((uint32_t)address_data_buffer[26] << 8)  |
                          ((uint32_t)address_data_buffer[27]);
    
    // Pointer 8: DC_STATUS_ADDRESS (bytes 28-31)
    DC_STATUS_ADDRESS = ((uint32_t)address_data_buffer[28] << 24) |
                        ((uint32_t)address_data_buffer[29] << 16) |
                        ((uint32_t)address_data_buffer[30] << 8)  |
                        ((uint32_t)address_data_buffer[31]);
    
    // Pointer 9: HIGH_SAMP_HK_ADDRESS (bytes 32-35)
    HIGH_SAMP_HK_ADDRESS = ((uint32_t)address_data_buffer[32] << 24) |
                           ((uint32_t)address_data_buffer[33] << 16) |
                           ((uint32_t)address_data_buffer[34] << 8)  |
                           ((uint32_t)address_data_buffer[35]);
    
    // BC_ATTEMPT_FLAG (byte 36) - already exists, just update
    BC_ATTEMPT_FLAG = address_data_buffer[36];
    
    // ADDRESS_WRITTING_COUNTER (bytes 37-40)
    ADDRESS_WRITTING_COUNTER = ((uint32_t)address_data_buffer[37] << 24) |
                               ((uint32_t)address_data_buffer[38] << 16) |
                               ((uint32_t)address_data_buffer[39] << 8)  |
                               ((uint32_t)address_data_buffer[40]);
    
    return 1;  // Data recovered successfully
}

//Todo esto va combinando 4 bytes separados de vuelta en un número de 32 bits (uint32_t)

//------------------------------------------------------------------------------
// Functions for LCD Display
//------------------------------------------------------------------------------


static void LCD_PulseEnable(void){
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	HAL_Delay(1); // ~1ms
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}

static void LCD_Send4Bits(uint8_t data){
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	LCD_PulseEnable();
}

void LCD_SendCommand(uint8_t cmd){
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

	LCD_Send4Bits(cmd >> 4);
	LCD_Send4Bits(cmd & 0x0F);

	HAL_Delay(2);
}

void LCD_SendData(uint8_t data){
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

	LCD_Send4Bits(data >> 4);
	LCD_Send4Bits(data & 0x0F);

	HAL_Delay(2);
}

void LCD_Clear(void){
	LCD_SendCommand(0x01);
	HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row){
	uint8_t address;

	if (row == 0)
		address = 0x00 + col;
	else
		address = 0x40 + col;

	LCD_SendCommand(0x80 | address);
}

void LCD_SendString(const char *str){
	while (*str){
		LCD_SendData((uint8_t)*str);
		str++;
	}
}

void LCD_Init(void){
	HAL_Delay(50); // LCD power up delay >40ms

// Set RS/E low initially
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

// 4-bit initialization sequence
	LCD_Send4Bits(0x03);
	HAL_Delay(5);
	LCD_Send4Bits(0x03);
	HAL_Delay(5);
	LCD_Send4Bits(0x03);
	HAL_Delay(5);
	LCD_Send4Bits(0x02); // now 4-bit mode

// Function set: 4-bit, 2 lines, 5x8
	LCD_SendCommand(0x28);
// Display ON, cursor OFF, blink OFF
	LCD_SendCommand(0x0C);
// Clear display
	LCD_Clear();
// Entry mode: increment, no shift
	LCD_SendCommand(0x06);
}



//------------------------------------------------------------------------------
// Functions for MUX
//------------------------------------------------------------------------------
static inline void MUX_Enable(void){
    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);  // OE=0 -> connect
}

static inline void MUX_Disable(void){
    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_SET);    // OE=1 -> disconnect
}

static inline void MUX_Conn_STM(void){
	MUX_Enable();
	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

}

static inline void MUX_Conn_PIC(void){
	MUX_Enable();
	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_SET);
}

// ==== Parser Messages ====
int parse_message_number(const char *s){

		int num = 0;
		int found = 0;

		while (*s){

			if (*s >= '0' && *s <= '9'){
				found = 1;
				num = num * 10 + (*s - '0');
			}else if (found){
				// once we started reading digits and they stop, exit
				break;
			}
        s++;
		}

		if (!found)return 0;
    	return num;
	}

/* Leer línea con timeout (retorna true si éxito, false si timeout) */
bool UART_ReadLine_Timeout(char *buffer, uint16_t maxLen, uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint8_t ch;
    uint32_t start_tick = HAL_GetTick();

    while (idx < (maxLen - 1))
    {
        // Verificar timeout
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            buffer[idx] = '\0';  // Terminar string
            return false;  // Timeout alcanzado
        }

        // Intentar recibir con timeout corto
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &ch, 1, 100);

        if (status == HAL_OK)
        {
            if (ch == '\n')
                break;
            buffer[idx++] = (char)ch;
        }
        // Si status == HAL_TIMEOUT, continuar esperando hasta timeout_ms total
    }

    buffer[idx] = '\0';
    return true;  // Éxito
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  	  AT25_CS_High();
  	//LCD Setup
	  LCD_Init();

	  // Conectar STM32 a Flash y desbloquear protección
	  MUX_Conn_STM();
	  HAL_Delay(10);
	  AT25_UnlockFlash();  // Quitar protección de escritura

	  uint8_t demo_phase = 0; // 0 = STM32 escribe, 1 = PIC escribe
      char msg[17];

	  LCD_Clear();
	  LCD_SetCursor(0, 0);
	  LCD_SendString("WASKIRI-SAT");
	  LCD_SetCursor(0, 1);
	  LCD_SendString("Flash Demo");
	  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // ============================================================
      // TURNO 1: STM32 ESCRIBE EN FLASH
      // ============================================================
      if (demo_phase == 0)
      {
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("=== TEST FLAGS ===");
          HAL_Delay(1500);

          // Connect STM32 to Flash
          MUX_Conn_STM();
          HAL_Delay(100);

          // --- PART 1: Create test data ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Setting flags:");

          PASSED_DAYS = 45;        // 45 days in orbit
          ANT_DEP_STATUS = 1;      // Antenna deployed
          RESET_COUNT = 3;         // 3 resets

          char msg[17];
          snprintf(msg, 17, "Days:%d Ant:%d", PASSED_DAYS, ANT_DEP_STATUS);
          LCD_SetCursor(0, 1);
          LCD_SendString(msg);
          HAL_Delay(2000);

          // --- PART 2: Save to Flash ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Saving to Flash");
          LCD_SetCursor(0, 1);
          LCD_SendString("...");

          STORE_FLAG_INFO();  // <-- SAVE!

          LCD_SetCursor(0, 1);
          LCD_SendString("Done!          ");
          HAL_Delay(1500);

          // --- PART 3: Clear variables (simulate reset) ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Simul. RESET...");

          PASSED_DAYS = 0;         // "Forget" everything
          ANT_DEP_STATUS = 0;
          RESET_COUNT = 0;

          LCD_SetCursor(0, 1);
          LCD_SendString("Vars cleared!");
          HAL_Delay(1500);

          // --- PART 4: Recover from Flash ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Reading Flash...");

          uint8_t result = CHECK_FLAG_INFO();  // <-- RECOVER!

          if (result)
          {
              LCD_SetCursor(0, 1);
              LCD_SendString("Data recovered!");
              HAL_Delay(1500);

              // Show recovered data
              LCD_Clear();
              LCD_SetCursor(0, 0);
              snprintf(msg, 17, "Days: %d", PASSED_DAYS);
              LCD_SendString(msg);
              LCD_SetCursor(0, 1);
              snprintf(msg, 17, "Ant:%d Rst:%d", ANT_DEP_STATUS, RESET_COUNT);
              LCD_SendString(msg);
              HAL_Delay(3000);

              LCD_Clear();
              LCD_SetCursor(0, 0);
              LCD_SendString("FLAGS TEST");
              LCD_SetCursor(0, 1);
              LCD_SendString("   OK! :D");
              HAL_Delay(2000);
          }
          else
          {
              LCD_SetCursor(0, 1);
              LCD_SendString("Flash empty!");
              HAL_Delay(2000);
          }

          demo_phase = 1;  // Next phase
      }

      // ============================================================
      // TURNO 2: DAR ACCESO AL PIC
      // ============================================================
      else if (demo_phase == 1)
        {
            // == REQUEST DATA FROM PIC (ResetPIC) ==
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_SendString("Request data PIC");
            LCD_SetCursor(0, 1);
            LCD_SendString("ResetPIC...");
            HAL_Delay(1500);

            //Send command 0x28 to PIC to request data (as BIRDS 3)
            uint8_t cmd = 0x28;
            HAL_UART_Transmit(&huart2, &cmd, 1, HAL_MAX_DELAY);

            // Receive 8-byte packet from PIC
            uint8_t picData[8];
            HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, picData, 8, 5000); // 5s timeout

            if (status == HAL_OK && picData[0] == 0x8E && picData[7] == 0xBB)
            {
                // Extract time from packet
                uint8_t pic_sec = picData[1];
                uint8_t pic_min = picData[2];
                uint8_t pic_hour = picData[3];
                uint16_t pic_day = (picData[4] << 8) | picData[5];

                //Update flags with real PIC data
                PASSED_DAYS = pic_day;
                RESET_COUNT++; // Increment reset count since PIC is resetting us

                // Show received data
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("PIC Data Recv!");
                LCD_SetCursor(0, 1);
                snprintf(msg, 17, "Day:%d %02d:%02d:%02d", pic_day, pic_hour, pic_min, pic_sec);
                LCD_SendString(msg);
                HAL_Delay(2500); // Show success message

                // Save to Flash so we can verify it later in phase 3
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("Saving FLAGS...");

                MUX_Conn_STM(); // Connect STM32 to Flash
                HAL_Delay(10);
                STORE_FLAG_INFO(); // Save updated flags with PIC data

                LCD_SetCursor(0, 1);
                LCD_SendString("Saved to SFM");
                HAL_Delay(1500);

                demo_phase = 2; // Next phase
            }
            else
            {
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("PIC no response");
                LCD_SetCursor(0, 1);
                LCD_SendString("Retrying...");
                HAL_Delay(2000);

                // Stays in phase 1, retries next loop
            }
        }
      // ============================================================
      // TURNO 3: STM32 VERIFICA CAMBIOS DEL PIC
      // ============================================================
      else if (demo_phase == 2)
       {
        // Verify Flash persistence after PIC update
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_SendString("Simul. RESET...");
        HAL_Delay(1500);

        // Clear variables (simulate power loss)
        PASSED_DAYS = 0;
        RESET_COUNT = 0;

        LCD_SetCursor(0, 1);
        LCD_SendString("POWER LOST!");
        HAL_Delay(1500);

        //Recover from Flash
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_SendString("Recover Flash...");

        MUX_Conn_STM(); // Connect STM32 to Flash
        HAL_Delay(10);
        uint8_t result = CHECK_FLAG_INFO();  // <-- RECOVER!

        if (result)
        {
            LCD_Clear();
            LCD_SetCursor(0, 0);
            snprintf(msg, 17, "Days:%d Rst:%d", PASSED_DAYS, RESET_COUNT);
            LCD_SendString(msg);
            LCD_SetCursor(0, 1);
            LCD_SendString("Recovery OK!");
            HAL_Delay(3000);
        }
        else
        {
            LCD_SetCursor(0, 1);
            LCD_SendString("Failed FM!");
            HAL_Delay(2000);
        }

        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_SendString("Demo Complete!");
        LCD_SetCursor(0, 1);
        LCD_SendString("Next test in 5s");
        HAL_Delay(5000);

        demo_phase = 3;  // Next: test ADDRESS pointers
      }
          // ============================================================
      // FASE 3: TEST ADDRESS POINTERS
      // ============================================================
      else if (demo_phase == 3)
      {
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("=TEST POINTERS=");
          HAL_Delay(1500);

          // Connect STM32 to Flash
          MUX_Conn_STM();
          HAL_Delay(100);

          // --- PART 1: Set test pointer values ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Setting ptrs...");

          // Set some test addresses (in hex for clarity)
          FAB_HK_ADDRESS = 0x002000;       // Sector 2
          CAM_ADDRESS = 0x010000;      // Sector 16
          SAT_LOG_ADDRESS = 0x020000;  // Sector 32
          ADDRESS_WRITTING_COUNTER = 1; // First time

          snprintf(msg, 17, "HK:%lX CAM:%lX", FAB_HK_ADDRESS, CAM_ADDRESS);
          LCD_SetCursor(0, 1);
          LCD_SendString(msg);
          HAL_Delay(2000);

          // --- PART 2: Save to Flash ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Saving ptrs...");

          SAVE_ADDRESS_DATA();  // <-- SAVE!

          LCD_SetCursor(0, 1);
          LCD_SendString("Done!");
          HAL_Delay(1500);

          // --- PART 3: Clear variables (simulate reset) ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Simul. RESET...");

          FAB_HK_ADDRESS = 0;
          CAM_ADDRESS = 0;
          SAT_LOG_ADDRESS = 0;
          ADDRESS_WRITTING_COUNTER = 0;

          LCD_SetCursor(0, 1);
          LCD_SendString("Ptrs cleared!");
          HAL_Delay(1500);

          // --- PART 4: Recover from Flash ---
          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Reading Flash...");

          uint8_t result = LOAD_ADDRESS_POINTERS();  // <-- RECOVER!

          if (result)
          {
              LCD_Clear();
              LCD_SetCursor(0, 0);
              LCD_SendString("Ptrs recovered!");
              HAL_Delay(1500);

              // Show recovered pointers
              LCD_Clear();
              LCD_SetCursor(0, 0);
              snprintf(msg, 17, "HK:%lX LOG:%lX", FAB_HK_ADDRESS, SAT_LOG_ADDRESS);
              LCD_SendString(msg);
              LCD_SetCursor(0, 1);
              snprintf(msg, 17, "CAM:%lX Cnt:%lu", CAM_ADDRESS, ADDRESS_WRITTING_COUNTER);
              LCD_SendString(msg);
              HAL_Delay(3000);

              LCD_Clear();
              LCD_SetCursor(0, 0);
              LCD_SendString("POINTERS TEST");
              LCD_SetCursor(0, 1);
              LCD_SendString("   OK!");
              HAL_Delay(2000);
          }
          else
          {
              LCD_Clear();
              LCD_SetCursor(0, 0);
              LCD_SendString("Flash empty!");
              LCD_SetCursor(0, 1);
              LCD_SendString("No pointers");
              HAL_Delay(2000);
          }

          LCD_Clear();
          LCD_SetCursor(0, 0);
          LCD_SendString("Demo Complete!");
          LCD_SetCursor(0, 1);
          LCD_SendString("Restart in 5s");
          HAL_Delay(5000);

          demo_phase = 0;  // Restart from beginning
      }

    
      /* USER CODE END WHILE */
    }
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
  }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SW_Pin|OE_Pin|CS_STM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_E_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_RS_Pin
                          |LCD_D4_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_Pin OE_Pin CS_STM_Pin */
  GPIO_InitStruct.Pin = SW_Pin|OE_Pin|CS_STM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_D5_Pin LCD_D6_Pin LCD_RS_Pin
                           LCD_D4_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_RS_Pin
                          |LCD_D4_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
