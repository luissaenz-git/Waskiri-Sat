#include <main.h>
#include <string.h>
#include <stdio.h> 


#device PASS_STRINGS=IN_RAM
#FUSES HS, NOWDT, NOPROTECT, NOLVP
#use delay(clock=20000000)

// ---------------------------
// UART (com con STM32)
// ---------------------------
#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7, bits=8, parity=N, stop=1)

// ---------------------------
// LCD 16x2 � 4 bits
// D4-D7 -> RD0..RD3, RS/E -> RA0/RA1 (ajusta si usas otros)
// ---------------------------
#define LCD_D4  PIN_D0
#define LCD_D5  PIN_D1
#define LCD_D6  PIN_D2
#define LCD_D7  PIN_D3

#define LCD_RS  PIN_A0
#define LCD_E   PIN_A1

#define LED_RX  PIN_B2      // LED para RX (debug)
#define LED_TX  PIN_B1      // LED para TX (debug)

// -------------------
// RST Variables
// -------------------
int8 sec = 0;
int8 min = 0;
int8 hr = 0;
unsigned int16 day = 0;
int8 tick_count = 0;     // Contador para desborde del temporizador
int8 resetData[8] = {0}; //Data Packet to send to STM32 in case of reset

// ---------------------------
// SPI FLASH (AT25F4096)
// CS en RE2 (como en el codigo original)
// ---------------------------
#define FLASH_CS   PIN_E2
// ---------------------------
// Timer1 Interrupt (counts time) **It works as a simple Real Time Clock (RTC) to keep track of time in case of reset, so we can send this data to STM32 and it can decide if it needs to do a reset or not based on the time elapsed since last reset.
// ---------------------------
// PIC16F877A @ 20MHZ: For one second/4 = 5Mhz this means that the timer will overflow every 4 seconds, so we can use it to count seconds by counting overflows.
// Prescaler 1:8 -> 5MHz/8 = 625kHz -> 1 tick cada 1.6us
// Para contar 1 segundo necesitamos 625k ticks -> 625000/65536 = ~9.5 overflows por segundo
// 10 overflows = 1 segundo (con un pequeño error que se acumula, pero es aceptable para este demo)
#INT_TIMER1
void TIMER1_isr() {
   set_timer1(3036); // Cargar el timer para que vuelva a overflow en 1 segundo (65536 - 62500 = 3036)
   tick_count ++; // Reiniciar contador de ticks
   if (tick_count >= 10) {
      tick_count = 0;
      sec ++;
      if(sec>= 60) {
         sec = 0; min++;}
      if (min >= 60) {
         min = 0; hr++;}
      if (hr >= 24) {
         hr = 0; day++;
      }
   }
} 
// ---------------------------
// Helpers LEDs debug
// ---------------------------
static void pulse_rx_led(void){
   output_high(LED_RX);
   delay_ms(60);
   output_low(LED_RX);
}

static void pulse_tx_led(void){
   output_high(LED_TX);
   delay_ms(60);
   output_low(LED_TX);
}

// ---------------------------
// LCD DRIVER 4-BIT
// ---------------------------
void lcd_pulse_enable() {
   output_high(LCD_E);
   delay_us(5);
   output_low(LCD_E);
   delay_us(50);
}

void lcd_send_nibble(int8 nibble) {
   output_bit(LCD_D4, (nibble & 0x01) != 0);
   output_bit(LCD_D5, (nibble & 0x02) != 0);
   output_bit(LCD_D6, (nibble & 0x04) != 0);
   output_bit(LCD_D7, (nibble & 0x08) != 0);
   lcd_pulse_enable();
}

void lcd_send_byte(int1 rs, int8 data) {
   output_bit(LCD_RS, rs);  // rs=0 comando, rs=1 dato

   lcd_send_nibble(data >> 4);
   lcd_send_nibble(data & 0x0F);

   delay_us(50);
}

void lcd_cmd(int8 cmd) {
   lcd_send_byte(0, cmd);
}

void lcd_data(int8 data) {
   lcd_send_byte(1, data);
}

void lcd_clear() {
   lcd_cmd(0x01);
   delay_ms(2);
}

void lcd_gotoxy(int8 col, int8 row) {
   int8 addr;
   col--;  // a �ndice 0
   row--;  // a �ndice 0

   if(row == 0)
      addr = 0x00 + col;
   else
      addr = 0x40 + col;

   lcd_cmd(0x80 | addr);
}

void lcd_puts(char *s) {
   while(*s) {
      lcd_data(*s++);
   }
}

void lcd_init_custom() {
   delay_ms(40); // power-up

   output_low(LCD_RS);
   output_low(LCD_E);

   // Secuencia 4-bit
   lcd_send_nibble(0x03);
   delay_ms(5);
   lcd_send_nibble(0x03);
   delay_ms(5);
   lcd_send_nibble(0x03);
   delay_ms(5);
   lcd_send_nibble(0x02);  // modo 4-bit

   // Function set: 4-bit, 2 l�neas, 5x8
   lcd_cmd(0x28);
   // Display ON, cursor OFF
   lcd_cmd(0x0C);
   // Clear
   lcd_clear();
   // Entry mode
   lcd_cmd(0x06);
}

// ---------------------------
// SPI FLASH � funciones b�sicas
// (tomado de tu segundo programa)
// ---------------------------
void flash_select(void) {
   output_low(FLASH_CS);
}

void flash_deselect(void) {
   output_high(FLASH_CS);
}

// RDSR (0x05)
BYTE flash_read_status(void) {
   BYTE status;
   flash_select();
   spi_write(0x05);
   status = spi_read(0);
   flash_deselect();
   return status;
}

// Espera RDY/BUSY
void flash_wait_ready(void) {
   BYTE status;
   do {
      status = flash_read_status();
   } while(status & 0x01);
}

// WREN (0x06)
void flash_write_enable(void) {
   flash_select();
   spi_write(0x06);
   flash_deselect();
}

// Escribir Status Register (0x01) - para desbloquear protección
void flash_write_status(BYTE value) {
   flash_write_enable();
   
   flash_select();
   spi_write(0x01);  // WRSR command
   spi_write(value);
   flash_deselect();
   
   delay_ms(50);  // Esperar a que se complete
}

// Desbloquear toda la Flash (quitar protección BP0, BP1, BP2)
void flash_unlock(void) {
   flash_write_status(0x00);
}

// Page Program un byte (0x02)
void flash_write_byte(unsigned int32 addr, BYTE value) {
   flash_write_enable();

   flash_select();
   spi_write(0x02);                     // Page Program
   spi_write((BYTE)(addr >> 16));       // A23..A16
   spi_write((BYTE)(addr >> 8));        // A15..A8
   spi_write((BYTE)(addr));             // A7..A0
   spi_write(value);                    // dato
   flash_deselect();

   flash_wait_ready();
}

// Escribir string C en flash (sin '\0')
void flash_write_string(unsigned int32 addr, char *str) {
   int i = 0;
   while(str[i] != '\0') {
      flash_write_byte(addr + (unsigned int32)i, str[i]);
      i++;
   }
}

//------------------------------------------------------------------------------
// flash_read_n_bytes: Lee N bytes desde la Flash (comando 0x03)
// Parámetros:
//   - addr: Dirección de inicio (24 bits)
//   - buffer: Buffer donde almacenar los datos leídos
//   - n: Número de bytes a leer
//------------------------------------------------------------------------------
void flash_read_n_bytes(unsigned int32 addr, char *buffer, int n) {
   int i;
   
   flash_select();
   
   // Enviar comando READ (0x03) + dirección de 24 bits
   spi_write(0x03);                     // Comando READ
   spi_write((BYTE)(addr >> 16));       // A23..A16
   spi_write((BYTE)(addr >> 8));        // A15..A8
   spi_write((BYTE)(addr));             // A7..A0
   
   // Leer N bytes consecutivos
   for(i = 0; i < n; i++) {
      buffer[i] = spi_read(0x00);       // Enviar dummy byte, recibir dato
   }
   
   flash_deselect();
   
   // Agregar terminador nulo para uso como string
   buffer[n] = '\0';
}

//------------------------------------------------------------------------------
// flash_sector_erase: Borra un sector de 4KB (comando 0x20)
// Parámetros:
//   - sector_addr: Dirección del sector (0x000000, 0x001000, 0x002000, ...)
// Nota: El borrado pone todos los bytes a 0xFF
//------------------------------------------------------------------------------
#define FLASH_CMD_SECTOR_ERASE  0x52
#define FLASH_SECTOR_SIZE       4096

void flash_sector_erase(unsigned int32 sector_addr) {
   // Alinear dirección al inicio del sector (múltiplo de 4KB)
   sector_addr = sector_addr & 0xFFF000;
   
   // 1) Habilitar escritura (requerido antes de erase)
   flash_write_enable();
   
   // 2) Enviar comando Sector Erase + dirección de 24 bits
   flash_select();
   spi_write(FLASH_CMD_SECTOR_ERASE);   // Comando 0x20
   spi_write((BYTE)(sector_addr >> 16)); // A23..A16
   spi_write((BYTE)(sector_addr >> 8));  // A15..A8
   spi_write((BYTE)(sector_addr));       // A7..A0
   flash_deselect();
   
   // 3) Esperar a que termine el borrado (típico ~200ms para sector)
   flash_wait_ready();
}

// ---------------------------
// UART helpers en PIC
// ---------------------------
void uart_read_line(char *buffer, int maxLen)
{
   int i = 0;
   char c;

   while(i < (maxLen - 1))
   {
      c = getc();             // bloqueante
      if(c == '\n')
         break;
      buffer[i++] = c;
   }
   buffer[i] = '\0';
   pulse_rx_led();
}

// Parsear unsigned int32 desde cadena
unsigned int32 parse_uint32_from(char *s)
{
   unsigned int32 val = 0;

   // Saltar espacios
   while(*s == ' ') s++;

   while(*s >= '0' && *s <= '9')
   {
      val = val * 10u + (unsigned int32)(*s - '0');
      s++;
   }
   return val;
}

// ---------------------------
// MAIN - Demo de Flash Compartida
// ---------------------------
void main()
{
   char line[32];
   char read_buf[16];

   // Direcciones de sectores
   #define SECTOR_0_ADDR  0x000000
   #define SECTOR_1_ADDR  0x001000

   // Inicializar puertos
   output_high(FLASH_CS);        // CS inactivo
   output_low(LED_RX);
   output_low(LED_TX);

   set_tris_b(0b00000000);       // RB como salida (LEDs, etc.)
   set_tris_c(0b10010000);       // RC4 (SDI) entrada, resto salida
   set_tris_e(0b00000000);       // RE2 como salida (CS)

   // Inicializar SPI (modo 0, clock baja -> alta)
   setup_spi(SPI_MASTER | SPI_L_TO_H | SPI_XMIT_L_TO_H | SPI_CLK_DIV_16);

   // Inicializar LCD
   lcd_init_custom();
   // Inicializar Timer1 (RTC Simulado) (Satellite Clock)
   // Prescaler 1:8, overflow cada ~1 segundo
   // For simulation purposes, we will use this timer to keep track of time in case of reset, original schematic seems to use an external RTC of 32.768khz, we are using an internal of 20Mhz
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
   set_timer1(3036);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);
   // ---------------------------
   lcd_clear();
   lcd_gotoxy(1, 1);
   lcd_puts("WASKIRI-SAT");
   lcd_gotoxy(1, 2);
   lcd_puts("PIC Ready");
   
   delay_ms(2000);
   
   while(TRUE)
   {
      // == Show time on LCD
      char buf[17];

      lcd_gotoxy(1, 1);
      sprintf(buf, "Day:%04lu %02d:%02d", day, hr, min);
      lcd_puts(buf);

      lcd_gotoxy(1,2);
      sprintf(buf, "Sec:%02d RstPIC", sec);
      lcd_puts(buf);

      // == Listen for STM32 command
      if(kbhit())
      {
         int8 cmd = getc();

         if(cmd == 0x28) // If STM32 request data...
         {
            // Pack data SAme format as BIRDS 3)
            resetData[0] = 0x8E; // Cabezal o Header del paquete, para que el STM32 sepa que es un paquete de reset
            resetData[1] = sec;
            resetData[2] = min;
            resetData[3] = hr;
            resetData[4] = (day >> 8) & 0xFF; // Parte alta del día
            resetData[5] = day & 0xFF;        // Parte baja del día
            resetData[6] = 0x00; // Reserved for future use
            resetData[7] = 0xBB; // Cabezal Final o Header Final del paquete, para que el STM32 sepa que el paquete de reset ha terminado
            // Send data packet to STM32
            for(int i = 0; i < 8; i++) {
               putc(resetData[i]);
            }
            pulse_tx_led();

            lcd_clear();
            lcd_gotoxy(1, 1);
            lcd_puts("Sent to STM32!");
            lcd_gotoxy(1, 2);
            sprintf(buf, "Day:%04lu", day);
            lcd_puts(buf);
            delay_ms(1500);
         }
      }
      delay_ms(250); // pequeño delay para evitar que el LCD se actualice demasiado rápido, y para dar tiempo a que el STM32 procese los comandos
   }
}

