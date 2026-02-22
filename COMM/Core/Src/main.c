/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   STM32F401VE - ADDNICS COM Board (ADD1397B) Behavior Simulation
  * @author  Laura Choque
  *
  *  MAPPING (Proteus Simulation):
  *   - USART1 @115200: TRX UART (COM96_TRX_TXD / COM96_TRX_RXD) -> Ground Station terminal
  *   - USART2 @115200: CONFIG UART (CONFIG UART RXD) -> command console terminal
  *   - PB8: COM96_PWR_ONOFF (TX enable)   [0=OFF, 1=ON]
  *   - PB9: COM_CWPIC_CONFIG (CW enable)  [0=OFF, 1=ON]
  *   - PB13: COM_CWPIC_CWKEY (CW keying)   [0=OFF, 1=ON, keying pulse]
  *   - PB0 / ADC_CHANNEL_8 : COM_RSSI (analog out)
  *   - PB1 / ADC_CHANNEL_9 : COM96_TEMP (analog out)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ===================== LCD DEFINES ===================== */
#define LCD_PORT GPIOA
#define RS_PIN   GPIO_PIN_1
#define EN_PIN   GPIO_PIN_2
#define D4_PIN   GPIO_PIN_4
#define D5_PIN   GPIO_PIN_5
#define D6_PIN   GPIO_PIN_6
#define D7_PIN   GPIO_PIN_7

/* ===================== LED DEFINES ===================== */
#define LED_PORT    GPIOC
#define LED_RX_PIN  GPIO_PIN_4   // extra LED (optional)
#define LED_TX_PIN  GPIO_PIN_5   // extra LED (optional)

/* ===================== COM CONTROL PIN MAPPING ===================== */
#define COM_CTRL_PORT GPIOB
#define PIN_PWR_ONOFF GPIO_PIN_8
#define PIN_CW_CONFIG GPIO_PIN_9
#define PIN_CW_KEY    GPIO_PIN_13

/* ===================== HANDLES ===================== */
UART_HandleTypeDef huart1;  // TRX UART
UART_HandleTypeDef huart2;  // CONFIG UART
IWDG_HandleTypeDef hiwdg;
ADC_HandleTypeDef hadc1;

/* ===================== UART BUFFERS ===================== */
static uint8_t trx_rx_byte;
static uint8_t cfg_rx_byte;

#define CFG_LINE_MAX 32
static char cfg_line[CFG_LINE_MAX];
static volatile uint8_t cfg_idx = 0;
static volatile uint8_t cfg_line_ready = 0;

/* ===================== TRX (USART1) FRAME PARSER ===================== */
/* Detect: 0x7E 0x7E 0x42  ... payload ... 0x7E */
#define TRX_MAX_PAYLOAD 128
static uint8_t trx_payload[TRX_MAX_PAYLOAD];
static uint16_t trx_pl_len = 0;

typedef enum {
  TRX_WAIT_7E1 = 0,
  TRX_WAIT_7E2,
  TRX_WAIT_42,
  TRX_IN_PAYLOAD
} TRX_ParseState_t;

static TRX_ParseState_t trx_state = TRX_WAIT_7E1;
static volatile uint8_t trx_frame_ready = 0;

/* ===================== COM STATE (matches ICD concepts) ===================== */
static volatile uint8_t tx_enabled = 1;   // RON/ROF  (default ON)
static volatile uint8_t mod_enabled = 1;  // MON/MOF  (default ON)
static volatile uint8_t src_pnf    = 1;   // PNF/PNN  (default PNF, data from UART)

typedef enum {
  COM_MODE_RX = 0,
  COM_MODE_TX_GMSK = 1,
  COM_MODE_CW = 2
} COM_Mode_t;

static COM_Mode_t com_mode = COM_MODE_RX;

/* ===================== PROTOTYPES ===================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);

/* LCD */
static void LCD_Init(void);
static void LCD_Command(uint8_t cmd);
static void LCD_Char(char data);
static void LCD_String(const char *str);
static void LCD_SetCursor(uint8_t row, uint8_t col);
static void LCD_Send4Bits(uint8_t data);

/* COM behavior */
static void COM_SetRX(void);
static void COM_SetTX_GMSK(void);
static void COM_SendDownlinkBurst(uint8_t n_pkts);
static void COM_SendCW_Beacon(uint16_t ms_total);

/* ADC */
static uint16_t Read_ADC(uint32_t channel);
static uint16_t COM_Read_RSSI(void);
static uint16_t COM_Read_TEMP(void);

/* TRX framing */
static void TRX_ParseByte(uint8_t b);
static void TRX_HandleFrame(const uint8_t *pl, uint16_t len);

/* Helpers */
static void TRX_PrintStatus(void);
static void CFG_PrintHelp(void);
static void CFG_HandleLine(const char *line);
static void safe_uart_tx(UART_HandleTypeDef *huart, const char *s);

/* ===================== UART CALLBACK ===================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* TRX UART (USART1): receives framed "uplink" bytes from Ground Station */
  if (huart->Instance == USART1)
  {
    TRX_ParseByte(trx_rx_byte);
    HAL_UART_Receive_IT(&huart1, &trx_rx_byte, 1);
  }

  /* CONFIG UART (USART2): receives ASCII commands RON/MON/PNF... */
  if (huart->Instance == USART2)
  {
    if (cfg_rx_byte == '\r' || cfg_rx_byte == '\n')
    {
      if (cfg_idx > 0)
      {
        cfg_line[cfg_idx] = '\0';
        cfg_line_ready = 1;
      }
      cfg_idx = 0;
    }
    else
    {
      if (cfg_idx < (CFG_LINE_MAX - 1))
      {
        cfg_line[cfg_idx++] = (char)cfg_rx_byte;
      }
      else
      {
        /* overflow -> reset */
        cfg_idx = 0;
      }
    }
    HAL_UART_Receive_IT(&huart2, &cfg_rx_byte, 1);
  }
}

/* ===================== TRX FRAMING ===================== */
static void TRX_ParseByte(uint8_t b)
{
  switch (trx_state)
  {
    case TRX_WAIT_7E1:
      if (b == 0x7E) trx_state = TRX_WAIT_7E2;
      break;

    case TRX_WAIT_7E2:
      if (b == 0x7E) trx_state = TRX_WAIT_42;
      else trx_state = TRX_WAIT_7E1;
      break;

    case TRX_WAIT_42:
      if (b == 0x42) {
        trx_pl_len = 0;
        trx_state = TRX_IN_PAYLOAD;
      } else {
        trx_state = TRX_WAIT_7E1;
      }
      break;

    case TRX_IN_PAYLOAD:
      if (b == 0x7E) {
        trx_frame_ready = 1;          // complete
        trx_state = TRX_WAIT_7E1;
      } else {
        if (trx_pl_len < TRX_MAX_PAYLOAD) {
          trx_payload[trx_pl_len++] = b;
        } else {
          trx_pl_len = 0;             // drop on overflow
          trx_state = TRX_WAIT_7E1;
        }
      }
      break;

    default:
      trx_state = TRX_WAIT_7E1;
      break;
  }
}

static void TRX_HandleFrame(const uint8_t *pl, uint16_t len)
{
  if (len == 0) {
    safe_uart_tx(&huart1, "URX: empty frame\r\n");
    return;
  }

  uint8_t cmd = pl[0];

  char msg[96];
  snprintf(msg, sizeof(msg), "URX FRAME: CMD=0x%02X LEN=%u\r\n", cmd, (unsigned)len);
  safe_uart_tx(&huart1, msg);

  switch (cmd)
  {
    case 0x77:
      /* Like BIRDS COM PIC: automatic packet transmission */
      safe_uart_tx(&huart1, "CMD 0x77: AUTO DOWNLINK BURST\r\n");
      COM_SendDownlinkBurst(5);
      break;

    case 0x20:
      safe_uart_tx(&huart1, "CMD 0x20: CW BEACON\r\n");
      COM_SendCW_Beacon(2000);
      break;

    case 0x30:
    {
      uint16_t rssi = COM_Read_RSSI();
      uint16_t temp = COM_Read_TEMP();
      char hk[96];
      snprintf(hk, sizeof(hk), "HK: RSSI=%u TEMP=%u\r\n", (unsigned)rssi, (unsigned)temp);
      safe_uart_tx(&huart1, hk);
      break;
    }

    case 0xAA:
      safe_uart_tx(&huart1, "CMD 0xAA: SYNC/KEEPALIVE\r\n");
      break;

    default:
      safe_uart_tx(&huart1, "CMD: UNKNOWN\r\n");
      break;
  }
}

/* ===================== LCD ===================== */
static void LCD_Send4Bits(uint8_t data)
{
  HAL_GPIO_WritePin(LCD_PORT, D4_PIN, (data >> 0) & 1);
  HAL_GPIO_WritePin(LCD_PORT, D5_PIN, (data >> 1) & 1);
  HAL_GPIO_WritePin(LCD_PORT, D6_PIN, (data >> 2) & 1);
  HAL_GPIO_WritePin(LCD_PORT, D7_PIN, (data >> 3) & 1);

  HAL_GPIO_WritePin(LCD_PORT, EN_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_PORT, EN_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
}

static void LCD_Command(uint8_t cmd)
{
  HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_RESET);
  LCD_Send4Bits(cmd >> 4);
  LCD_Send4Bits(cmd & 0x0F);
}

static void LCD_Char(char data)
{
  HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_SET);
  LCD_Send4Bits((uint8_t)data >> 4);
  LCD_Send4Bits((uint8_t)data & 0x0F);
}

static void LCD_String(const char *str)
{
  while (*str) LCD_Char(*str++);
}

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
  LCD_Command((row == 0 ? 0x80 : 0xC0) + col);
}

static void LCD_Init(void)
{
  HAL_Delay(50);
  LCD_Command(0x02);
  LCD_Command(0x28);
  LCD_Command(0x0C);
  LCD_Command(0x06);
  LCD_Command(0x01);
}

/* ===================== ADC ===================== */
static uint16_t Read_ADC(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return value;
}

static uint16_t COM_Read_RSSI(void)
{
  return Read_ADC(ADC_CHANNEL_8); // PB0
}

static uint16_t COM_Read_TEMP(void)
{
  return Read_ADC(ADC_CHANNEL_9); // PB1
}

/* ===================== COM MODES ===================== */
static void COM_SetRX(void)
{
  com_mode = COM_MODE_RX;

  HAL_GPIO_WritePin(LED_PORT, LED_RX_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_PORT, LED_TX_PIN, GPIO_PIN_RESET);

  /* In RX, CW outputs off */
  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_CONFIG, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_KEY, GPIO_PIN_RESET);

  LCD_Command(0x01);
  LCD_String("MODE: RX");
  LCD_SetCursor(1, 0);
  LCD_String("Waiting uplink");

  TRX_PrintStatus();
}

static void COM_SetTX_GMSK(void)
{
  com_mode = COM_MODE_TX_GMSK;

  HAL_GPIO_WritePin(LED_PORT, LED_RX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_TX_PIN, GPIO_PIN_SET);

  LCD_Command(0x01);
  LCD_String("MODE: TX GMSK");
  LCD_SetCursor(1, 0);
  LCD_String("Downlink...");

  TRX_PrintStatus();
}

static void COM_SendDownlinkBurst(uint8_t n_pkts)
{
  if (!tx_enabled)
  {
    safe_uart_tx(&huart1, "TX BLOCKED: ROF (tx off)\r\n");
    return;
  }
  if (!mod_enabled)
  {
    safe_uart_tx(&huart1, "TX BLOCKED: MOF (mod off) -> carrier only mode\r\n");
    safe_uart_tx(&huart1, "[CARRIER 437.375MHz]\r\n");
    return;
  }

  COM_SetTX_GMSK();

  for (uint8_t i = 0; i < n_pkts; i++)
  {
    uint16_t rssi = COM_Read_RSSI();
    uint16_t temp = COM_Read_TEMP();

    char pkt[96];
    if (src_pnf)
    {
      snprintf(pkt, sizeof(pkt),
               "DL[%u]: AX25(GMSK4800) RSSI=%u TEMP=%u\r\n",
               (unsigned)i, (unsigned)rssi, (unsigned)temp);
    }
    else
    {
      snprintf(pkt, sizeof(pkt),
               "DL[%u]: PN9(PNN) RSSI=%u TEMP=%u\r\n",
               (unsigned)i, (unsigned)rssi, (unsigned)temp);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)pkt, (uint16_t)strlen(pkt), 200);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(300);
  }

  COM_SetRX();
}

static void COM_SendCW_Beacon(uint16_t ms_total)
{
  if (!tx_enabled)
  {
    safe_uart_tx(&huart1, "CW BLOCKED: ROF (tx off)\r\n");
    return;
  }

  com_mode = COM_MODE_CW;

  /* Enable CW TX */
  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_CONFIG, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LED_PORT, LED_RX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_TX_PIN, GPIO_PIN_SET);

  LCD_Command(0x01);
  LCD_String("MODE: CW");
  LCD_SetCursor(1,0);
  LCD_String("Beacon keying");

  safe_uart_tx(&huart1, "CW: start (0.1W OOK) 437.375MHz\r\n");

  uint16_t elapsed = 0;
  while (elapsed < ms_total)
  {
    /* tone ON */
    HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_KEY, GPIO_PIN_SET);
    safe_uart_tx(&huart1, "CW_KEY=1 (tone ON)\r\n");
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(200);
    elapsed += 200;
    if (elapsed >= ms_total) break;

    /* tone OFF */
    HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_KEY, GPIO_PIN_RESET);
    safe_uart_tx(&huart1, "CW_KEY=0 (tone OFF)\r\n");
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(200);
    elapsed += 200;
  }

  /* Disable CW */
  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_KEY, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_CW_CONFIG, GPIO_PIN_RESET);

  safe_uart_tx(&huart1, "CW: stop\r\n");
  COM_SetRX();
}

/* ===================== STATUS / HELP ===================== */
static void safe_uart_tx(UART_HandleTypeDef *huart, const char *s)
{
  HAL_UART_Transmit(huart, (uint8_t*)s, (uint16_t)strlen(s), 300);
}

static void TRX_PrintStatus(void)
{
  char st[128];
  snprintf(st, sizeof(st),
           "STATUS: R%s M%s %s | PWR_ONOFF(PB8)=%d CW_CFG(PB9)=%d CW_KEY(PB13)=%d\r\n",
           tx_enabled ? "ON" : "OF",
           mod_enabled ? "ON" : "OF",
           src_pnf ? "PNF" : "PNN",
           (int)HAL_GPIO_ReadPin(COM_CTRL_PORT, PIN_PWR_ONOFF),
           (int)HAL_GPIO_ReadPin(COM_CTRL_PORT, PIN_CW_CONFIG),
           (int)HAL_GPIO_ReadPin(COM_CTRL_PORT, PIN_CW_KEY));
  safe_uart_tx(&huart1, st);
}

static void CFG_PrintHelp(void)
{
  safe_uart_tx(&huart2,
    "ADDNICS CONFIG UART COMMANDS:\r\n"
    "  RON = TX output ON\r\n"
    "  ROF = TX output OFF\r\n"
    "  MON = Modulation ON (normal TX)\r\n"
    "  MOF = Modulation OFF (unmodulated carrier)\r\n"
    "  PNF = Data source = UART (TRX)\r\n"
    "  PNN = Data source = internal PN9\r\n"
    "  DL  = send downlink burst (5 packets)\r\n"
    "  CW  = send CW beacon (2 seconds)\r\n"
    "  ST  = print status\r\n"
    "  HELP = show this help\r\n\r\n");
}

static void CFG_HandleLine(const char *line)
{
  char cmd[CFG_LINE_MAX];
  size_t n = strlen(line);
  if (n >= sizeof(cmd)) n = sizeof(cmd)-1;

  for (size_t i=0;i<n;i++)
  {
    char c = line[i];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    cmd[i] = c;
  }
  cmd[n] = '\0';

  if (strcmp(cmd, "HELP") == 0)
  {
    CFG_PrintHelp();
    return;
  }

  if (strcmp(cmd, "RON") == 0) { tx_enabled = 1; safe_uart_tx(&huart2, "OK: RON\r\n"); }
  else if (strcmp(cmd, "ROF") == 0) { tx_enabled = 0; safe_uart_tx(&huart2, "OK: ROF\r\n"); }
  else if (strcmp(cmd, "MON") == 0) { mod_enabled = 1; safe_uart_tx(&huart2, "OK: MON\r\n"); }
  else if (strcmp(cmd, "MOF") == 0) { mod_enabled = 0; safe_uart_tx(&huart2, "OK: MOF\r\n"); }
  else if (strcmp(cmd, "PNF") == 0) { src_pnf    = 1; safe_uart_tx(&huart2, "OK: PNF\r\n"); }
  else if (strcmp(cmd, "PNN") == 0) { src_pnf    = 0; safe_uart_tx(&huart2, "OK: PNN\r\n"); }
  else if (strcmp(cmd, "DL")  == 0) { safe_uart_tx(&huart2, "ACTION: DL burst\r\n"); COM_SendDownlinkBurst(5); }
  else if (strcmp(cmd, "CW")  == 0) { safe_uart_tx(&huart2, "ACTION: CW beacon\r\n"); COM_SendCW_Beacon(2000); }
  else if (strcmp(cmd, "ST")  == 0) { TRX_PrintStatus(); safe_uart_tx(&huart2, "OK: ST\r\n"); }
  else
  {
    safe_uart_tx(&huart2, "ERR: Unknown cmd. Type HELP\r\n");
  }

  /* Mirror tx_enabled to PB8 immediately */
  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_PWR_ONOFF,
                    tx_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* Update LCD short status */
  LCD_Command(0x01);
  LCD_String(tx_enabled ? "RON " : "ROF ");
  LCD_String(mod_enabled ? "MON " : "MOF ");
  LCD_String(src_pnf ? "PNF" : "PNN");
  LCD_SetCursor(1,0);
  LCD_String("CFG UART CMD");

  COM_SetRX();
}

/* ===================== MAIN ===================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();  // TRX
  MX_USART2_UART_Init();  // CONFIG
  MX_IWDG_Init();
  MX_ADC1_Init();

  LCD_Init();

  /* Defaults per ICD: RON, MON, PNF at power-on */
  tx_enabled = 1;
  mod_enabled = 1;
  src_pnf = 1;

  HAL_GPIO_WritePin(COM_CTRL_PORT, PIN_PWR_ONOFF, GPIO_PIN_SET);

  safe_uart_tx(&huart2, "\r\nCONFIG UART READY. Commands in HELP + Enter\r\n");
  safe_uart_tx(&huart1, "\r\nTRX UART READY (Ground Station)\r\n");

  COM_SetRX();

  HAL_UART_Receive_IT(&huart1, &trx_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &cfg_rx_byte, 1);

  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);

    if (cfg_line_ready)
    {
      cfg_line_ready = 0;
      CFG_HandleLine(cfg_line);
    }

    if (trx_frame_ready)
    {
      trx_frame_ready = 0;
      TRX_HandleFrame(trx_payload, trx_pl_len);
    }

    static uint32_t last = 0;
    if (HAL_GetTick() - last > 1500)
    {
      last = HAL_GetTick();
      uint16_t rssi = COM_Read_RSSI();
      uint16_t temp = COM_Read_TEMP();

      char row2[17];
      snprintf(row2, sizeof(row2), "R%4u T%4u", (unsigned)rssi, (unsigned)temp);
      LCD_SetCursor(1,0);
      LCD_String("                ");
      LCD_SetCursor(1,0);
      LCD_String(row2);
    }
  }
}

/* ===================== CLOCK ===================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* ===================== ADC INIT (stable for Proteus) ===================== */
static void MX_ADC1_Init(void)
{
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
}

/* ===================== IWDG ===================== */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

/* ===================== UART INIT ===================== */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;   // FIX: ICD = 115.2 kbps
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;   // FIX: ICD = 115.2 kbps
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* ===================== GPIO INIT ===================== */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* LCD pins */
  HAL_GPIO_WritePin(GPIOA, RS_PIN|EN_PIN|D4_PIN|D5_PIN|D6_PIN|D7_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = RS_PIN|EN_PIN|D4_PIN|D5_PIN|D6_PIN|D7_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* LEDs */
  HAL_GPIO_WritePin(GPIOC, LED_RX_PIN|LED_TX_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_RX_PIN|LED_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PWR_ONOFF PB8 */
  HAL_GPIO_WritePin(GPIOB, PIN_PWR_ONOFF, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PIN_PWR_ONOFF;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* CW_CONFIG PB9 */
  HAL_GPIO_WritePin(GPIOB, PIN_CW_CONFIG, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PIN_CW_CONFIG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* CW_KEY PB13 */
  HAL_GPIO_WritePin(GPIOB, PIN_CW_KEY, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PIN_CW_KEY;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* ===================== ERROR HANDLER ===================== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
