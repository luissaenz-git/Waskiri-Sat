//FUNCTIONS FOR MAIN PIC BASED ON THE BIRDS 3 FUNCTIONS
//THE FOLLOWING FUNCTIONS WERE DEVELOPED FOR THE EOBC WASKIRI-SAT




//--------MAIN PIC Buffer------------------------------------------------------

//!!!!!!!!!!!! Check the used UART ports!!!!!!!!!!!!!!!!!
//  PC(Debug) -> USART2
//  COM -> USART1
// RESET PIC -> USART3

static uint8_t in_bffr_main[16];
static uint8_t CMD_FROM_PC[6];
static volatile uint8_t COM_DATA = 0;

static void SerialDataReceive(void) // get buffer data one by one (16 bytes)
{
  // Blocking receive: exactly 16 bytes
  HAL_UART_Receive(&huart1, in_bffr_main, 16, HAL_MAX_DELAY);
}
static void DELETE_CMD_FROM_PC(void)
{
  //fills a block of memory with a byte value in this case 0x00.
  memset(CMD_FROM_PC, 0, sizeof(CMD_FROM_PC));
}
static void Delete_Buffer(void) 
{
  // delete com command buffer
  memset(in_bffr_main, 0x00, sizeof(in_bffr_main));
  COM_DATA = 0;
}
static void Transfer_Buffer(int PORT_NUM)
{
  switch (PORT_NUM)
  {
    // Structure of the HAL_UART_Transmit(UART Port, Message, Message Size, Timeout )
    
    //!!!!!!!!!!!! Check the used UART ports!!!!!!!!!!!!!!!!!
    
    case 1: // PC (debug) -> USART2
      HAL_UART_Transmit(&huart2, in_bffr_main, 16, 100);
      break;

    case 2: // COM -> USART1 back to PIC
      HAL_UART_Transmit(&huart1, in_bffr_main, 16, 100);
      break;

    case 3: // RESET PIC -> USART3
      HAL_UART_Transmit(&huart3, in_bffr_main, 16, 100);
      break;
  }
}

static void TRANSFER_Buffer_OF(uint32_t TO_ADDRESS)
{
  if (TO_ADDRESS + 16 > OF_MEM_SIZE)
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)"OF write: address out of range\r\n", 32, 100);
    return;
  }

  for (int i = 0; i < 16; i++)
  {
    OF_MEM[TO_ADDRESS] = in_bffr_main[i];
    TO_ADDRESS++;
  }
}
    // Extra functions for debugging
/*static void dbg_print_hex16(const uint8_t *b)
{
  char line[16*3 + 4];
  int idx = 0;
  for (int i = 0; i < 16; i++)
  {
    idx += snprintf(&line[idx], sizeof(line) - idx, "%02X ", b[i]);
  }
  line[idx++] = '\r';
  line[idx++] = '\n';
  HAL_UART_Transmit(&huart2, (uint8_t*)line, idx, 100);
}
static void send_ack(uint8_t cmd)
{
  uint8_t ack[4] = { 'A','C','K', cmd };
  HAL_UART_Transmit(&huart1, ack, sizeof(ack), 100);
}*/

//--------BC Function----------------------------------------------------------

// PC UART connection is Uart2

/* --- Beacon globals --- */
static uint16_t BC_TEMP_RAW      = 0;
static float    BC_TEMP_C        = 0.0f;
static float    BC_INITIALTEMP_C = 0.0f;
static float    BC_MAXTEMP_C     = 0.0f;

static uint8_t  BC_ATTEMPT_FLAG = 0;  // 0..4
static uint8_t  ANT_DEP_STATUS  = 0;  // 0=not deployed, 1=deployed
static float    BC_LAST_DELTA_C = 0.0f;

// Beacon HELPERS for Functions they are proper not BIRDS legacy

static void PC_Printf(const char *fmt, ...)
{
  char buf[220];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}

// Beacon legacy Functions

/* Ensure ADC1 configured for channel PA0 (IN0). CubeMX does most, but this keeps it robust. */
static void BC_SETUP(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel      = BC_ADC_CHANNEL;
  sConfig.Rank         = 1;
  /* Sampling time: increase if your source impedance is high */
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;   //High cycle sample time = more precision
  (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static uint16_t BC_ReadAdcRaw(void) //NOT Legacy but added to prove just ADC reading
{
  BC_SETUP();

  (void)HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0;
  }

  uint16_t raw = (uint16_t)(HAL_ADC_GetValue(&hadc1) & 0x0FFF); // 12-bit
  (void)HAL_ADC_Stop(&hadc1);
  return raw;
}

/* conversion: TempC = (V*100) - 50, where V is sensor voltage */
static float BC_RawToTempC(uint16_t raw)
{
  const float VDD = 3.30f;
  float v = ((float)raw / 4096.0f) * VDD;
  return (v * 100.0f) - 50.0f;
}

static void MEASURE_BC_TEMP(void)
{
  BC_TEMP_RAW = BC_ReadAdcRaw();  //Read the ADC value
  BC_TEMP_C   = BC_RawToTempC(BC_TEMP_RAW); // Make a conversion to know the temperature
}

static void BC_READ_TO_PC(void) // Send temperature and Raw ADC reading to the PC
{
  MEASURE_BC_TEMP();
  PC_Printf("RAW=%u  Temp=%6.2f C\r\n", BC_TEMP_RAW, BC_TEMP_C);
}

static void CHECK_BC_TEMP(void)
{
  MEASURE_BC_TEMP();
  if (BC_TEMP_C > BC_MAXTEMP_C) BC_MAXTEMP_C = BC_TEMP_C;
  PC_Printf("ADC=%4u  Temp=%6.2f C\r\n", BC_TEMP_RAW, BC_TEMP_C);
}
static void Turn_ON_BC(void)
{
  HAL_GPIO_WritePin(BC_EN_GPIO_Port, BC_EN_Pin, GPIO_PIN_SET);
  PC_Printf("BC: ON  (PD0=1)\r\n");
}

static void Turn_OFF_BC(void)
{
  HAL_GPIO_WritePin(BC_EN_GPIO_Port, BC_EN_Pin, GPIO_PIN_RESET);
  PC_Printf("BC: OFF (PD0=0)\r\n");
}
static void BC_OPERATION(void)
{
  PC_Printf("\r\n--- BC_OPERATION START ---\r\n");

  /* baseline */
  BC_MAXTEMP_C = -1000.0f;
  CHECK_BC_TEMP();
  BC_INITIALTEMP_C = BC_TEMP_C;
  PC_Printf("Initial Temp: %6.2f C\r\n", BC_INITIALTEMP_C);

  /* ON + monitor 30s (sample each 1s) */
  Turn_ON_BC();

  uint32_t t0 = HAL_GetTick();
  uint32_t last = t0;

  while ((HAL_GetTick() - t0) < 30000UL)
  {
    if ((HAL_GetTick() - last) >= 1000UL)
    {
      last += 1000UL;
      CHECK_BC_TEMP();
    }
    HAL_Delay(5);
  }

  Turn_OFF_BC();

  float delta = BC_MAXTEMP_C - BC_INITIALTEMP_C;
  BC_LAST_DELTA_C = delta;

  PC_Printf("Max Temp:     %6.2f C\r\n", BC_MAXTEMP_C);
  PC_Printf("Delta:        %6.2f C\r\n", delta);

  if (delta > 5.0f)
    PC_Printf("Result: PASS (delta > 5C)\r\n");
  else
    PC_Printf("Result: FAIL (delta <= 5C)\r\n");

  PC_Printf("--- BC_OPERATION END ---\r\n\r\n");
}
static void CLEAR_BC_FLAG(void)
{
  BC_ATTEMPT_FLAG = 0;
  PC_Printf("\r\nBC_ATTEMPT_FLAG cleared -> %u\r\n", BC_ATTEMPT_FLAG);
}

static void MAKE_BC_FLAG_1(void)
{
  BC_ATTEMPT_FLAG = 1;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 1\r\n");
}

static void MAKE_BC_FLAG_2(void)
{
  BC_ATTEMPT_FLAG = 2;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 2\r\n");
}

static void MAKE_BC_FLAG_3(void)
{
  BC_ATTEMPT_FLAG = 3;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 3\r\n");
}

static void MAKE_BC_FLAG_4(void)
{
  BC_ATTEMPT_FLAG = 4;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 4\r\n");
}

static void Antenna_Deploy(void)
{
  PC_Printf("\r\n=== Antenna_Deploy() ===\r\n");

  if (ANT_DEP_STATUS)
  {
    PC_Printf("Already deployed (ANT_DEP_STATUS=1). No action.\r\n");
    return;
  }

  if (BC_ATTEMPT_FLAG >= 4)
  {
    PC_Printf("Max attempts reached (BC_ATTEMPT_FLAG=%u). Aborting.\r\n", BC_ATTEMPT_FLAG);
    return;
  }

  // Mark that we are attempting ( increment / track attempts)
  BC_ATTEMPT_FLAG++;
  PC_Printf("Attempt #%u\r\n", BC_ATTEMPT_FLAG);

  // Run the beacon heating/verification routine
  BC_OPERATION();

  // Decide deployment success based on delta
  if (BC_LAST_DELTA_C > 5.0f)
  {
    ANT_DEP_STATUS = 1;
    PC_Printf("DEPLOY SUCCESS: delta=%6.2f C -> ANT_DEP_STATUS=1\r\n", BC_LAST_DELTA_C);
  }
  else
  {
    PC_Printf("DEPLOY FAIL: delta=%6.2f C -> ANT_DEP_STATUS=0\r\n", BC_LAST_DELTA_C);
  }
}



//--------FAB HK collection----------------------------------------------------

#define CW_SIZE        5
#define COM_BUF_SIZE   16
#define ACK_SIZE       24

#define FAB_SENSOR_SIZE   45
#define HK_SIZE          124

/* ---------- FAB buffers (kept for context; COM/CW does not require FAB) ---------- */
static uint8_t  in_HK[FAB_SENSOR_SIZE];
static uint8_t  HKDATA[HK_SIZE];
static volatile uint8_t  fab_rx_byte = 0;
static volatile uint16_t fab_rx_count = 0;
static volatile uint8_t  fab_collect_enable = 0;

/* ---------- COM buffers ---------- */
static volatile uint8_t  com_rx_byte = 0;
static volatile uint8_t  COM_DATA = 0;                 // set when buffer full
static uint8_t           in_bffr_main[COM_BUF_SIZE];   // like PIC code
static volatile uint8_t  com_idx = 0;

/* ---------- CW + ACK ---------- */
static uint8_t CW_FORMAT[CW_SIZE] = {0};               // 5 bytes

static uint8_t ACK_for_COM[ACK_SIZE] =
{
  0xAA, 0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0, 0xBB
};

//########### Herritage functions from BIRDS ###############

/* Clears ACK_for_COM[1..6] */
void REFLESH_CW_ACK_for_COM(void)
{
  for (int i = 1; i < 7; i++) ACK_for_COM[i] = 0x00;
}

/* Clears ACK_for_COM[12..22] */
void REFLESH_MSN_ACK_for_COM(void)
{
  for (int i = 12; i < 23; i++) ACK_for_COM[i] = 0x00;
}

/* Build and send CW response frame to COM:
   AA 50 CW0 CW1 CW2 CW3 CW4 ... BB  (24 bytes total) */
void CW_RESPOND(void)
{
  REFLESH_CW_ACK_for_COM();
  HAL_Delay(100);

  ACK_for_COM[0]  = 0xAA;
  ACK_for_COM[1]  = 0x50;
  ACK_for_COM[2]  = CW_FORMAT[0];
  ACK_for_COM[3]  = CW_FORMAT[1];
  ACK_for_COM[4]  = CW_FORMAT[2];
  ACK_for_COM[5]  = CW_FORMAT[3];
  ACK_for_COM[6]  = CW_FORMAT[4];
  ACK_for_COM[23] = 0xBB;

  HAL_UART_Transmit(&huart2, ACK_for_COM, ACK_SIZE, 200);

  /* Show it as hex text too (nice in Proteus terminal) */
  com_text("\r\nANS:");
  for (int i = 0; i < 23; i++) com_text("%02X,", ACK_for_COM[i]);
  com_text("%02X\r\n", ACK_for_COM[23]);
}
/* If in_bffr_main[4] == 0x50, respond and clear buffer */
static void Delete_Buffer(void)
{
  memset(in_bffr_main, 0, sizeof(in_bffr_main));
  com_idx = 0;
  COM_DATA = 0;
}

void CHECK_50_and_CW_RESPOND(void)
{
  if (in_bffr_main[4] == 0x50)
  {
    CW_RESPOND();
    Delete_Buffer();
    COM_DATA = 0;
  }
}

//--------ADCS MISSION---------------------------------------------------------








//--------FAB collection----------------------------------------------------

#define FAB_SENSOR_SIZE   45
#define HK_SIZE          124

/* ---------- FAB buffers ---------- */
static uint8_t  in_HK[FAB_SENSOR_SIZE];
static uint8_t  HKDATA[HK_SIZE];

static volatile uint8_t  fab_rx_byte = 0;
static volatile uint16_t fab_rx_count = 0;
static volatile uint8_t  fab_collect_enable = 0;


/* ---------- FAB RX interrupt callback ---------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (fab_collect_enable)
    {
      in_HK[fab_rx_count % FAB_SENSOR_SIZE] = fab_rx_byte;
      fab_rx_count++;
    }

    /* re-arm RX for next byte */
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&fab_rx_byte, 1);
  }
}



static void Delete_in_HK(void)
{
  memset((void*)in_HK, 0, sizeof(in_HK));
}

static void Delete_HKDATA(void)
{
  memset((void*)HKDATA, 0, sizeof(HKDATA));
}

static void Send_Command_to_FAB(uint8_t cmd)
{
  HAL_UART_Transmit(&huart2, &cmd, 1, 100);
}

static void COMMAND_TO_FAB(uint32_t delay_ms_after_first_byte)
{
  /* equivalent to:
     FAB_DATA = 0; enable_interrupts(INT_rda3); Send_Command_to_FAB(0x61);
     wait FAB_DATA != 0; waiting(delaytime); disable_interrupts(INT_rda3);
  */

  fab_rx_count = 0;
  fab_collect_enable = 1;

  Send_Command_to_FAB(0x61);
  dbg_printf("[MAIN] Sent 0x61 to FAB\r\n");

  /* wait until at least 1 byte arrives (like FAB_DATA != 0) */
  uint32_t t0 = HAL_GetTick();
  while (fab_rx_count == 0)
  {
    if ((HAL_GetTick() - t0) > 300) break; // timeout 300ms
  }

  /* then wait extra time to let the rest of the frame arrive */
  HAL_Delay(delay_ms_after_first_byte);

  fab_collect_enable = 0;
}

static void CHECK_HKDATA(uint8_t in_shift, uint32_t delay_ms)
{
  /* keeps the same indexing from MAIN PIC:
     for num=1..(10-in): HKDATA[num+9] = in_HK[num+2-in]
     then for num=9..(FAB_SENSOR_SIZE-3): HKDATA[num+11] = in_HK[num+2-in]
  */
  dbg_printf("[MAIN] CHECK_HKDATA(in=%u)\r\n", in_shift);

  Delete_HKDATA();
  HAL_Delay(delay_ms);

  /* first block */
  for (int num = 1; num < (11 - in_shift); num++)
  {
    uint8_t src = in_HK[num + 2 - in_shift];
    HKDATA[num + 9] = src; // (num + 5 + 4)
  }

  /* second block */
  for (int num = 9; num < (FAB_SENSOR_SIZE - 2); num++)
  {
    uint8_t src = in_HK[num + 2 - in_shift];
    HKDATA[num + 11] = src; // (num + 7 + 4)
  }

  /* reset  FAB_DATA = 0 */
  fab_rx_count = 0;
}

static uint8_t VERIFY_FABDATA(uint32_t delay_after_cmd_ms, uint32_t delay_before_parse_ms)
{
  /*
     try 3 times: COMMAND_TO_FAB(); then check in_HK[0..2] == 0x33 and align
  */
  for (int attempt = 0; attempt < 3; attempt++)
  {
    Delete_in_HK();

    COMMAND_TO_FAB(delay_after_cmd_ms);

    /* We expect a 45-byte frame; give a little time if it’s still arriving */
    uint32_t t0 = HAL_GetTick();
    while (fab_rx_count < FAB_SENSOR_SIZE)
    {
      if ((HAL_GetTick() - t0) > 400) break; // up to 400ms
    }

    dbg_printf("[MAIN] Attempt %d, rx_count=%u, first bytes: %02X %02X %02X\r\n",
               attempt + 1, fab_rx_count, in_HK[0], in_HK[1], in_HK[2]);

    if (in_HK[0] == 0x33)
    {
      CHECK_HKDATA(2, delay_before_parse_ms);
      return 1;
    }
    else if (in_HK[1] == 0x33)
    {
      CHECK_HKDATA(1, delay_before_parse_ms);
      return 1;
    }
    else if (in_HK[2] == 0x33)
    {
      CHECK_HKDATA(0, delay_before_parse_ms);
      return 1;
    }
  }
  return 0;
}






//--------CAM MISSION----------------------------------------------------------

typedef struct {              //Struct to handle Camera characteristics
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *pwrPort;
    uint16_t pwrPin;
} CameraSim_Handle;


#define CAM_CMD_START  0x54
#define CAM_ACK        0x61
#define CAM_DONE       0x62



  CameraSim_Handle cam = {  //Definition of the Camera properties, UART Port, GPIO pin
        .huart = &huart2,
        .pwrPort = GPIOD,     
        .pwrPin = GPIO_PIN_1  // STM -> wire to D1 
    };

//##############  Extra functions for the camera  #########

static void uart_flush(CameraSim_Handle *cam) {
    // Non-blocking drain of any pending bytes
    uint8_t b;
    while (HAL_UART_Receive(cam->huart, &b, 1, 0) == HAL_OK) {
        // discard the rest of the message after ACK is received
    }
}

//########### Native Functions from Birds3  ##########

void Turn_On_CAM(CameraSim_Handle *cam) {// Turn the corresponding GPIO on
    HAL_GPIO_WritePin(cam->pwrPort, cam->pwrPin, GPIO_PIN_SET);
}

void Turn_Off_CAM(CameraSim_Handle *cam) {// Turn the corresponding GPIO off
    HAL_GPIO_WritePin(cam->pwrPort, cam->pwrPin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef Send_Command_to_CAM(CameraSim_Handle *cam, uint8_t data) {
    return HAL_UART_Transmit(cam->huart, &data, 1, 50);
}

void CAM_SETTINGS(CameraSim_Handle *cam) {
    // power + small delays + clear buffers
    uart_flush(cam);
    Turn_On_CAM(cam);
    HAL_Delay(20);
    uart_flush(cam);
}

HAL_StatusTypeDef CAM_TEST_OPERATION(CameraSim_Handle *cam, uint8_t mode,
                                     uint32_t ackTimeoutMs,
                                     uint32_t doneTimeoutMs)
{
    uint8_t rx = 0;

    CAM_SETTINGS(cam);

    // Send command 0x54
    if (Send_Command_to_CAM(cam, CAM_CMD_START) != HAL_OK) return HAL_ERROR;

    // Wait for 0x61
    if (CHECK_ACK_FROM_CAM(cam, &rx, ackTimeoutMs) != HAL_OK) return HAL_TIMEOUT;
    if (rx != CAM_ACK) return HAL_ERROR;

    // Send mode
    if (Send_Command_to_CAM(cam, mode) != HAL_OK) return HAL_ERROR;

    // Expect echo == mode
    if (CHECK_ACK_FROM_CAM(cam, &rx, ackTimeoutMs) != HAL_OK) return HAL_TIMEOUT;
    if (rx != mode) return HAL_ERROR;

    // Wait for DONE 0x62
    if (CHECK_ACK_FROM_CAM(cam, &rx, doneTimeoutMs) != HAL_OK) return HAL_TIMEOUT;
    if (rx != CAM_DONE) return HAL_ERROR;

    return HAL_OK;
}