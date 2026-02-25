#ifndef VARIABLE_DEFINITIONS_H
#define VARIABLE_DEFINITIONS_H

#include "hal/device_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "hal/abstractions.h"
#include "hal/uart_constant_definitions.h"

//! Interrupt handling variables ====================================================================

uint8_t clock_update = false; //! If true, indicates that periodic functions should run


inline void sync()
{
    while (!clock_update) { }
}

typedef enum {
    mux_msnboss = 0,
    mux_adcs = 1,
    mux_aprs = 2,
    mux_osil = 3,
    mux_tum = 4,
    mux_eobc = 5,
} cpld_mux_sel;

char* mux_str_list[] = {
    "MBS",
    "ADC",
    "APR",
    "OSIL",
    "TUM",
    "EOB"
};

uint8_t mux_cpld_position = mux_msnboss; //! Current MUX position

uint8_t mux_lock = false;    // True if long operation is locking the mux
uint8_t verbose = false;     // True to set verbose mode

// Change mission boss MUX position
uint8_t mux_sel(cpld_mux_sel sel)
{
    if (mux_lock)
        return mux_cpld_position; // Do not change if locked

    // MUX_CPLD_SEL_2 is the MSB
    output_bit(MUX_CPLD_SEL_2, (sel >> 2) & 0x1); // Most significant bit
    output_bit(MUX_CPLD_SEL_1, (sel >> 1) & 0x1); // Middle bit
    output_bit(MUX_CPLD_SEL_0, (sel >> 0) & 0x1); // Least significant bit

    mux_cpld_position = sel;
    delay_ms(10);

    if (verbose)
        fprintf(PC, "|MUX changed to %s|", mux_str_list[mux_cpld_position]);

    return mux_cpld_position;
}


//! General use variables and constants =============================================================

#define SPACECRAFT_ID 0x57 //CURTIS 49; BIRDSRPM 57 - EM, 58 - FM

time_t current_time, previous_time; //! Keep the current (and previous) satellite timestamp
#define TIME_T_MAX 0x7FFFFFFF

#define T0 946684800        //! earliest time possible for RTC (Jan 1st 2000, 00:00:00)
#define T_ANTENNA 946686630 //! Antenna deployment time
#define T_ANTENNA_2 946687290  //! Secondary Antenna deployment time (Jan 1st 2000, 00:41:30)
#define Tn 2147483646       //! latest time possible for RTC (Jan 19th 2038, 3:14:06)

uint8_t uart_mux = 0; //! Software UART multiplexing

#define BUFF_LENGTH 25        //! Must accomodate messages of all UART sources that may be scheduled
#define MAX_LENGTH 108        //! Maximum UART length
#define TERMINAL_COLS 80      //! Number of columns in terminal
#define EMPTY_BLOCKS_LIMIT 64 //! Stop after reading n bytes of empty data on flash

uint8_t memory_busy = false;       //! True if flash memory is used by COM
uint8_t response_rx = 0;           //! Indicates a response was received
time_t reset_time = T0;            //! Stores the time of last reset
uint8_t rst_clock_update = 0;      //! Update local clock with reset pic
uint8_t rst_clock_updated = false; //! Becomes true after reset pic clock sync
uint8_t adcs_mode = 12;            //! ADCS desired mode (by OBC)

bool LEDFlag = true;
//! Telemetry =======================================================================================

typedef struct telemetry_time_str { //! Stores the time when telemetry is received
    time_t reset_time;
    time_t fab_time;
    time_t msn_time;
    time_t adcs_time;
    time_t aprs_time;
    time_t tum_time;
    time_t eobc_time;   
    time_t osil_time;
    time_t com_time;
} telemetry_time_str;

typedef struct telemetry_str { //! Stores the telemetry of the last period, before being written to flash
    uint8_t reset_time;
    uint8_t reset_message[MSG_LENGTH_RST - 12];
    uint8_t fab_time;
    uint8_t fab_message[MSG_LENGTH_FAB - 4];
    uint8_t msn_time;
    uint8_t msn_message[MSG_LENGTH_MSNBOSS-4];
    uint8_t adcs_time;
    uint8_t adcs_message[MSG_LENGTH_ADCS - 4];
    uint8_t aprs_time;
    uint8_t aprs_message[MSG_LENGTH_APRS - 4];
    uint8_t tum_time;
    uint8_t tum_message[MSG_LENGTH_TUM - 4];
    uint8_t eobc_time;
    uint8_t eobc_message[MSG_LENGTH_EOBC - 3];
    uint8_t osil_time;
    uint8_t osil_message[MSG_LENGTH_OSIL - 4];
    uint8_t com_time;
    //uint8_t com_message[MSG_LENGTH_COMM];
    uint16_t com_rssi;
    uint16_t com_temp;
    time_t obc_time;
    uint8_t master_footer[2];
} telemetry_str;

telemetry_str telemetry;
telemetry_time_str telemetry_time;

#define CW_PAGES 2
#define CW_LENGTH 6              //! in bytes
uint8_t cw[CW_PAGES][CW_LENGTH]; //! Stores the CW beacon string for the last 50s

//! Flash memory address mapping ====================================================================

#define BOOT_FLAGS_ADDRESS 0x00000000
#define OBC_FLAGS_ADDRESS BOOT_FLAGS_ADDRESS + MEMORY_PAGE_SIZE // 4KB
#define ADDR_FLAGS_ADDRESS OBC_FLAGS_ADDRESS + MEMORY_PAGE_SIZE // 8KB
#define SCHEDULED_CMD_ADDRESS ADDR_FLAGS_ADDRESS + MEMORY_PAGE_SIZE // 12KB

#define FLASH_ADDR_START ADDR_FLAGS_ADDRESS
#define FLASH_ADDR_END SCHEDULED_CMD_ADDRESS
#define FLASH_ADDR_DELTA 8 //! log and telemetry -- 4 bytes each

#define FLASH_LOG_START (1 * MEMORY_SECTOR_SIZE)
#define FLASH_LOG_END (17 * MEMORY_SECTOR_SIZE)
#define FLASH_LOG_DELTA 7 //! time,origin,command,return

#define FLASH_TELEMETRY_START (17 * MEMORY_SECTOR_SIZE)
#define FLASH_TELEMETRY_END (749 * MEMORY_SECTOR_SIZE) //25.11.20:: 1098
#define FLASH_TELEMETRY_DELTA sizeof(telemetry)
#define FLASH_TELEMETRY_SECTORS_PER_DAY 3

#define FLASH_OSIL_START (749 * MEMORY_SECTOR_SIZE)
#define FLASH_OSIL_END (1005 * MEMORY_SECTOR_SIZE)

#define FLASH_APRS_START (1005 * MEMORY_SECTOR_SIZE)
#define FLASH_APRS_END (1133 * MEMORY_SECTOR_SIZE)
// #define FLASH_APRS_DELTA 128

#define FLASH_TUM_LOG_START (1133 * MEMORY_SECTOR_SIZE)       // 0x00000000
#define FLASH_TUM_LOG_END   (1134 * MEMORY_SECTOR_SIZE)       // 0x00010000

#define FLASH_TUM_START (1134 * MEMORY_SECTOR_SIZE)
#define FLASH_TUM_END (1261 * MEMORY_SECTOR_SIZE)
// #define FLASH_TUM_DELTA 128

#define FLASH_EOBC_START (1261 * MEMORY_SECTOR_SIZE)
#define FLASH_EOBC_END (1517 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_HS_START (1517 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_HS_END (1524 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_GPS_START (1524 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_GPS_END (1529 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_TELEMETRY_START (1529 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_TELEMETRY_END (1785 * MEMORY_SECTOR_SIZE)

// #define FLASH_N1_START (1785 * MEMORY_SECTOR_SIZE)
// #define FLASH_N1_END (2027 * MEMORY_SECTOR_SIZE)

// #define FLASH_N2_START (2027 * MEMORY_SECTOR_SIZE)
// #define FLASH_N2_END (2037 * MEMORY_SECTOR_SIZE)

// #define FLASH_N3_START (2037 * MEMORY_SECTOR_SIZE)
// #define FLASH_N3_END (2048 * MEMORY_SECTOR_SIZE)

//! Flash memory structures =========================================================================

typedef struct flash_ctrl {
    uint32_t start;
    uint32_t end;
    uint32_t current;
    uint8_t delta;
} flash_ctrl;

//! Store the OBC boot variables
typedef struct bflags {
    uint8_t deployment_flag_1;
    uint8_t deployment_flag_2;
} bflags;

bflags boot_flags = { 0xFF, 0xFF };

//!Store the mission variables
typedef struct mflags {
    uint8_t aprs_on_off;                   //! 0: turn OFF 1: turn ON
    uint8_t osil_on_off;                    //! 0: turn OFF 1: turn ON
    uint8_t eobc_on_off;                   //! 0: turn OFF 1: turn ON
    uint8_t tum_on_off;                    //! 0: turn OFF 1: turn ON
    uint8_t aprs_mission_status;           //! 0: no mission 1: Digipeater 2:S&Fward
} mflags;

mflags mission_flags = { 0, 0, 0, 0, 1 };   //! Update to reference values

//! Store the OBC variables
typedef struct oflags {
    int8_t leap_seconds;
    uint8_t adcs_on_off;
    uint8_t msnboss_on_off;
    uint8_t led_on_off; //1: enabled, 0: off
    uint8_t trx_on_off;  //! Addnics com board power on/off
    uint8_t ntrx_on_off; //! New com board power on/off
    uint8_t gps_time_sync_state;
    int8_t adcs_initial_value;
    uint8_t cw_mode; //! 0: never send CW; 1: follow ADCS; 2: Send CW always
    uint8_t cw_trx_source; //00: Addnics TRX only; 01: New TRX only ; 02: Both TRX [default]            
    uint16_t heater_th_temperature; //! battery heater
    uint16_t heater_th_voltage;     //! battery heater
    uint8_t camera_parameters[11]; // store parameters for camera capture command (0xC0CA)
} oflags;

//! Update to reference values
oflags obc_flags = {
    0,     // leap_seconds
    1,     // adcs_on_off
    1,     // msnboss_on_off
    1,     // led_on_off
    1,     // trx_on_off
    1,     // ntrx_on_off
    0,     // gps_time_sync_state
    0,     // adcs_initial_value
    2,     // cw_mode
    2,     // cw_trx_source
    0xC3F, // heater_th_temperature
    0xC48, // heater_th_voltage
    { 0 }  // camera_parameters[15]
}; 

void print_flags()
{
    fprintf(PC, "leap_seconds = %d\r\n", obc_flags.leap_seconds);
    fprintf(PC, "adcs_on_off = %d\r\n", obc_flags.adcs_on_off);
    fprintf(PC, "msnboss_on_off = %d\r\n", obc_flags.msnboss_on_off);
    fprintf(PC, "led_on_off = %d\r\n", obc_flags.led_on_off);
    fprintf(PC, "Addnics TRX_on_off = %d\r\n", obc_flags.trx_on_off);
    fprintf(PC, "NewTRX_on_off = %d\r\n", obc_flags.ntrx_on_off);
    fprintf(PC, "gps_time_sync_state = %d\r\n", obc_flags.gps_time_sync_state);
    fprintf(PC, "adcs_initial_value = %d\r\n", obc_flags.adcs_initial_value);
    fprintf(PC, "cw_mode = %d\r\n", obc_flags.cw_mode);
    fprintf(PC, "cw_trx_source = %d\r\n", obc_flags.cw_trx_source);
    fprintf(PC, "heater_th_temperature = 0x%04lX\r\n", obc_flags.heater_th_temperature);
    fprintf(PC, "heater_th_voltage = 0x%04lX\r\n", obc_flags.heater_th_voltage);
     fprintf(PC, "camera_parameters = ");                                                 // uint8_t[]
    for(uint8_t i = 0; i < sizeof(obc_flags.camera_parameters); i++){
        fprintf(PC, "%02X", obc_flags.camera_parameters[i]);
    }
    fprintf(PC, "\r\n");
}
//! Store the address variables
typedef struct aflags {
    flash_ctrl flash_addr;
    flash_ctrl flash_log;
    flash_ctrl flash_telemetry;
} aflags;

aflags addr_flags = { 0 };

//! Satellite log ===================================================================================
#define MAX_LOGS_IN_RAM 64

//! A structure that defines a log entry.
typedef struct log_entry {
    time_t time;
    uint8_t origin;
    uint8_t command;
    uint8_t return_value;
} log_entry;

log_entry log_buffer[MAX_LOGS_IN_RAM]; //! Stores logs in RAM before they are flushed to flash
uint8_t log_index = 0;                 //! The index of current log in RAM

//! Scheduled commands ==============================================================================

#define SCHEDULED_COMMANDS_MAX 32 //! Requires 1600 bytes = 40 * (36 + 4)

//! A structure that defines a scheduled command.
typedef struct scheduled_command {
    time_t time; //! Unix time (seconds after Jan 1st 1970)
    uint8_t command[BUFF_LENGTH];
} scheduled_command;

scheduled_command scheduled_commands[SCHEDULED_COMMANDS_MAX];

//! Boot commands ===================================================================================

typedef struct boot_command { //! Struct to hold a boot command.
    time_t time;              //! Time to run the command after boot.
    uint8_t command[BUFF_LENGTH];
} boot_command;

#define BOOT_COMMANDS_MAX 8
#define BOOT_COMMANDS_ADDR 0 //! Dedicated memory

boot_command boot_commands[BOOT_COMMANDS_MAX];

//! ADCS ============================================================================================

#define STM32_SIZE 32
uint8_t stm32_command_uhf[STM32_SIZE];

#endif //! VARIABLE_DEFINITIONS_H
