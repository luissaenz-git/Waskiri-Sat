#ifndef INTERPRETER_H
#define INTERPRETER_H

#include "hal/board_configuration.h"
#include "variable_definitions.h"
#include "scheduler.h"
#include <math.h>
#include "crc16.h"
#include "log_control.h"
#include "boot_command.h"
#include "xmodem.h"

//! Interpreter: The procedures here are concerned with interpreting
//! received commands and executing the appropriate commands.

//! Definition of commands. Should follow the prototype: "uint8_t command_name(uint8_t *data)"
//! Return value = 0 indicates that the command was successful
//! Return value > 0 indicates that there was an error

//! ============ Helper functions ============

// Convert seconds since 2000-01-01 00:00:00 into calendar date/time
void seconds_to_datetime(time_t t,
                         uint8_t *yy, uint8_t *mm, uint8_t *dd,
                         uint8_t *hh, uint8_t *mi, uint8_t *ss)
{
    // Days per month in normal year
    static const uint8_t days_in_month[] = {
        31,28,31,30,31,30,31,31,30,31,30,31
    };

    // 1. Break down time
    *ss = t % 60;   t /= 60;
    *mi = t % 60;   t /= 60;
    *hh = t % 24;   t /= 24;

    // 2. Count days since 2000-01-01
    uint32_t days = (uint32_t)t;

    // 3. Compute year
    uint16_t year = 1970; // because epoch starts from 1970; T0 = 946684800 
    while (1) {
        uint16_t days_in_year = 365;
        // leap year check (2000 is divisible by 400, so leap)
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            days_in_year = 366;
        }
        if (days < days_in_year) break;
        days -= days_in_year;
        year++;
    }

    // 4. Compute month
    uint8_t month = 0;
    while (1) {
        uint8_t dim = days_in_month[month];
        // February leap-year adjustment
        if (month == 1 &&
            ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
            dim = 29;
        }
        if (days < dim) break;
        days -= dim;
        month++;
    }

    *yy = (uint8_t)(year - 2000);
    *mm = month + 1;
    *dd = days + 1;
}

uint8_t mux_lock_unlock(uint8_t mux_state, time_t time)
{
    if (time > 1800L) { // Maximum time that the mux can be reserved is 30 minutes
        fprintf(PC, "Warning: mux reservation time too long!");
        time = 1800L;
    }
    if (mux_state) {
        scheduled_command_clear_specified_command(0xC0, 0x02); // Disable scheduled command to regain access to memory in the future
        schedule(current_time + time, { 0xC0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 }); // Release the lock in the future
    }
    mux_lock = mux_state;
    return mux_state;
}

uint8_t command_mux_lock_unlock(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t mux_state; // 0: unlock, 1: lock
        time_t time;       // in seconds
    }* packet = (struct packet*)data;

    uart_print_pc_hex(data, 7);
    return mux_lock_unlock(packet->mux_state, packet->time);
}

uint8_t command_mux_sel_sfm(uint8_t* data)
{
    fprintf(PC, "Changed MUX to position %d", data[2]);
    return mux_sel(data[2]);
}


void get_com_shared_fm_access()
{
    if (memory_busy) {
        scheduled_command_clear_specified_command(0xC0, 0x58); //! Disable scheduled command to regain access to memory in the future
        output_low(MUX_SEL_COM_SHARED_FM);                     //! Regain access to memory now
        memory_busy = 0;                                       //! Now memory is free
    }
}

void get_msn_shared_fm_access()
{
    if (memory_busy) {
        scheduled_command_clear_specified_command(0xC0, 0x40); //! Disable scheduled command to regain access to memory in the future
        output_low(MUX_SEL_MSN_SHARED_FM);                     //! Main PIC Regain access to memory now
        memory_busy = 0;                                       //! Now memory is free
    }
}

// Helper function to copy data from flash memory to flash memory
uint8_t copy(
    uint8_t origin, // 0:COM, 1:MAIN, 2:MISSION
    uint8_t dest,   // 0:COM, 1:MAIN, 2:MISSION
    uint32_t to_page_or_sector_or_address,
    uint32_t from_page_or_sector_or_address,
    uint32_t size, // in page or sectors or sectors
    uint8_t mode   // mode: 0 = pages, 1 = sectors, 2 = addressed by byte, size as pages
)
{
    get_com_shared_fm_access();

    uint8_t erase_mode;
    uint32_t increment;

    if (mode == 0 || mode == 2) {
        erase_mode = ERASE_PAGE;
        increment = MEMORY_PAGE_SIZE;
    } else {
        erase_mode = ERASE_SECTOR;
        increment = MEMORY_SECTOR_SIZE;
    }

    uint32_t to_address = 0;
    uint32_t from_address = 0;
    if (mode == 2) {
        to_address = to_page_or_sector_or_address;
        from_address = from_page_or_sector_or_address;
    } else {
        to_address = to_page_or_sector_or_address * increment;
        from_address = from_page_or_sector_or_address * increment;
    }

    size *= increment;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX,mode=%d", origin, dest, to_address, from_address, size, mode);

    // Erase the pages before copying
    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += increment) {
            flash_erase(&spi_port_COM_FM, to_address + i, erase_mode);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += increment) {
            flash_erase(&spi_port_MAIN_FM, to_address + i, erase_mode);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += increment) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address + i, erase_mode);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    if (origin == 0x00 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address, &spi_port_COM_FM, to_address, size);

    } else if (origin == 0x00 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address, &spi_port_MAIN_FM, to_address, size);

    } else if (origin == 0x00 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address, &spi_port_MISSION_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x01 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address, &spi_port_COM_FM, to_address, size);

    } else if (origin == 0x01 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address, &spi_port_MAIN_FM, to_address, size);

    } else if (origin == 0x01 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address, &spi_port_MISSION_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x00) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address, &spi_port_COM_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x01) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address, &spi_port_MAIN_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address, &spi_port_MISSION_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else {
        return 1;
    }

    return 0;
}

//! Helper function to calculate OBC checksum
void checksum_obc(uint8_t* data, uint8_t size)
{
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < size - 2; i++) {
        checksum ^= data[i];
    }
    data[size - 2] = checksum;
    data[size - 1] = data[0] + 1; //! Footer
}

//! Helper function to check if uplink is valid
uint8_t uplink_valid(uint8_t* buffer)
{
    const uint8_t cmd_length = 22;        //! Extended packet length
    const uint8_t cmd_legacy_length = 14; //! Legacy packet length

    struct packet {
        uint8_t packet_format_id;
        uint8_t satellite_id;
        uint8_t cmd_format_id;
    }* packet = (struct packet*)buffer;

    uint16_t cr, pk;                                                                 //! these are the crc check variables
    if ((packet->packet_format_id == 0x42 || packet->packet_format_id == 0x54)   && packet->satellite_id == SPACECRAFT_ID) { //! This packet is meant for this satellite
        if (packet->cmd_format_id == 0xCC) {                                         //! Extended KITSUNE format (22 bytes)
            cr = mk_crc(buffer, cmd_length - 2);
            pk = make16(buffer[cmd_length - 1], buffer[cmd_length - 2]);
        } else { //! Herritage BIRDS format (14 bytes)
            cr = mk_crc(buffer, cmd_legacy_length - 2);
            pk = make16(buffer[cmd_legacy_length - 1], buffer[cmd_legacy_length - 2]);
        }
        if (cr == pk) { //! CRC is good to go
            return 1;   //! Packet is valid
        }
    }
    return 0; //! Packet is invalid
}

//! Helper function to send an acknowledge back to COM
void send_com_ack(uint8_t* data)
{
    uint8_t cmd[24] = { 0 };
    cmd[0] = MSG_OBC; //! OBC message header 0x0B
    cmd[1] = 0xAA;
    cmd[2] = 0xCC;
    memcpy(cmd + 3, data, 8);
    cmd[12] = 0x66;
    cmd[23] = 0x0C;
    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], COMM);
    }
}

//! Helper function to change reset time
void reset_pic_update_clock(time_t time)
{
    struct_tm* tstr = localtime(&time);
    struct rst_msg {
        uint8_t rst_command;
        uint8_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    } msg;

    msg.rst_command = 0x70;
    msg.year = tstr->tm_year - 100;
    msg.month = tstr->tm_mon + 1;
    msg.day = tstr->tm_mday;
    msg.hour = tstr->tm_hour;
    msg.minute = tstr->tm_min;
    msg.second = tstr->tm_sec;

    uint8_t i;
    uint8_t cmd[36] = { 0 };
    cmd[0] = 0xB0;
    uint8_t* ptr = (uint8_t*)&msg;
    for (i = 0; i < sizeof(msg); i++) {
        cmd[i + 1] = ptr[i];
    }
    cmd[35] = 0xB1;
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    uart_print_pc_hex(cmd, sizeof(cmd));
}

//! Helper function to print binary data
void print_binary16(uint16_t data)
{
    const uint8_t size = 16;
    for (uint8_t i = 1; i <= size; i++) {
        fprintf(PC, "%Lu", ((data >> (size - i)) & 1));
    }
    fputc('\r', PC);
    fputc('\n', PC);
}

//! Helper funtion to initialize the telemetry array.
void initialize_telemetry()
{
    memset(&telemetry_time, 0, sizeof(telemetry_time));
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.master_footer[0] = 0xB0;
    telemetry.master_footer[1] = 0x0B;
}



//! Convert gyro data 16 -> 8 bits (-128 to 127 with LSB = 1deg/s, with overflow protection)
int8_t gyro_to_cw(uint8_t msb, uint8_t lsb)
{
    int8_t gyro_cw;
    float gyro = ((int16_t)make16(msb, lsb)) * 8.75e-3;
    if (gyro > 127.) {
        gyro_cw = 127;
    } else if (gyro < -128.) {
        gyro_cw = -128;
    } else {
        if (gyro > 0.) {
            gyro_cw = (int8_t)(gyro + 0.5);
        } else {
            gyro_cw = (int8_t)(gyro - 0.5);
        }
    }
    return gyro_cw;
}


// Conversion for CW beacon in the range 0-15
uint8_t get_adcs_mode_index(uint8_t adcs_mode) {
    switch (adcs_mode) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            return adcs_mode;  // Direct mapping for 0 to 4
        case 10:
        case 11:
        case 12:
        case 13:
            return adcs_mode - 5;  // Adjust for 10 to 13
        case 20:
        case 21:
            return adcs_mode - 11; // Adjust for 20 to 21
        case 25:
        case 26:
        case 27:
        case 28:
        case 29:
            return adcs_mode - 14; // Adjust for 25 to 28
        default:
            return 0; // Return 0 if the mode is out of range or not mapped
    }
}

// Helper function for data serialization
void set_bits(uint8_t *data, uint32_t value, uint16_t *bit_offset, uint8_t bits) {
    while (bits > 0) {
        uint8_t byte_offset = *bit_offset / 8;
        uint8_t bit_in_byte_offset = *bit_offset % 8;
        uint8_t bits_to_write = (bits > 8 - bit_in_byte_offset) ? 8 - bit_in_byte_offset : bits;

        data[byte_offset] |= ((value & ((1U << bits_to_write) - 1U)) << bit_in_byte_offset);

        value >>= bits_to_write;
        bits -= bits_to_write;
        *bit_offset += bits_to_write;
    }
}

// Helper function for data deserialization
uint32_t get_bits(uint8_t *data, uint16_t *bit_offset, uint8_t bits) {
    uint32_t value = 0;
    uint8_t value_offset = 0;

    while (bits > 0) {
        uint8_t byte_offset = *bit_offset / 8;
        uint8_t bit_in_byte_offset = *bit_offset % 8;
        uint8_t bits_to_read = (bits > 8 - bit_in_byte_offset) ? 8 - bit_in_byte_offset : bits;

        value |= ((uint32_t)(data[byte_offset] >> bit_in_byte_offset) & ((1U << bits_to_read) - 1U)) << value_offset;

        bits -= bits_to_read;
        *bit_offset += bits_to_read;
        value_offset += bits_to_read;
    }

    return value;
}

void deserialize_cw()
{
    uint16_t bit_offset = 0;

    // Page 0 Deserialization
    uint8_t battery_voltage = get_bits(cw[0], &bit_offset, 8);                 // Battery Voltage (RST RAW voltage)
    uint8_t battery_current = get_bits(cw[0], &bit_offset, 8);                 // Battery Current
    uint8_t battery_temperature = get_bits(cw[0], &bit_offset, 8);             // Battery Temperature
    uint8_t cpld_temperature = get_bits(cw[0], &bit_offset, 8);                // CPLD Temperature
    uint8_t solar_cell_y_plus = get_bits(cw[0], &bit_offset, 1);               // Solar Cell +Y
    uint8_t solar_cell_x_plus = get_bits(cw[0], &bit_offset, 1);               // Solar Cell +X
    uint8_t solar_cell_y_minus = get_bits(cw[0], &bit_offset, 1);              // Solar Cell -Y
    uint8_t solar_cell_z_minus = get_bits(cw[0], &bit_offset, 1);              // Solar Cell -Z
    uint8_t scheduled_commands = get_bits(cw[0], &bit_offset, 2);              // Scheduled commands in memory
    uint8_t battery_heater_flag = get_bits(cw[0], &bit_offset, 1);             // Battery Heater Flag
    uint8_t kill_switch_main_pic = get_bits(cw[0], &bit_offset, 1);            // Kill Switch Main PIC
    uint8_t kill_switch_eps = get_bits(cw[0], &bit_offset, 1);                 // Kill Switch EPS
    uint8_t adcs_mode = get_bits(cw[0], &bit_offset, 4);                       // ADCS mode
    uint8_t format_identifier_0 = get_bits(cw[0], &bit_offset, 1);             // Format identifier

    fprintf(PC, "Page 0 Deserialized Data:\r\n");
    fprintf(PC, "Battery Voltage (RST RAW voltage): %u\r\n", battery_voltage);
    fprintf(PC, "Battery Current: %u\r\n", battery_current);
    fprintf(PC, "Battery Temperature: %u\r\n", battery_temperature);
    fprintf(PC, "CPLD Temperature: %u\r\n", cpld_temperature);
    fprintf(PC, "Solar Cell +Y: %u\r\n", solar_cell_y_minus);
    fprintf(PC, "Solar Cell +X: %u\r\n", solar_cell_x_plus);
    fprintf(PC, "Solar Cell -Y: %u\r\n", solar_cell_y_minus);
    fprintf(PC, "Solar Cell -Z: %u\r\n", solar_cell_z_minus);
    fprintf(PC, "Scheduled commands in memory: %u\r\n", scheduled_commands);
    fprintf(PC, "Battery Heater Flag: %u\r\n", battery_heater_flag);
    fprintf(PC, "Kill Switch Main PIC: %u\r\n", kill_switch_main_pic);
    fprintf(PC, "Kill Switch EPS: %u\r\n", kill_switch_eps);
    fprintf(PC, "ADCS mode: %u\r\n", adcs_mode);
    fprintf(PC, "Format identifier: %u\r\n", format_identifier_0);

    bit_offset = 0; // Reset bit offset for page 1

    // Page 1 Deserialization
    uint8_t gyro_x = get_bits(cw[1], &bit_offset, 8);                   // Gyro X axis
    uint8_t gyro_y_minus = get_bits(cw[1], &bit_offset, 8);             // Gyro -Y axis
    uint8_t gyro_z_minus = get_bits(cw[1], &bit_offset, 8);             // Gyro -Z axis
    uint8_t magnetometer_x = get_bits(cw[1], &bit_offset, 5);           // Magnetometer X
    uint8_t magnetometer_y = get_bits(cw[1], &bit_offset, 5);           // Magnetometer Y
    uint8_t magnetometer_z = get_bits(cw[1], &bit_offset, 5);           // Magnetometer Z
    uint8_t subsystems_communicating = get_bits(cw[1], &bit_offset, 3); // Subsystems communicating
    uint8_t time_after_reset = get_bits(cw[1], &bit_offset, 5);         // Time after last reset (hours)
    uint8_t format_identifier_1 = get_bits(cw[1], &bit_offset, 1);      // Format identifier

    fprintf(PC, "Page 1 Deserialized Data:\r\n");
    fprintf(PC, "Gyro X axis: %u\r\n", gyro_x);
    fprintf(PC, "Gyro -Y axis: %u\r\n", gyro_y_minus);
    fprintf(PC, "Gyro -Z axis: %u\r\n", gyro_z_minus);
    fprintf(PC, "Magnetometer X: %u\r\n", magnetometer_x);
    fprintf(PC, "Magnetometer Y: %u\r\n", magnetometer_y);
    fprintf(PC, "Magnetometer Z: %u\r\n", magnetometer_z);
    fprintf(PC, "Subsystems communicating: %u\r\n", subsystems_communicating);
    fprintf(PC, "Time after last reset (hours): %u\r\n", time_after_reset);
    fprintf(PC, "Format identifier: %u\r\n", format_identifier_1);
}

//! Helper funtion to initialize the cw beacon array.
void build_cw()
{
    uint8_t sc = scheduled_command_count();
    uint8_t adcs_mode = telemetry.adcs_message[0];
    time_t time_after_reset = (current_time - reset_time) / 3600;
    uint8_t time_after_reset_ = time_after_reset > 24 ? 0x1F : time_after_reset;

    memset(&cw, 0, sizeof(cw)); //! Erase old data.

    //! Page 0
    cw[0][0] = (telemetry.reset_message[6] << 4) | (telemetry.reset_message[7] >> 4);      //! Battery voltage
    cw[0][1] = (telemetry.fab_message[48] << 4) | (telemetry.fab_message[49] >> 4);        //! Battery current, 12 -> 8 bit
    cw[0][2] = (telemetry.fab_message[50] << 4) | (telemetry.fab_message[51] >> 4);        //! Battery temperature, 12 -> 8 bit
    cw[0][3] = (telemetry.fab_message[6] << 4) | (telemetry.fab_message[7] >> 4);          //! CPLD temperature
    cw[0][4] = (make16(telemetry.fab_message[28], telemetry.fab_message[29]) > 0x229) << 7 //! +X sun / no sun
        | (make16(telemetry.fab_message[34], telemetry.fab_message[35]) > 0x223) << 6      //! -X sun / no sun
        | (make16(telemetry.fab_message[26], telemetry.fab_message[27]) > 0x2E8) << 5      //! +Y sun / no sun
        | (make16(telemetry.fab_message[30], telemetry.fab_message[31]) > 0x03E) << 4      //! -Y sun / no sun
        | (make16(telemetry.fab_message[36], telemetry.fab_message[37]) > 0x081) << 3      //! +Z sun / no sun
        | (make16(telemetry.fab_message[32], telemetry.fab_message[33]) > 0x065) << 2      //! -Z sun / no sun
        | ((time_after_reset_ >> 4) & 0x1);                                                //! Time after reset bit 5
    cw[0][5] = (time_after_reset_ & 0x0F) << 4                                             //! Time after reset bits 0-4
        | (telemetry.fab_message[52] & 0x1) << 3                                           //! Battery heater on/off
        | (telemetry.fab_message[53] & 0x1) << 2                                           //! Kill switch Main PIC
        | ((telemetry.fab_message[53] & 0x10) >> 4) << 1                                   //! Kill switch EPS PIC
        | 0x0;      //1 bit free for assignmnet here                                       //! Format identifier

    //! Page 1
    cw[1][0] = gyro_to_cw(telemetry.adcs_message[5], telemetry.adcs_message[6]); //! Gyro X axis (deg/s), 16 -> 8 bits (-128 to 127 with LSB = 1deg/s)
    cw[1][1] = gyro_to_cw(telemetry.adcs_message[7], telemetry.adcs_message[8]); //! Gyro Y axis (deg/s), 16 -> 8 bits (-128 to 127 with LSB = 1deg/s)
    cw[1][2] = gyro_to_cw(telemetry.adcs_message[9], telemetry.adcs_message[10]); //! Gyro Z axis (deg/s), 16 -> 8 bits (-128 to 127 with LSB = 1deg/s)
    cw[1][3] = ((telemetry.adcs_message[13] >> 2) & 0x1) << 7                    //! Magnetometer X axis sign bit
        | ((telemetry.adcs_message[15] >> 1) & 0x1) << 6                         //! Magnetometer Y axis sign bit
        | (telemetry.adcs_message[17] & 0x1) << 5                                //! Magnetometer Z axis sign bit
        | (adcs_mode < 8 ? adcs_mode : 7) << 2                                   //! ADCS mode
        | ((sc < 4 ? sc : 3) & 0x03);                                            //! No. of scheduled commands

    cw[1][4] = ((telemetry_time.reset_time > 0)
                   + (telemetry_time.fab_time > 0)
                   + (telemetry_time.msn_time > 0)
                   + (telemetry_time.adcs_time > 0)
                   + (telemetry_time.aprs_time > 0)
                   + (telemetry_time.osil_time > 0)
                   + (telemetry_time.tum_time > 0)
                   + (telemetry_time.eobc_time > 0))<< 4                        //! Number of subsystems communicating with OBC
        | (boot_flags.deployment_flag_1 >= 10) << 3                              //! main antenna flag
        | (boot_flags.deployment_flag_2 >= 10) << 2                              //! secondary antenna flag
        | ((telemetry.msn_message[1]  & (1 << 4)) ? 1 : 0) <<1                  //! secondary antenna detector#1 UHF
        | ((telemetry.msn_message[1]  & (1 << 7)) ? 1 : 0);                   //! secondary antenna detector#4 UHF                                
   

    cw[1][5] =  0//! 7 bits free for assignment here
        | 0x1;   //! Format identifier

    fprintf(PC, "CW: 0x");
    uart_print_pc_hex_short(cw[0], sizeof(cw[0]));
    fprintf(PC, " 0x");
    uart_print_pc_hex_short(cw[1], sizeof(cw[1]));
    fputc(' ', PC);
}

void save_flags()
{
    uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
    flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
    flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));
}

//! Helper function to save state to flash
void save_state(uint8_t current_command)
{
    get_com_shared_fm_access();

    //! Save state of obc_flags:
    flash_erase_pages(&spi_port_COM_FM, OBC_FLAGS_ADDRESS, OBC_FLAGS_ADDRESS + sizeof(obc_flags));
    uint8_t* obc_flag_ptr = (uint8_t*)&obc_flags;
    flash_transfer_data_from_ram(
        &spi_port_COM_FM,
        OBC_FLAGS_ADDRESS,
        obc_flag_ptr,
        sizeof(obc_flags));

    //! Disable the current command before saving
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        if (scheduled_commands[i].command[0] == MSG_COMM && scheduled_commands[i].command[1] == current_command && scheduled_commands[i].time <= current_time) {
            scheduled_commands[i].time = TIME_T_MAX; //! Disable the command from executing again (== reschedule it at infinity).
        }
    }

    //! Save state of scheduled commands:
    flash_erase_pages(&spi_port_COM_FM, SCHEDULED_CMD_ADDRESS, SCHEDULED_CMD_ADDRESS + sizeof(scheduled_commands));
    uint8_t* cmd_ptr = (uint8_t*)scheduled_commands;
    flash_transfer_data_from_ram(&spi_port_COM_FM, SCHEDULED_CMD_ADDRESS, cmd_ptr, sizeof(scheduled_commands));
}

//Checks if it is time to turn off the LEDs
void ledBlinking()
{   
    if ( obc_flags.led_on_off == 1)
        output_Toggle(EN_LED);
    struct_tm* local_time = localtime(&current_time);
        //LEDs check: to be on for the first 2 minutes, then turn off and set the flag to 0
        // if (local_time->tm_hour == 00 && local_time->tm_min <= 2 && obc_flags.led_on_off == 1) {
        //    //fprintf(PC, "time is less than 2 minutes and LEDs flag is on\r\n");
        //    //output_Toggle(EN_LED);   
        // }
        // else
    if (local_time->tm_min > 3 && obc_flags.led_on_off == 1)
    {
        fprintf(PC, "Time is over 3 minutes, disabling LEDs flag\r\n");
        output_low(EN_LED);   
        obc_flags.led_on_off = 0;
        save_state(0xFC); //passing print flags command (any command)
    }
    // else
    //     fprintf(PC, "time is less than 3 minutes, LEDs are blinking\r\n");
        
}

//helper function for sending commands to mission boss
void send_mb_command(uint8_t cmd_id, uint16_t data, uint8_t silent)
{
    if (mux_sel(mux_msnboss) != mux_msnboss) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return;
    }

    uint8_t cmd[MSG_LENGTH_MSNBOSS] = { 0 };

    struct mb_packet {
        uint8_t origin;
        uint8_t command;
        uint16_t data;
        uint8_t checksum;
        uint8_t footer;
    }* mb_packet = (struct mb_packet*)cmd;

    mb_packet->origin = MSG_OBC; //! OBC Message header 0x0B
    mb_packet->command = cmd_id;
    mb_packet->data = data;
    
    checksum_obc(cmd, sizeof(cmd)); // checksum

    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], MSNBOSS);
    }

    if (!silent) {
        fprintf(PC, "MSNBOSS cmd: ");
        uart_print_pc_hex(cmd, sizeof(cmd));
    }
}


//! ============ Commands for Telemetry request ============

//! Request for reset telemetry
uint8_t command_request_reset(uint8_t* data)
{
    uart_clean(RST);
    uart_mux = 1;
    const uint8_t cmd[36] = {
        0xB0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1
    };
    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    return 0;
}

//! Request for eps telemetry
uint8_t command_request_eps(uint8_t* data)
{
    uart_clean(FAB);
    uart_mux = 1;

    uint8_t i;
    enum { cmd_size = 6 };

    uint8_t cmd[cmd_size] = { 0 };

    cmd[0] = MSG_FAB;
    cmd[1] = 0x61;
    *(uint32_t*)&cmd[2] = current_time;

    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], FAB);
    }

    return 0;
}

// //! ask com for telemetry
// uint8_t command_request_com(uint8_t* data)
// {
//     uart_clean(COMM);
//     uart_mux = 1;

//     uint8_t i;
//     enum { cmd_size = 24 };

//     uint8_t cmd[cmd_size] = { 0 };

//     cmd[0] = MSG_OBC; //! OBC message header expected 0x0B
//     cmd[1] = 0xCA; //! OBC message ID
//     cmd[2] = 0x50; //! OBC message ID
//     cmd[23] = 0x0C; //! OBC message footer expected

//     fprintf(PC, "\r\nRequesting telemetry from COM PIC...\r\n");

//     for (i = 0; i < sizeof(cmd); i++) {
//         fputc(cmd[i], COMM); // ack should be c0 50 to trigger data cw printing
//     }

//     return 0;
// }


//! ================================================================================================
//! ========= MISSION PAYLOAD Commands (will be moved to after mission boss) =======================
//! ================================================================================================

//! ============ OSIL Commands ============
//! request for OSIL camera mission telemetry 
uint8_t command_request_osil(uint8_t* data)
{ 
    
    if (mux_sel(mux_osil) != mux_osil) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
        }
       return 1;
    } 
    uart_clean(OSIL);
    
    
    enum { cmd_size = 12 };

    uint8_t cmd[cmd_size] = { 0 };

    cmd[0] = MSG_OBC; //! OBC message header expected 0x0B
    cmd[1] = 0xDD; //! OBC message ID
    checksum_obc(cmd, sizeof(cmd)); //! Calculate checksum
    
    //fprintf(PC, "\r\nRequesting telemetry from OSIL OSIL...\r\n");
    
    for (uint8_t i = 0; i < cmd_size; i++) {
        fputc(cmd[i], OSIL);
    }

    return 0;

}

//! Receive OSIL mission telemetry and print to screen
uint8_t command_osil_tlm_log (uint8_t* data)
{
    response_rx = 1; //! Received a reply
    uart_mux = 1;
    
    fprintf(PC, "OSIL: ");
    // fprintf(PC, "Mission Mode=%X | ", data[7]);
    uart_print_pc_hex(data, MSG_LENGTH_OSIL);

    telemetry_time.osil_time = current_time;
    memcpy(telemetry.osil_message, data + 2, sizeof(telemetry.osil_message));

    return 0;
}

//! Sends a command to the OSIL camera and waits for an ACK
uint8_t command_generic_osil_camera(uint8_t* data)
{
        if (mux_sel(mux_osil) != mux_osil) { // If MUX did not change
            if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
            }
            return 1;
        } 

    uart_clean(OSIL); // Clear UART OSIL buffer
    uint8_t i;
    enum { cmd_length = 12 };
    struct packet {
        uint8_t command;           
        uint8_t cam_cmd;         
        uint8_t mode;        //! 0xD0: default image capture; 0xD1: default target image capture
        uint8_t config[cmd_length-4]; //! Configuration data for the command
    }* packet = (struct packet*)data;

    uint8_t cmd[cmd_length] = { 0 };
   const uint8_t n_tries = 3; // Number of total tries for the procedure
   uint8_t max_timeout = 5;

    cmd[0] = MSG_OBC; // OBC Message header 0x0B
    cmd[1] = packet->mode;

    // Copy the configuration data into the command
    if(packet->mode == 0xC0 || packet->mode == 0xC1 || packet->mode == 0xA0 || packet->mode == 0xFF) {
        for (i = 0; i < cmd_length - 4; i++) {
        cmd[i + 2] = packet->config[i];
        }
    }
    

    checksum_obc(cmd, sizeof(cmd)); // Checksum 
    
    uart_print_pc_hex(cmd, sizeof(cmd));

    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d\r\n", attempt + 1);

        // Send command
        for (uint8_t j = 0; j < sizeof(cmd); j++) {
            fputc(cmd[j], OSIL);
        }

        fprintf(PC, "OSIL: ");
        uart_print_pc_hex(cmd, sizeof(cmd));
        fprintf(PC, "\r\n");

        // wait for ACK
        uint8_t timeout_counter = 0;
        while (timeout_counter++ < max_timeout) {
            if (kbhit(OSIL)) {
                uint8_t ACK = fgetc(OSIL);
                if (ACK == 0xAB) {
                    fprintf(PC, "ACK received: %02X\r\n", ACK);
                    return 0; // Success
                }
            }
            delay_ms(1000);
        }
    }

    fprintf(PC, "Failed to get an ACK after %d tries.\r\n", n_tries);
    return 0xFF; // Failure
}


//! ============ Caburei Commands ============
//! Request for Caburei APRS mission telemetry
uint8_t command_request_aprs(uint8_t* data)
{
    uart_clean(APRS);
    enum { cmd_length = 12 };
    if (mux_sel(mux_aprs) != mux_aprs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
        }
        return 1;
    } 
    
    const uint8_t cmd[cmd_length] = {0x0B, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB, 0x0C};
   //fprintf(PC, "\r\nRequesting telemetry from APRS...\r\n");
   for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], APRS);
    }
    return 0;
}

uint8_t command_aprs_tlm_log(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    fprintf(PC, "CABUREI: ");
    uart_print_pc_hex(data, MSG_LENGTH_APRS);
    telemetry_time.aprs_time = current_time;
    memcpy(telemetry.aprs_message, data + 2, sizeof(telemetry.aprs_message));
    return 0;
}


//! Sends command to Caburei mission
uint8_t command_generic_aprs(uint8_t* data)
{
       if (mux_sel(mux_aprs) != mux_aprs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1; // MUX change failed
    }
    
    uart_clean(APRS);

    enum { cmd_length = 12 };
    struct packet {
        uint8_t command;           //! 0B: MSG_OBC 
        uint8_t aprs_cmd;          //! TODO: remove later
        uint8_t command_id;        //! command identifier
        uint8_t config[cmd_length-4];        //! configuration data, TODO: remove later
    }* packet = (struct packet*)data;

    uint8_t cmd[cmd_length] = { 0 };
    cmd[0] = MSG_OBC; // OBC Message header 0x0B
    cmd[1] = packet->command_id; // command_id
 
    uint8_t max_timeout = 5;
    const uint8_t n_tries = 3; // Number of total tries for the procedure

    for(uint8_t i = 0; i < cmd_length - 4; i++) {
        cmd[i + 2] = packet->config[i];
    }

    checksum_obc(cmd, sizeof(cmd)); // checksum
    uart_print_pc_hex(cmd, sizeof(cmd));
    fprintf(PC, "\r\n");

        for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d...\r\n", attempt + 1);

        // Send command
        for (uint8_t j = 0; j < sizeof(cmd); j++) {
            fputc(cmd[j], APRS);
        }

        fprintf(PC, "APRS: ");
        uart_print_pc_hex(cmd, sizeof(cmd));
        fprintf(PC, "\r\n");

        // wait for ACK
        uint8_t timeout_counter = 0;
        while (timeout_counter++ < max_timeout) {
            if (kbhit(APRS)) {
                uint8_t ACK = fgetc(APRS);
                if (ACK == 0xA4) {
                    fprintf(PC, "ACK received: %02X\r\n", ACK);
                    return 0; // Success
                }
            }
            delay_ms(1000);
        }
    }

    fprintf(PC, "Failed to get an ACK after %d tries.\r\n", n_tries);
    return 0xFF; // Failure    
}

//! APRS copy packet data
uint8_t command_aprs_packet_copy(uint8_t* data)
{
    struct packet {
        uint8_t origin;  //! CO
        uint8_t command; //! B1
        uint8_t slots_30m;
    }* packet = (struct packet*)data;

    uint32_t source_address = (1831UL + 6.23 * (uint32_t)packet->slots_30m) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_APRS_START;
    const uint32_t n_packets = 6.23 * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy APRS HS data to Main PIC\r\n");
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  //! C0
        uint8_t command; //! B1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xB1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 2;      //! APRS
    xmodem_command.destination = 0; //! 0: COM Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);
    return 0;
}


//! ============ EOBC Commands ============

//! Request for Kyutech EOBC telemetry
uint8_t command_request_eobc(uint8_t* data)
{
    uart_clean(EOBC);
    enum { cmd_size = 9 };

    if (mux_sel(mux_eobc) != mux_eobc) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
        }
        return 1;
    } 

    const uint8_t cmd[cmd_size] = {0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C};
   // fprintf(PC, "\r\nRequesting telemetry from EOBC...\r\n");
    for (uint8_t i = 0; i < cmd_size; i++) {
        fputc(cmd[i], EOBC);
    }
    return 0;
}


uint8_t command_eobc_tlm_log(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    fprintf(PC, "eOBC: ");
    uart_print_pc_hex(data, MSG_LENGTH_EOBC);// eOBC requires 21 bytes not 20 
    telemetry_time.eobc_time = current_time;
    memcpy(telemetry.eobc_message, data + 2, sizeof(telemetry.eobc_message));
    return 0;
}

//! Passes a command from ground-station to EOBC and receive acknowledgement
uint8_t command_generic_eobc(uint8_t* data)
{
    uart_clean(EOBC);
    enum { cmd_length = 9 };

    if (mux_sel(mux_eobc) != mux_eobc) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
        }
       return 1;
    } 
   
    const uint8_t n_tries = 3;
    const uint8_t max_timeout = 5; // In seconds

    uint8_t cmd[cmd_length] = {0};
    cmd[0] = MSG_OBC;
    
    // Copy payload directly
    for (uint8_t i = 1; i < cmd_length; i++) {
        cmd[i] = data[i + 1];
    }

    if(cmd[1]== 0x01) //Realtime uplink
        {
            uint8_t yy, mm, dd, hh, mi, ss;
        seconds_to_datetime(current_time, &yy, &mm, &dd, &hh, &mi, &ss);
        fprintf(PC, "\r\nSending current time to eOBC: %02u-%02u-%02u %02u:%02u:%02u\r\n",
            yy, mm, dd, hh, mi, ss);
        cmd[2] = yy;
        cmd[3] = mm;
        cmd[4] = dd;
        cmd[5] = hh;
        cmd[6] = mi;
        cmd[7] = ss;
        }
    
    //checksum_obc(cmd, cmd_length); // Calculate checksum
     cmd[8] = MSG_OBC +1;   
    uart_print_pc_hex(cmd, sizeof(cmd));
    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d\r\n", attempt + 1);

        // Send command
        for (uint8_t j = 0; j < sizeof(cmd); j++) {
            fputc(cmd[j], EOBC);
        }

        fprintf(PC, "EOBC: ");
        uart_print_pc_hex(cmd, sizeof(cmd));
        fprintf(PC, "\r\n");
        // wait for ACK
        uint8_t timeout_counter = 0;
        while (timeout_counter++ < max_timeout) {
            if (kbhit(EOBC)) {
                uint8_t ACK = fgetc(EOBC);
                if (ACK == 0xAC) {
                    fprintf(PC, "ACK received: %02X\r\n", ACK);
                    return 0; // Success
                }
            }
            delay_ms(1000);
        }
    }

    fprintf(PC, "Failed to get an ACK after %d tries.\r\n", n_tries);
    return 0xFF; // Failure
}

//! ============ ADCS Commands ============

//! request for adcs telemetry
uint8_t command_request_adcs(uint8_t* data)
{
    uart_clean(ADCS);
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        } 
       return 1;
    } 
    uart_mux = 1;

    uint8_t i;
    enum { cmd_size = 12 };

    uint8_t cmd[cmd_size] = { 0 };

    cmd[0] = MSG_OBC; //! OBC message header 0x0B
    cmd[1] = 0xAB;
    // *(uint32_t*)&cmd[2] = current_time;
    cmd[11] = MSG_OBC + 1;
    //fprintf(PC, "\r\nRequesting ADCS telemetry...\r\n");
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

//! request for adcs gps time
uint8_t command_request_gps_time(uint8_t* data)
{
    uart_clean(ADCS);
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        } 
       // return 1;
    } 
    uart_mux = 1;

    uint8_t i;
    enum { cmd_size = 6 };

    uint8_t cmd[cmd_size] = { 0 };

    cmd[0] = MSG_OBC; //! OBC message header 0x0B
    cmd[1] = 0xAC; //! ADCS gps time request message ID
    fprintf(PC, "\r\nRequesting GPS time from ADCS...\r\n");
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

//! logging ADCS telemetry
uint8_t command_adcs_telemetry(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    uart_mux = 1;

    fprintf(PC, "ADCS: ");
    fprintf(PC, "Mode=%X | ", data[2]);
    uart_print_pc_hex(data, MSG_LENGTH_ADCS);
    //fprintf(PC, " ADCS tlm recieved "); //! ADCS telemetry received ack, not here originally

    telemetry_time.adcs_time = current_time;
    memcpy(telemetry.adcs_message, data + 2, sizeof(telemetry.adcs_message));

    return 0;
}


enum { //! updated on 2025/08/22
    adcs_mode_detumbling = 0,
    adcs_mode_nominal = 1,
    adcs_mode_sun_pointing = 2,
    adcs_mode_nadir_pointing = 3,
    adcs_mode_high_precision_pointing = 4,
    adcs_mode_gps_pointing = 5,
    adcs_mode_low_consumption = 6
};

//! Helper function to send STM32 commands (up to 32-bytes)
void stm32_raw_command(uint8_t* data, uint8_t length, uint8_t tle)
{
    uint8_t adcs_command[MSG_LENGTH_ADCS] = { 0x0B, 0x06 };
    if (tle) {
        adcs_command[1] = 0x07;
    }
    memcpy(adcs_command + 2, data, length);
    checksum_obc(adcs_command, sizeof(adcs_command));
    for (uint8_t i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }
    fprintf(PC, "STM RAW CMD: ");
    uart_print_pc_hex(adcs_command, sizeof(adcs_command));
}

//! Change ADCS mode internally and externally
uint8_t command_adcs_request_gps(uint8_t* data)
{
    uart_clean(ADCS);
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        } 
       return 1;
    } 
    uart_mux = 1;

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent; //! when true, change mode permanently
    }* packet = (struct packet*)data;

    uint8_t adcs_command[12] = { 0 };

    adcs_mode = packet->mode;

    adcs_command[0] = MSG_OBC;
    adcs_command[1] = packet->command;
    adcs_command[11] = MSG_OBC + 1;
    
    uart_print_pc_hex(adcs_command, sizeof(adcs_command));

    for (uint8_t i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }

    return packet->mode;
}

//! Change ADCS mode internally and externally
uint8_t command_adcs_mode(uint8_t* data)
{
    uart_clean(ADCS);
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        } 
       return 1;
    } 
    uart_mux = 1;

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t customParameter;
        uint8_t permanent; //! when true, change mode permanently
    }* packet = (struct packet*)data;

    uint8_t adcs_command[12] = { 0 };

    adcs_mode = packet->mode;

    adcs_command[0] = MSG_OBC;
    adcs_command[1] = packet->command;
    adcs_command[2] = packet->mode;
    adcs_command[3] = packet->customParameter;
    adcs_command[4] = packet->permanent;
    adcs_command[11] = MSG_OBC + 1;
    
    uart_print_pc_hex(adcs_command, sizeof(adcs_command));

    for (uint8_t i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }

    return packet->mode;
}

//! Change ADCS default mode
uint8_t command_adcs_default_mode(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t new_mode;
    }* packet = (struct packet*)data;

    //! Change stored value in OBC
    obc_flags.adcs_initial_value = packet->new_mode;
    adcs_mode = obc_flags.adcs_initial_value;
    save_state(packet->command);

    //! Change stored value in ADCS
    struct adcs_mode_st {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent;
    } adcs_mode_st;
    adcs_mode_st.origin = MSG_COMM;
    adcs_mode_st.command = 0xAD;
    adcs_mode_st.mode = packet->new_mode;
    adcs_mode_st.permanent = true;
    vschedule(current_time, (uint8_t*)&adcs_mode_st); //! ADCS change to selected mode

    return adcs_mode;
}

//! Schedule ADCS mode (without coordinates)
uint8_t command_schedule_mode(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t adcs_mode;
        time_t mode_scheduled_time;
        uint16_t total_duration;
    }* packet = (struct packet*)data;

    if (packet->mode_scheduled_time == 0) { //! debug case
        packet->mode_scheduled_time = current_time;
    }

    struct adcs_mode_st {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent;
    } adcs_mode_st;

    adcs_mode_st.origin = MSG_COMM;
    adcs_mode_st.command = 0xAD;
    adcs_mode_st.mode = packet->adcs_mode;
    adcs_mode_st.permanent = false;
    vschedule(packet->mode_scheduled_time, (uint8_t*)&adcs_mode_st); //! ADCS change to selected mode

    adcs_mode_st.mode = obc_flags.adcs_initial_value;
    vschedule(packet->mode_scheduled_time + packet->total_duration, (uint8_t*)&adcs_mode_st); //! ADCS back to default mode

    return 0;
}

//! Print the satellite flags to the debug line
uint8_t command_print_flags(uint8_t* data)
{
    print_flags();
    return 0;
}

//! Send a raw command to ADCS
uint8_t command_adcs_raw(uint8_t* data)
{
    uart_clean(ADCS);
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        } 
       // return 1;
    } 
    uart_mux = 1;

    enum { adcs_cmd_size = 12 };

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t adcs_command[adcs_cmd_size];
    }* packet = (struct packet*)data;

    uint8_t msg[adcs_cmd_size] = { 0 };
    msg[0] = MSG_OBC; //! OBC Message header 0x0B
    memcpy(msg + 1, packet->adcs_command, adcs_cmd_size);
    msg[adcs_cmd_size - 1] = 0x0C;

    uart_print_pc_hex(msg, sizeof(msg));

    for (uint8_t i = 0; i < sizeof(msg); i++) {
        fputc(msg[i], ADCS);
    }

    uart_mux = 0;

    return 0;
}

uint8_t command_stm32_raw_8_16(uint8_t* data)
{
    enum { length = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part[length];
    }* packet = (struct packet*)data;

    memset(stm32_command_uhf, 0, STM32_SIZE);
    memcpy(stm32_command_uhf, packet->part, length);
    stm32_raw_command(stm32_command_uhf, STM32_SIZE, 0);
    memset(stm32_command_uhf, 0, STM32_SIZE);

    fprintf(PC, "STM32 command (8/16)");

    return 0;
}

uint8_t command_stm32_raw_uhf32(uint8_t* data)
{
    enum { length = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part[length];
    }* packet = (struct packet*)data;

    uint8_t part = ((packet->command & 0x0F) - 1);

    memcpy(stm32_command_uhf + length * part, packet->part, length);

    fprintf(PC, "STM32 command (32) part %d. ", part + 1);

    if (part == 1) {
        stm32_raw_command(stm32_command_uhf, STM32_SIZE, 0);
        memset(stm32_command_uhf, 0, STM32_SIZE);
    }

    return 0;
}

uint8_t command_stm32_raw_uhf32_tle(uint8_t* data)
{
    enum { length = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part[length];
    }* packet = (struct packet*)data;

    uint8_t part = ((packet->command & 0x0F) - 1);

    memcpy(stm32_command_uhf + length * part, packet->part, length);

    fprintf(PC, "STM32 command (32) part %d. ", part + 1);

    if (part == 1) {
        stm32_raw_command(stm32_command_uhf, STM32_SIZE, true);
        memset(stm32_command_uhf, 0, STM32_SIZE);
    }

    return 0;
}


//! Receive GPS time from ADCS
uint8_t command_adcs_gps_time(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t day;
        uint8_t month;
        uint8_t year;
    }* packet = (struct packet*)data;

    uint8_t valid_time = 0;

    uint8_t gps_data_legnth = 16;
    uart_print_pc_hex(data, gps_data_legnth);

    if (packet->year == 0x00 || packet->year >= 38) {
        valid_time = 2;
    } else if (obc_flags.gps_time_sync_state == 1 && boot_flags.deployment_flag_1 >= 10) {
        valid_time = 1;
        struct_tm gps_time;
        gps_time.tm_year = packet->year + 100;
        gps_time.tm_mon = packet->month - 1;
        gps_time.tm_mday = packet->day;
        gps_time.tm_hour = packet->hour;
        gps_time.tm_min = packet->minute;
        gps_time.tm_sec = packet->second + obc_flags.leap_seconds;

        time_t unix_time = mktime(&gps_time);

        SetTimeSec(unix_time + 1);
        current_time = time(0);
        reset_pic_update_clock(current_time);
        //! Change memory location based on the new date
        flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag_1, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);
        fprintf(PC, "\r\nNew time: %04ld/%02d/%02d %02d:%02d:%02d", gps_time.tm_year + 1900,
            (char)gps_time.tm_mon + 1,
            gps_time.tm_mday,
            gps_time.tm_hour,
            gps_time.tm_min,
            gps_time.tm_sec);
    }
    return valid_time;
}

// //! ADCS copy high sampling data
// uint8_t command_adcs_hs_copy(uint8_t* data)
// {
//     struct packet {
//         uint8_t origin;
//         uint8_t command;
//         uint8_t slots_30m;
//     }* packet = (struct packet*)data;

//     uint32_t source_address = (1831UL + 6.23 * (uint32_t)packet->slots_30m) * MEMORY_SECTOR_SIZE;
//     uint32_t destination_address = FLASH_ADCS_HS_START;
//     const uint32_t n_packets = 6.23 * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

//     fprintf(PC, "\r\n");
//     fprintf(PC, "Copy ADCS HS data to Main PIC\r\n");
//     fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
//     fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
//     fprintf(PC, "Size                = %lu\r\n", n_packets);

//     struct xmodem_command {
//         uint8_t origin;  //! C0
//         uint8_t command; //! D1
//         uint32_t destination_address;
//         uint8_t source;
//         uint8_t destination;
//         uint32_t source_address;
//         uint32_t n_packets;
//     } xmodem_command;
//     xmodem_command.origin = 0xC0;
//     xmodem_command.command = 0xD1;
//     xmodem_command.destination_address = destination_address;
//     xmodem_command.source = 3;      //! ADCS
//     xmodem_command.destination = 0; //! 0: COM Shared FM
//     xmodem_command.source_address = source_address;
//     xmodem_command.n_packets = n_packets;

//     uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
//     vschedule(current_time + 1, (uint8_t*)&xmodem_command);
//     return 0;
// }

//! ADCS copy GPS data
uint8_t command_adcs_gps_copy(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t slots_1h;
    }* packet = (struct packet*)data;

    uint32_t source_address = (1987UL + 5UL * (uint32_t)packet->slots_1h) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_ADCS_GPS_START;
    const uint32_t n_packets = 5UL * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy GPS data to Main PIC\r\n");
    fprintf(PC, "1h slots            = %d\r\n", packet->slots_1h);
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  //! C0
        uint8_t command; //! D1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xD1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 3;      //! ADCS
    xmodem_command.destination = 0; //! 0: COM Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);
    return 0;
}

//! Copy ADCS data to COM shared FM
uint8_t command_copy_adcs_data_to_uhf(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t day_of_the_year;
        uint8_t slot;
    }* packet = (struct packet*)data;

    const uint8_t sectors_per_day = 5;
    uint32_t source_address = (1UL + (uint32_t)packet->day_of_the_year * (uint32_t)sectors_per_day) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_ADCS_TELEMETRY_START + (uint32_t)packet->slot * (uint32_t)sectors_per_day * MEMORY_SECTOR_SIZE;
    uint32_t n_packets = 5UL * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;
    uint8_t copy_delay = 54;

    if (mux_sel(mux_adcs) == mux_adcs) {                                 // Try to change mission shared FM mux position to ADCS
            mux_lock_unlock(true, ((time_t)copy_delay) * n_packets); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
    delay_ms(100);

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy ADCS data to Main PIC\r\n");
    fprintf(PC, "Day of the year     = %lu\r\n", packet->day_of_the_year + 1);
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  //! C0
        uint8_t command; //! D1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xD1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 3;      //! ADCS
    xmodem_command.destination = 0; //! 0: COM Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);

    return 0;
}

//! ============ COM Commands ============

//! Message from UHF
uint8_t command_uhf_message(uint8_t* data)
{
    enum { length = 22 };
    struct packet {
        uint8_t origin;
        uint8_t gs_message[length];
    }* packet = (struct packet*)data;

    static uint8_t last_upload[length] = { 0 }; //! Stores the last uploaded command
    static time_t last_upload_t = 0;

    fprintf(PC, "Uplink: ");
    uart_print_pc_hex(packet->gs_message, length);
    uint8_t valid = uplink_valid(packet->gs_message);
    uint8_t different = memcmp(last_upload, packet->gs_message, length) || (last_upload_t <= (current_time - 4 * 60)); //! Commands are different within 4 min window
    memcpy(last_upload, packet->gs_message, length);
    last_upload_t = current_time;
    if (valid && different) {
        fprintf(PC, "\r\nValid uplink");
        uint8_t cmd[BUFF_LENGTH] = { MSG_COMM };
        uint8_t copy_size = packet->gs_message[2] != 0xCC ? 9 : length; //! Don't pass CRC onwards for short commands
        memcpy(cmd + 1, packet->gs_message + 3, copy_size);
        vschedule(current_time, cmd);
        fprintf(PC, "\r\nScheduled for execution from TRX");
    } else {
        fprintf(PC, " Invalid (%d) or identical (%d) uplink.", valid, different);
    }
    return !(valid && different);
    
}

//! Message from NUHF
uint8_t command_nuhf_message(uint8_t* data)
{
    enum { length = 22 };
    struct packet {
        uint8_t origin;
        uint8_t gs_message[length];
    }* packet = (struct packet*)data;

    static uint8_t last_upload[length] = { 0 }; //! Stores the last uploaded command
    static time_t last_upload_t = 0;

    fprintf(PC, "Uplink: ");
    uart_print_pc_hex(packet->gs_message, length);
    uint8_t valid = uplink_valid(packet->gs_message);
    uint8_t different = memcmp(last_upload, packet->gs_message, length) || (last_upload_t <= (current_time - 4 * 60)); //! Commands are different within 4 min window
    memcpy(last_upload, packet->gs_message, length);
    last_upload_t = current_time;
    if (valid && different) {
        fprintf(PC, "\r\nValid uplink");
        uint8_t cmd[BUFF_LENGTH] = { MSG_COMM };
        uint8_t copy_size = packet->gs_message[2] != 0xCC ? 9 : length; //! Don't pass CRC onwards for short commands
        memcpy(cmd + 1, packet->gs_message + 3, copy_size);
        vschedule(current_time, cmd);
        fprintf(PC, "\r\nScheduled for execution from NTRX");
    } else {
        fprintf(PC, " Invalid (%d) or identical (%d) uplink.", valid, different);
    }
    return !(valid && different);
    
}

//! receive telemetry data from COM pic and print to screen
uint8_t command_com_telemetry(uint8_t* data)
{
    if (uart_ready(COMM)) {
        data = uart_message(COMM); // Assumes this returns a pointer to MSG_LENGTH_COMM bytes
        
        // Expecting 40 bytes: data[0]=0xB0, data[1]=0x00, data[2]=0xCA, data[39]=0xB1
        response_rx = 1; //! Received a reply
        if (!data) return 1;
        if (data[0] != 0xB0 || data[1] != 0x00 || data[2] != 0xCA || data[39] != 0xB1) {
            fprintf(PC, "[COM_TLM] Invalid header/footer: %02X %02X %02X ... %02X\r\n", data[0], data[1], data[2], data[39]);
            return 2;
        }
        fprintf(PC, "[COM_TLM] Received 40-byte telemetry:\r\n");
        for (int i = 0; i < MSG_LENGTH_COMM; ++i) {
            fprintf(PC, "%02X ", data[i]);
            if ((i+1)%16 == 0) fprintf(PC, "\r\n");
        }
        if (MSG_LENGTH_COMM % 16 != 0) fprintf(PC, "\r\n");
        telemetry_time.com_time = current_time;
        // return 0;
        uart_clean(COMM);
    }
    return 1; // No data available
    
}


// uint8_t try_receive_com_telemetry()
// {
//     if (uart_ready(COMM)) {
//         uint8_t* buffer = uart_message(COMM); // Assumes this returns a pointer to MSG_LENGTH_COMM bytes
//         uint8_t result = command_com_telemetry(buffer);
//         uart_clean(COMM);
//         return result;
//     }
//     return 1; // No data available
// }


uint8_t command_send_data_to_com(uint8_t* data)
{
    uint8_t i;
    struct packet {
        uint8_t command;     //! 0B: comm pic message
        uint8_t com_cmd;     //! 50: com message
        uint8_t data[24];     //! data to be passed to com pic
    }* packet = (struct packet*)data;

    uint8_t cmd[6] = { 0 };
    cmd[0] = 0xC0; // COM_MSG
    for (i = 0; i < 5; i++) {
        cmd[i + 1] = packet->data[i];
    }
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], COMM);
    }

    fprintf(PC, "COM cmd: ");
    uart_print_pc_hex(cmd, sizeof(cmd));

    return 0;
}

// //! Change cw TRX source mode flags
// uint8_t command_change_cw_trx_source_flags(uint8_t* data)
// {
//     obc_flags.cw_trx_source = data[2]; //! data[2] is the new flag status
//     save_state(data[1]);               //! data[1] is the current command ID
//     return obc_flags.cw_trx_source;
// }


//! Change CW mode flags
uint8_t command_change_cw_mode_flags(uint8_t* data)
{
    obc_flags.cw_mode = data[2]; //! data[2] is the new flag status
    save_state(data[1]);         //! data[1] is the current command ID
    return obc_flags.cw_mode;
}

//! Ask ADCS if satellite is over Japan
uint8_t over_japan_check()
{
    uint8_t command[] = { 0xFC };
    stm32_raw_command(command, sizeof(command), false); //! Send request to ADCS

    uint8_t response[2] = { 0 };
    uart_download_packet(&uart_port_MSN, response, sizeof(response), 100000); //! Try to get a response

    if (response[1] == 0xA5) {
        return ((int8_t)response[0] > 0); //! Elevation > 0 degrees
    } else {
        return false;
    }
}

//! Decides if it is OK to send CW or not based on flags and ADCS
uint8_t ok_to_send_cw()
{
    switch (obc_flags.cw_mode) {
    case 0: return false;
    case 1: return true;// over_japan_check();
    case 2: return true;
    default: return false;
    }
}

//! COM CW request
uint8_t command_com_cw(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    
    fprintf(PC, "COM: ");
    uart_print_pc_hex(data, MSG_LENGTH_COMM); //! changed com_to_main_size to MSG_LENGTH_COMM

    static uint8_t current_cw = 0;
    telemetry.com_rssi = *(uint16_t*)&data[2];
    //telemetry.com_temp = *(uint16_t*)&data[3];
    telemetry_time.com_time = current_time;

    //! Generate CW reply
    uint8_t cmd[24] = { 0 };
    cmd[0] = MSG_OBC; //! OBC Message header 0x0B
    cmd[1] = 0x50;
    memcpy(cmd + 2, cw[current_cw], sizeof(cw[0])); //cmd[2] ~ cmd[7]
    cmd[sizeof(cw[0]) + 2] = ok_to_send_cw(); //cmd[8]
    // new OBC flags
    cmd[sizeof(cw[0]) + 3] = obc_flags.cw_trx_source; //cmd[9]
    cmd[sizeof(cw[0]) + 4] = obc_flags.trx_on_off; //cmd[10]
    cmd[sizeof(cw[0]) + 5] = obc_flags.ntrx_on_off; //cmd[11]

    cmd[23] = 0x0C;

    // fprintf(PC, "\r\nOBC >> COM: ");
    // uart_print_pc_hex(cmd, sizeof(cmd)); 

    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], COMM);
    }
    current_cw++;
    if (current_cw >= sizeof(cw) / sizeof(cw[0]))
        current_cw = 0;
   
    return 0;
}


// //! COM GMSK request
// uint8_t command_com_gmsk(uint8_t* data)
// {

// }


//! COM Access request
uint8_t command_com_access_request(uint8_t* data)
{
    struct packet {
        uint8_t origin;  //! 0xC0
        uint8_t command; //! 0x59
        uint8_t time;    //! in minutes
    }* packet = (struct packet*)data;

    time_t disable_time = current_time + (60 * (time_t)packet->time);
    scheduled_command_clear_specified_command(0xC0, 0x58);
    schedule(current_time, { 0xC0, 0x58, 0x01 });
    schedule(disable_time, { 0xC0, 0x58, 0x00 });

    uint8_t reply[8] = { 0 };
    send_com_ack(reply);

    return packet->time;
}

//! Change COM access
uint8_t command_com_access_change(uint8_t* data)
{
    struct packet {
        uint8_t origin;  //! 0xC0
        uint8_t command; //! 0x58
        uint8_t state;   //! 0: OBC side; 1: COM side
    }* packet = (struct packet*)data;

    output_bit(MUX_SEL_COM_SHARED_FM, packet->state);
    memory_busy = packet->state;

    return packet->state;
}

//! MISSION Access request
uint8_t command_msn_access_request(uint8_t* data)
{
    struct packet {
        uint8_t origin;  //! 0xC0
        uint8_t command; //! 0x41
        uint8_t time;    //! in minutes
    }* packet = (struct packet*)data;

    time_t disable_time = current_time + (60 * (time_t)packet->time);
    scheduled_command_clear_specified_command(0xC0, 0x40);
    schedule(current_time, { 0xC0, 0x40, 0x01 });
    schedule(disable_time, { 0xC0, 0x40, 0x00 });

    uint8_t reply[8] = { 0 };
    send_com_ack(reply);

    return packet->time;
}

//! Change MSN flash memory access
uint8_t command_msn_access_change(uint8_t* data)
{
    struct packet {
        uint8_t origin;  //! 0xC0
        uint8_t command; //! 0x40
        uint8_t state;   //! 0: OBC side; 1: MISSION side
    }* packet = (struct packet*)data;

    output_bit(MUX_SEL_MSN_SHARED_FM, packet->state);
    memory_busy = packet->state;

    return packet->state;
}


// Ask subsystem to copy data to mission shared FM, then copy data to COM shared FM
uint8_t command_copy_mission_to_com(uint8_t* data)
{
    enum {
        adcs_copy_delay = 54,  // seconds for one sector copy
        eobc_copy_delay = 6,   // seconds for one sector copy
        aprs_copy_delay = 8, //8, // seconds for one sector copy
        osil_copy_delay = 54, // seconds for one sector copy
        tum_copy_delay = 10, // seconds for one sector copy
    };

    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0xEA
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t size;       // in sectors
        uint8_t data_source; // 0: OBC-MAIN FM, 1: ADCS, 2: EOBC, 3: APRS, 4: OSIL, 5: TUM
    }* packet = (struct packet*)data;

    enum {
        dsource_obc = 0,
        dsource_adcs = 1,
        dsource_eobc = 2,
        dsource_aprs = 3,
        dsource_osil = 4,
        dsource_tum = 5,
    };

    uint8_t cpacket[MAX_LENGTH] = { 0 };

    struct copy_packet {
        uint8_t origin; //! OBC Message header 0x0B
        uint8_t command;
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t size;
        uint8_t data_destination; // 0:LOCAL, 1:SHARED
        uint8_t data_source; // 0:LOCAL, 1:SHARED
    }* copy_packet = (struct copy_packet*)cpacket;

    const uint8_t MSG_LENGTH_COPY = sizeof(struct copy_packet) + 2; // +2 for checksum

    switch (packet->data_source) {
    case dsource_obc:
        copy(2, 0, packet->destination_sector, packet->source_sector, packet->size, 1); // Source and destination: 0=COM, 1=MAIN, 2=MISSION ; mode = 1 -> sector copy; get access to memory
        break;
    case dsource_adcs:
        
        if (mux_sel(mux_adcs) == mux_adcs) {                                 // Try to change mission shared FM mux position to ADCS
            mux_lock_unlock(true, ((time_t)adcs_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        fprintf(PC, "Waiting for ADCS copy...");
        delay_ms(100);
        copy_packet->origin = MSG_OBC; //! OBC Message header 0x0B
        copy_packet->command = 0x12; // Copy sectors command on ADCS pic
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_COPY);
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_COPY, 5, 100); // Send 5 times with 100ms delay
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)adcs_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_eobc:
       
        if (mux_sel(mux_eobc) == mux_eobc) {                                 // Try to change mission shared FM mux position to EOBC
            mux_lock_unlock(true, ((time_t)eobc_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        fprintf(PC, "Waiting for EOBC copy...");
        delay_ms(100);
        copy_packet->origin = MSG_OBC; //! OBC Message header 0x0B
        copy_packet->command = 0x0D; // Copy sectors command on EOBC
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_COPY);
          fprintf(PC, "Sent command: ");
        for (int i = 0; i < MSG_LENGTH_COPY; i++) {
            fprintf(PC, "%02X ", cpacket[i]);
        }
        fprintf(PC, "\n");
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_COPY, 1, 10); 
      
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)eobc_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_aprs:
        fprintf(PC, "Waiting for APRS copy...");
        if (mux_sel(mux_aprs) == mux_aprs) {                                 // Try to change mission shared FM mux position to APRS
            mux_lock_unlock(true, ((time_t)aprs_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC; //! OBC Message header 0x0B
        copy_packet->command = 0x2A; // Copy sectors command on APRS
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_COPY);
        fprintf(PC, "Sent command: ");
        for (int j = 0; j < MSG_LENGTH_COPY; j++) {
            fprintf(PC, "%02X ", cpacket[j]);
        }
        fprintf(PC, "\n");
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_COPY, 1, 10); // Send 1 time
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)aprs_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_osil:
        fprintf(PC, "Waiting for OSIL copy...");
        if (mux_sel(mux_osil) == mux_osil) {                                 // Try to change mission shared FM mux position to OSIL
            mux_lock_unlock(true, ((time_t)osil_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC; //! OBC Message header 0x0B
        copy_packet->command = 0xA0; // Copy sectors command on OSIL
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_COPY);
        fprintf(PC, "Sent command: ");
        for (int k = 0; k < MSG_LENGTH_COPY; k++) {
            fprintf(PC, "%02X ", cpacket[k]);
        }
        fprintf(PC, "\n");
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_COPY, 1, 10); // Send 1 time
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)osil_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_tum:
        fprintf(PC, "Waiting for TUM copy...");
        if (mux_sel(mux_tum) == mux_tum) {                                 // Try to change mission shared FM mux position to TUM TUM
            mux_lock_unlock(true, ((time_t)tum_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC; //! OBC Message header 0x0B
        copy_packet->command = 0xA7; // Copy sectors command on TUM
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_COPY);
        fprintf(PC, "Sent command: ");
        for (int a = 0; a < MSG_LENGTH_COPY; a++) {
            fprintf(PC, "%02X ", cpacket[a]);
        }
        fprintf(PC, "\n");
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_COPY, 1, 10); // Send 1 time
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)tum_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    default:
        return 1;
    }
    return 0;
}


//! ============ OBC Commands ============

//! Schedule any command
uint8_t command_schedule_anything(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t schedule_time;
        uint8_t schedule_command[12];
    }* packet = (struct packet*)data;

    vschedule(packet->schedule_time, packet->schedule_command);

    return 0;
}

//! Set the clock to a given value (UNIX time, seconds after Jan 1st 1970)
uint8_t command_set_clock(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t current_time;
        int8_t leap_seconds;
        uint8_t gps_time_sync_state;
    }* packet = (struct packet*)data;
    obc_flags.leap_seconds = packet->leap_seconds;
    obc_flags.gps_time_sync_state = packet->gps_time_sync_state;

    uint8_t time_updated = 0;

    if (packet->current_time < T0) {
        time_updated = 1;
        SetTimeSec(T0); //! Set clock to as early as possible (Jan 1st 2000, 00:00:00);
        //! mai_400_update_clock(T0);
        reset_pic_update_clock(T0);
    } else {
        SetTimeSec(packet->current_time + obc_flags.leap_seconds);
        reset_pic_update_clock(packet->current_time + obc_flags.leap_seconds);
    }
    current_time = time(0);

    //! Change memory location based on the new date
    flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag_1, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);

    struct_tm* local_time = localtime(&current_time);

    fprintf(PC, "New time: %04ld/%02d/%02d %02d:%02d:%02d(0x%08lX) ",
        local_time->tm_year + 1900,
        (uint8_t)local_time->tm_mon + 1,
        local_time->tm_mday,
        local_time->tm_hour,
        local_time->tm_min,
        local_time->tm_sec,
        current_time);

    fprintf(PC, "Leap: %d GPS sync: 0x%02X", obc_flags.leap_seconds, obc_flags.gps_time_sync_state);
    return time_updated;
}

//! Display error to debug port when command is not found in LUT
uint8_t command_error_message(uint8_t* data)
{
    fprintf(PC, "\nCalling commmand that does not exist.\r\n");
    return 0xFF;
}

//! Display the TRIS status to debug port
uint8_t command_get_tris(uint8_t* data)
{
    fprintf(PC, "\n         fedcba9876543210\r\n");
    fprintf(PC, "tris_a = ");
    print_binary16(get_tris_a());
    fprintf(PC, "tris_b = ");
    print_binary16(get_tris_b());
    fprintf(PC, "tris_c = ");
    print_binary16(get_tris_c());
    fprintf(PC, "tris_d = ");
    print_binary16(get_tris_d());
    fprintf(PC, "tris_e = ");
    print_binary16(get_tris_e());
    fprintf(PC, "tris_f = ");
    print_binary16(get_tris_f());
    fprintf(PC, "tris_g = ");
    print_binary16(get_tris_g());
    return 0xEE;
}

//! Prints a memory address relative to satellite time (if ptr = 0x0000, prints satellite time).
uint8_t command_print_memory_address(uint8_t* data)
{
#ifndef TARGET_INTEL_X86
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t ptr;
    }* packet = (struct packet*)data;
    uint32_t* addr = (uint32_t*)packet->ptr + &current_time;
    fprintf(PC, "*%04lX = %08lX", addr, *addr);
#endif
    return 0;
}

// Sets the value of a memory address relative to satellite time
uint8_t command_set_obc_variable(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t offset;
        uint8_t size;
        uint8_t data[sizeof(uint32_t)];
        uint8_t save_state;
    }* packet = (struct packet*)data;

    uint8_t* dst = (uint8_t*)&current_time + packet->offset;
    uint8_t* src = packet->data;

    fprintf(PC, "Changing variable at 0x%04X, size %d to 0x%08LX", dst, packet->size, *(uint32_t*)&packet->data[0]);
    for (uint8_t i = 0; i < packet->size; i++) {
        *(dst + i) = *(src + i);
    }

    if (packet->save_state == 1) {
        save_state(data[1]); // data[1] is the current command id
    } else if (packet->save_state == 2) {
        save_flags();
        scheduled_command_clear_all();
    }

    return 0;
}

uint8_t command_update_obcFlags_variable(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t flag;
        uint16_t value;
        uint8_t save_state;
    }* packet = (struct packet*)data;
 
        enum {
        main_antenna_flag = 0,
        adm_antenna_flag = 1,
        leap_seconds = 2,
        adcs_on_off = 3,
        msnboss_on_off = 4,
        led_on_off = 5,
        trx_on_off = 6,
        ntrx_on_off = 7,
        gps_time_sync_state = 8,
        adcs_initial_value = 9,
        cw_mode = 10,
        cw_trx_source = 11,
        heater_th_temperature = 12,
        heater_th_voltage = 13
    };

    switch (packet->flag) {
    case main_antenna_flag:
        fprintf(PC, "Changing deployment_flag_1 from 0x%2X to 0x%2X", boot_flags.deployment_flag_1, (packet->value) >> 0 & 0xFF);
        boot_flags.deployment_flag_1= (packet->value) >> 0 & 0xFF;
        break;
    case adm_antenna_flag:
        fprintf(PC, "Changing deployment_flag_2 (ADMM) from 0x%2X to 0x%2X", boot_flags.deployment_flag_2, (packet->value) >> 0 & 0xFF);
        boot_flags.deployment_flag_2= (packet->value) >> 0 & 0xFF;
        break;
    case leap_seconds:
        fprintf(PC, "Changing leap_seconds flag from 0x%2X to 0x%2X", obc_flags.leap_seconds, (packet->value) >> 0 & 0xFF);
        obc_flags.leap_seconds = (packet->value) >> 0 & 0xFF;
        break;
    case adcs_on_off:
        fprintf(PC, "Changing adcs_on_off flag from 0x%2X to 0x%2X", obc_flags.adcs_on_off, (packet->value) >> 0 & 0xFF);
        obc_flags.adcs_on_off = (packet->value) >> 0 & 0xFF;
        break;
    case msnboss_on_off:
        fprintf(PC, "Changing msnboss_on_off flag from 0x%2X to 0x%2X", obc_flags.msnboss_on_off, (packet->value) >> 0 & 0xFF);
        obc_flags.msnboss_on_off =(packet->value) >> 0 & 0xFF;
        break;
    case led_on_off:
        fprintf(PC, "Changing led_on_off flag from 0x%2X to 0x%2X", obc_flags.led_on_off, (packet->value) >> 0 & 0xFF);
        obc_flags.led_on_off = (packet->value) >> 0 & 0xFF;
        break;
    case trx_on_off:
        fprintf(PC, "Changing trx_on_off flag from 0x%2X to 0x%2X", obc_flags.trx_on_off, (packet->value) >> 0 & 0xFF);
        obc_flags.trx_on_off = (packet->value) >> 0 & 0xFF;
        break;
    case ntrx_on_off:
        fprintf(PC, "Changing ntrx_on_off flag from 0x%2X to 0x%2X", obc_flags.ntrx_on_off, (packet->value) >> 0 & 0xFF);
        obc_flags.ntrx_on_off = (packet->value) >> 0 & 0xFF;
        break;
    case gps_time_sync_state:
        fprintf(PC, "Changing gps_time_sync_state flag from 0x%2X to 0x%2X", obc_flags.gps_time_sync_state, (packet->value) >> 0 & 0xFF);
        obc_flags.gps_time_sync_state = (packet->value) >> 0 & 0xFF;
        break;
    case adcs_initial_value:
        fprintf(PC, "Changing adcs_initial_value flag from 0x%2X to 0x%2X", obc_flags.adcs_initial_value, (packet->value) >> 0 & 0xFF);
        obc_flags.adcs_initial_value = (packet->value) >> 0 & 0xFF;
        break;
    case cw_mode:
        fprintf(PC, "Changing cw_mode flag from 0x%2X to 0x%2X", obc_flags.cw_mode, (packet->value) >> 0 & 0xFF);
        obc_flags.cw_mode= (packet->value) >> 0 & 0xFF;
        break;
    case cw_trx_source:
        fprintf(PC, "Changing cw_trx_source flag from 0x%2X to 0x%2X", obc_flags.cw_trx_source, (packet->value) >> 0 & 0xFF);
        obc_flags.cw_trx_source= (packet->value) >> 0 & 0xFF;
        break;
    case heater_th_temperature:
        fprintf(PC, "Changing heater_th_temperature from 0x%4X to 0x%4X", obc_flags.heater_th_temperature, packet->value);
        obc_flags.heater_th_temperature= packet->value;
        break;
    case heater_th_voltage:
        fprintf(PC, "Changing heater_th_voltage from 0x%4X to 0x%4X", obc_flags.heater_th_voltage, packet->value);
        obc_flags.heater_th_voltage= packet->value;
        break;
    default:
        fprintf(PC, "Invalid flag index!");
        break;
    }

    if (packet->save_state == 1) {
        save_state(data[1]); // data[1] is the current command id
    } else if (packet->save_state == 2) {
        save_flags();
        scheduled_command_clear_all();
    }

    return 0;
}

//helper function to make longer delays
void wait_time(uint32_t seconds)
{
    uint16_t step = 0;
    uint32_t total_delay = (uint32_t)seconds * 1000UL;
    while (total_delay > 0) {
    step = (total_delay > 60000) ? 60000 : total_delay;
    delay_ms(step);
    total_delay -= step;
}
}

 // Sends the antenna deployment command to increase flag before deployment - satellite cannot uplink with undeployed antenna
uint8_t command_deploy_antenna(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t time_s;
    }* packet = (struct packet*)data;

    output_low(DIO_BURNER_ANTENNA); // Ensure antenna burner is off before deployment

    if (current_time >= T_ANTENNA) {    // Never try to deploy ahead of 30m mark
        uint32_t time = (uint32_t) 60; // in seconds
        if (packet->time_s) {
            time = (uint32_t) packet->time_s;
        }
       
        fprintf(PC, "\r\nDeploying Main antenna (%us)... ", (unsigned int)packet->time_s);
        output_high(DIO_BURNER_ANTENNA);
        //delay_ms(time);
        wait_time(time);
        output_low(DIO_BURNER_ANTENNA);

        // Increment deployment flag and save state
        boot_flags.deployment_flag_1++;
        get_com_shared_fm_access();
        uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
        flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
        flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));

        fprintf(PC, "\r\nMain antenna deployment done!");
        return 0;
    } else {
        return 1;
    }
}

 // Sends the antenna deployment command to increase flag before deployment - satellite cannot uplink with undeployed antenna
uint8_t command_deploy_admm_antenna(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t time_s;
    }* packet = (struct packet*)data;
    
    if (current_time >= T_ANTENNA_2) {    // Never try to deploy second antenna ahead of 40m mark
        uint8_t thermalKnife = 0x60;
        
        struct antenna2_deployment_command {
        uint8_t origin;
        uint8_t command;
        uint8_t thermalKnife;
        uint8_t duration;
        } cmd;

        cmd.origin = MSG_COMM;
        cmd.command = 0xD0;
        cmd.thermalKnife= 0x6F;
        cmd.duration = packet->time_s;

        vschedule(current_time + 360L, (uint8_t*)&cmd); //! Antenna deploy command (Jan 1st 2000, 00:45:30), 60s.

        fprintf(PC, "\r\nDeploying secondary antenna (%us)... ", (unsigned int)packet->time_s);
        send_mb_command(thermalKnife, packet->time_s, false);
    
        // Increment deployment flag and save state
        boot_flags.deployment_flag_2++;
        get_com_shared_fm_access();
        uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
        flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
        flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));
        //fprintf(PC, "\r\nSecondary antenna deployment done!");
        return 0;
    } else {
        return 1;
    }
}


//! Clear completely the main memory.
uint8_t command_clear_state(uint8_t* data)
{
    get_com_shared_fm_access();
    flash_erase(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, ERASE_SECTOR);
    fprintf(PC, "Waiting 10s for reset...\r\n");
    delay_ms(10000);
    reset_cpu();
    return 0;
}

//! Save state to memory.
uint8_t command_save_state(uint8_t* data)
{
    get_com_shared_fm_access();
    save_state(data[1]); //! data[1] is the current command id
    return 0;
}


//! write dummy data in FM
uint8_t command_dummy_memory_write(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t startAddress;
        uint8_t destination;
        // uint32_t size;
    }* packet = (struct packet*)data;
    
    fprintf(PC, "\r\ndummy data write start...\r\n");

    //dummy data generation
    uint8_t dummy_data[256] = { 0 };
    for (uint32_t i = 0; i < sizeof(dummy_data); i++) {
        dummy_data[i] = (uint8_t)i;
        fprintf(PC, "%02X ", dummy_data[i]);
    }

    //SET A FIXED ADDRESS TO WRITE THE DUMMY DATA
    uint32_t address = packet->startAddress;//0x04ED0000;//0x00010000; // Example address, change as needed
 
    //get_com_shared_fm_access();
    #define MEMORY_SECTOR_SIZE (64UL * 1024UL)
    uint8_t erase_mode = ERASE_SECTOR;
    uint32_t increment = MEMORY_SECTOR_SIZE;
    
    //uint32_t to_address = address * increment;
   // uint32_t from_address = 0;
    uint32_t size = 256; //bytes to write
    
    uint8_t dest = packet->destination;//0x02; //MISSION

    fprintf(PC, "memwrt dest=%d,to_addr=%lX,size=%lX", dest, address, size);
    
    //flash_write(&spi_port_MISSION_FM, address, dummy_data);
    // Erase the pages before copying
    spi_fn* to_spi_functions;
    if (dest == 0x00) {
         fprintf(PC, "\r\nmemwrt to COM Shared FM!");
        //  to_spi_functions = &spi_port_COM_FM;
        // for (uint32_t i = 0; i < size; i += increment) {
        //     flash_erase(&spi_port_COM_FM, address + i, erase_mode);
        // }
        // //flash_write(&spi_port_COM_FM, address, dummy_data);
        //  output_low(MUX_SEL_MSN_SHARED_FM); 
        // for (uint32_t j = 0; j < size; j++) {
        //     flash_write(to_spi_functions, address, dummy_data[j]);
        //     address++;
        // }
        // output_high(MUX_SEL_MSN_SHARED_FM);
    } else if (dest == 0x01) {
         fprintf(PC, "\r\nmemwrt to Main FM!");
         to_spi_functions = &spi_port_MAIN_FM;
        for (uint32_t i = 0; i < size; i += increment) {
            flash_erase(&spi_port_MAIN_FM, address + i, erase_mode);
        }
        //flash_write(&spi_port_MAIN_FM, address, dummy_data);
        for (uint32_t j = 0; j < size; j++) {
            flash_write(to_spi_functions, address, dummy_data[j]);
            address++;
        }
    } else if (dest == 0x02) {
        fprintf(PC, "\r\nmemwrt to Mission Shared FM!");
        to_spi_functions = &spi_port_MISSION_FM;
        for (uint32_t i = 0; i < size; i += increment) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, address + i, erase_mode);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
            //flash_write(&spi_port_MISSION_FM, address, dummy_data);
            output_low(MUX_SEL_MSN_SHARED_FM); 
        for (uint32_t j = 0; j < size; j++) {
            flash_write(to_spi_functions, address, dummy_data[j]);
            address++;
        }
        output_high(MUX_SEL_MSN_SHARED_FM);
    }
 
    fprintf(PC, "\r\nmemwrt last add=%lX", address-1); //address -1 is the last byte address
  
    return 0;
}




//! Dump state.
uint8_t command_dump_memory(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t start;
        uint32_t size;
        uint8_t source;
    }* packet = (struct packet*)data;
    fprintf(PC, "\r\nHex dump start");
    switch (packet->source) {
    case 0:
        get_com_shared_fm_access();
        flash_dump(&spi_port_COM_FM, packet->start, packet->start + packet->size);
        break;
    case 1:
        flash_dump(&spi_port_MAIN_FM, packet->start, packet->start + packet->size);
        break;
    case 2:
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_dump(&spi_port_MISSION_FM, packet->start, packet->start + packet->size);
        output_high(MUX_SEL_MSN_SHARED_FM);
        break;
    default:
        get_com_shared_fm_access();
        flash_dump(&spi_port_COM_FM, packet->start, packet->start + packet->size);
        break;
    }
    fprintf(PC, "\r\nHex dump end\r\n");
    return packet->source;
}

//! Copy data between flash memories
uint8_t command_copy_memory_sector(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t n_sectors;
        uint8_t destination_port; //! 0:COM, 1:MAIN, 2:MISSION
        uint8_t origin_port;      //! 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t origin = packet->origin_port;
    uint8_t dest = packet->destination_port;

    uint32_t to_address_ = (uint32_t)packet->destination_sector * MEMORY_SECTOR_SIZE;
    uint32_t from_address_ = (uint32_t)packet->source_sector * MEMORY_SECTOR_SIZE;
    uint32_t size = (uint32_t)packet->n_sectors * MEMORY_SECTOR_SIZE;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX", origin, dest, to_address_, from_address_, size);

    get_com_shared_fm_access();

    //! Erase the pages before copying
    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address_ + i, ERASE_SECTOR);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address_ + i, ERASE_SECTOR);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address_ + i, ERASE_SECTOR);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    if (origin == 0x00 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x01 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x00) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_COM_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x01) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else {
        return 1;
    }

    return 0;
}

//! Copy data between flash memories
uint8_t command_copy_memory_page(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_page;
        uint16_t source_page;
        uint16_t n_pages;
        uint8_t destination_port; //! 0:COM, 1:MAIN, 2:MISSION
        uint8_t origin_port;      //! 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t origin = packet->origin_port;
    uint8_t dest = packet->destination_port;

    uint32_t to_address_ = (uint32_t)packet->destination_page * MEMORY_PAGE_SIZE;
    uint32_t from_address_ = (uint32_t)packet->source_page * MEMORY_PAGE_SIZE;
    uint32_t size = (uint32_t)packet->n_pages * MEMORY_PAGE_SIZE;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX", origin, dest, to_address_, from_address_, size);

    get_com_shared_fm_access();

    //! Erase the pages before copying
    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address_ + i, ERASE_PAGE);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address_ + i, ERASE_PAGE);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address_ + i, ERASE_PAGE);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    if (origin == 0x00 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x01 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x00) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_COM_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x01) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else {
        return 1;
    }

    return 0;
}


//! Copy TUM TUM data from board storage to MSN shared flash memory through SPI
uint8_t command_tum_copy(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t slots_1h;
    }* packet = (struct packet*)data;

    uint32_t source_address = (1987UL + 5UL * (uint32_t)packet->slots_1h) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_TUM_START;
    const uint32_t n_packets = 5UL * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy TUM data to MSN Shared Flash\r\n");
    fprintf(PC, "1h slots            = %d\r\n", packet->slots_1h);
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  //! C0
        uint8_t command; //! D1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xD1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 5;      //! 0: xxx 1: xxx 2:xxx 3: ADCS 4: xxx 5: TUM
    xmodem_command.destination = 2; //! 0: COM Shared FM 1: Main Shared FM 2: Mission Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);
    return 0;
}

//! Erase data from flash memory
uint8_t command_erase_memory_page(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_page;
        uint16_t n_pages;
        uint8_t destination_port; //! 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t dest = packet->destination_port;

    uint32_t to_address = (uint32_t)packet->destination_page * MEMORY_PAGE_SIZE;
    uint32_t size = (uint32_t)packet->n_pages * MEMORY_PAGE_SIZE;

    get_com_shared_fm_access();

    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address + i, ERASE_PAGE);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address + i, ERASE_PAGE);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address + i, ERASE_PAGE);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    return 0;
}

//! Erase data from flash memory
uint8_t command_erase_memory_sector(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t n_sectors;
        uint8_t destination_port; //! 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t dest = packet->destination_port;

    uint32_t to_address = (uint32_t)packet->destination_sector * MEMORY_SECTOR_SIZE;
    uint32_t size = (uint32_t)packet->n_sectors * MEMORY_SECTOR_SIZE;

    get_com_shared_fm_access();

    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address + i, ERASE_SECTOR);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address + i, ERASE_SECTOR);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address + i, ERASE_SECTOR);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    return 0;
}

uint8_t command_clear_all_schedule_commands(uint8_t* data)
{
    scheduled_command_clear_all();
    fprintf(PC, "\r\nAll scheduled commands cleared.");
    return 0;
}

uint8_t command_boot_flag_set(uint8_t* data)
{
    get_com_shared_fm_access();
    uint8_t value = data[2];
    uint8_t antennaMechanism = data[4];
        if (antennaMechanism == 0x00)
            boot_flags.deployment_flag_1 = value;
        else if(antennaMechanism == 0x01)
            boot_flags.deployment_flag_2 = value;
        else if (antennaMechanism == 0x02)
        {
            boot_flags.deployment_flag_1 = value;
            boot_flags.deployment_flag_2 = value;
        }

    uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
    flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
    flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));
    scheduled_command_clear_all();
    return value;
}

//! Reset log command
uint8_t command_reset_log(uint8_t* data)
{
    fprintf(PC, "Main PIC reset detected.");
    return boot_flags.deployment_flag_1;
}

//! Telemetry keeping function
uint8_t command_save_telemetry(uint8_t* data)
{
    uart_mux = 1;
    if (!memory_busy) {
        //! Write the obc timestamp to telemetry and relative collection times
        telemetry.obc_time = current_time;
        telemetry.reset_time = current_time - telemetry_time.reset_time > 255 ? 255 : current_time - telemetry_time.reset_time;
        telemetry.fab_time = current_time - telemetry_time.fab_time > 255 ? 255 : current_time - telemetry_time.fab_time;
        telemetry.msn_time = current_time - telemetry_time.msn_time > 255 ? 255 : current_time - telemetry_time.msn_time;
        telemetry.adcs_time = current_time - telemetry_time.adcs_time > 255 ? 255 : current_time - telemetry_time.adcs_time;
        telemetry.aprs_time = current_time - telemetry_time.aprs_time > 255 ? 255 : current_time - telemetry_time.aprs_time;
        telemetry.tum_time = current_time - telemetry_time.tum_time > 255 ? 255 : current_time - telemetry_time.tum_time;
        telemetry.eobc_time = current_time - telemetry_time.adcs_time > 255 ? 255 : current_time - telemetry_time.eobc_time;
        telemetry.osil_time = current_time - telemetry_time.adcs_time > 255 ? 255 : current_time - telemetry_time.adcs_time;
        
        
        telemetry.com_time = current_time - telemetry_time.com_time > 255 ? 255 : current_time - telemetry_time.com_time;

        //! Save telemetry to flash
        uint8_t* telemetry_data = (uint8_t*)&telemetry;
        flash_cycle_write(&spi_port_COM_FM, telemetry_data, &addr_flags.flash_telemetry);
        fprintf(PC, "Saving telemetry data: ");
        fprintf(PC, "Addr: 0x%08lX ", addr_flags.flash_telemetry.current - sizeof(telemetry));

        build_cw();             //! Prepare CW strings
        initialize_telemetry(); //! Reset for next iteration

        //! Save satellite log to flash
        log_flush();

        //! Save addresses to flash
        struct addr {
            uint32_t flash_log_current;
            uint32_t flash_telemetry_current;
        } addr;

        uint8_t* addr_flag_ptr = (uint8_t*)&addr;

        addr.flash_log_current = addr_flags.flash_log.current;
        addr.flash_telemetry_current = addr_flags.flash_telemetry.current;

        flash_cycle_write(&spi_port_COM_FM, addr_flag_ptr, &addr_flags.flash_addr);
        return 0;
    } else {
        fprintf(PC, "Skipping saving telemetry data.");
        return 1;
    }
}

//! Change OCP's default state
uint8_t command_ocp_state(uint8_t* data)
{
    struct packet {
        uint8_t origin;     //! 0xC0
        uint8_t command;    //! 0xAE
        uint8_t on_off;     // 0: off; 1: on
        uint8_t ocp_number; // 0: adcs; 1: msnboss; 2: led; 3: TRX; 4:NTRX
    }* packet = (struct packet*)data;

    enum {
        ocp_adcs = 0,
        ocp_msnboss = 1,
        enable_led = 2,
        ocp_trx = 3,
        ocp_ntrx = 4
    };

    switch (packet->ocp_number) {
    case ocp_adcs:
        output_bit(OCP_EN_ADCS, packet->on_off);
        obc_flags.adcs_on_off = packet->on_off;
        return obc_flags.adcs_on_off;
        break;
    case ocp_msnboss:
        output_bit(OCP_EN_MSNBOSS, packet->on_off);
        obc_flags.msnboss_on_off = packet->on_off;
        return obc_flags.msnboss_on_off;
        break;
    case enable_led:
        output_bit(EN_LED, packet->on_off);
        obc_flags.led_on_off = packet->on_off;
        fprintf(PC, "changing LED Flag to %d", obc_flags.led_on_off);
        return obc_flags.led_on_off;
        break; 
    case ocp_trx:
        obc_flags.trx_on_off = packet->on_off;
        fprintf(PC, "changing Addnics TRX mode to %d", obc_flags.trx_on_off);
        return obc_flags.trx_on_off;
        break; 
    case ocp_ntrx:
        obc_flags.ntrx_on_off = packet->on_off;
        fprintf(PC, "changing New TRX mode to %d", obc_flags.ntrx_on_off);
        return obc_flags.ntrx_on_off;
        break; 
    }

    return -1;
}

//! void boot_commands_clear_nth(uint8_t n)
uint8_t command_boot_cmd_clear_nth(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t n;
    }* packet = (struct packet*)data;
    boot_commands_clear_nth(packet->n);
    boot_commands_write();
    return 0;
}

//! void boot_commands_clear_all()
uint8_t command_boot_cmd_clear_all(uint8_t* data)
{
    boot_commands_clear_all();
    boot_commands_write();
    return 0;
}

//! Add a boot command
uint8_t command_boot_cmd_add(uint8_t* data)
{
    enum { length = MSG_LENGTH_COMM - 13 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t delay_from_boot;
        uint8_t new_boot_cmd[length];
    }* packet = (struct packet*)data;

    boot_command bc;
    bc.time = packet->delay_from_boot;
    memcpy(bc.command, packet->new_boot_cmd, length);
    boot_commands_add(bc);
    boot_commands_write();

    return 0;
}

//! ============ Reset Commands ============

//! Reset pic telemetry data
uint8_t command_reset_telemetry(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    struct packet {
        uint8_t message[MSG_LENGTH_RST - 10];
        uint8_t padding[9]; //! empty part
        uint8_t footer;
    }* packet = (struct packet*)data;

    fprintf(PC, "RESET: ");
    uart_print_pc_hex(data, MSG_LENGTH_RST);

    if (rst_clock_update) {
        rst_clock_updated = 1;
        rst_clock_update = 0;
        struct_tm rst_time;
        rst_time.tm_year = (unsigned long)packet->message[2] + 100;
        rst_time.tm_mon = (unsigned long)packet->message[3] - 1;
        rst_time.tm_mday = (unsigned long)packet->message[4];
        rst_time.tm_hour = (unsigned long)packet->message[5];
        rst_time.tm_min = (unsigned long)packet->message[6];
        rst_time.tm_sec = (unsigned long)packet->message[7] + 1;
        SetTime(&rst_time);
        time_t new_time = mktime(&rst_time);
        //! mai_400_update_clock(new_time);
        current_time = new_time;
        previous_time = new_time;
        reset_time = new_time;
        //! Read stored commands from memory
        get_com_shared_fm_access();
        uint8_t* cmd_ptr = (uint8_t*)scheduled_commands;
        flash_transfer_data_to_ram(
            &spi_port_COM_FM,
            SCHEDULED_CMD_ADDRESS,
            cmd_ptr,
            sizeof(scheduled_commands));
        //! Remove scheduled commads that are scheduled to run in the past
        for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
            if (scheduled_commands[i].time < current_time + 1800L) {
                scheduled_commands[i].time = TIME_T_MAX;
                fprintf(PC, "\r\nWarning: a scheduled command (%02X %02X) in flash memory was scheduled to run in a past date/time and was marked as complete.\r\n", scheduled_commands[i].command[0], scheduled_commands[i].command[1]);
            }
        }
        //! Read and schedule boot commands
        boot_commands_schedule();
        //! Change memory location based on the new date
        flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag_1, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);

        struct_tm* local_time = localtime(&current_time);
        fprintf(PC, " New time: %04ld/%02d/%02d %02d:%02d:%02d (0x%08lX)",
            local_time->tm_year + 1900,
            (uint8_t)local_time->tm_mon + 1,
            local_time->tm_mday,
            local_time->tm_hour,
            local_time->tm_min,
            local_time->tm_sec,
            current_time);
    }

    telemetry_time.reset_time = current_time;
    memcpy(telemetry.reset_message, packet->message + 2, sizeof(telemetry.reset_message));
    return rst_clock_updated;
}

//! Warn that 24-hour reset is about to happen
uint8_t command_reset_warning(uint8_t* data)
{
    uint8_t i;

    //! uint8_t adcs_cmd[] = { 0x22 };
    //! mai_400_command(adcs_cmd, sizeof(adcs_cmd));

    save_state(data[1]); //! data[1] is the current command id

    //! Reply:
    const uint8_t cmd[36] = {
        0xB0, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1
    };
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    uart_clean(RST);

    fprintf(PC, "Waiting for 24h reset...\r\n");
    delay_ms(60000);
    delay_ms(60000);
    fprintf(PC, "No 24h reset happened, doing soft reset instead.\r\n");
    reset_cpu();

    return 0;
}

//! Reset pic acknowledge of time change
uint8_t command_time_change_ack(uint8_t* data)
{
    fprintf(PC, "Reset PIC time change ACK.");
    return 0;
}

//! Passes a command from ground-station to reset pic
uint8_t command_send_data_to_reset(uint8_t* data)
{
    uint8_t i;
    struct packet {
        uint8_t command;     //! C0: comm pic message
        uint8_t reset_cmd;   //! F5: reset message
        uint8_t data[8];     //! data to be passed to reset pic
        uint8_t silent_mode; //! if equals to 1, do not print debug message
    }* packet = (struct packet*)data;
    uint8_t cmd[36] = { 0 };
    cmd[0] = 0xB0;
    for (i = 0; i < 8; i++) {
        cmd[i + 1] = packet->data[i];
    }
    cmd[35] = 0xB1;
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    if (packet->silent_mode != 1)
        uart_print_pc_hex(cmd, sizeof(cmd));
    return 0;
}

//! ============ Mission Boss Commands ============


typedef enum {
    mb_cmd_telemetry_req = 0x10,
    mb_cmd_aprs = 0x20,
    mb_cmd_osil = 0x30,
    mb_cmd_eobc = 0x40,
    mb_cmd_tum = 0x50,
    mb_cmd_admm_dep = 0x60,
} mb_command; 

uint8_t command_missionboss(uint8_t* data)
{
    struct packet {
        uint8_t origin;  // C0
        uint8_t command; // D0 for MB
        uint8_t cmd_id;
        uint8_t data;
    }* packet = (struct packet*)data;
   
    send_mb_command(packet->cmd_id, packet->data, false);
}

//! Request for mission boss telemetry
uint8_t command_request_mission_boss(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    
    uart_clean(MSNBOSS);

    if (mux_sel(mux_msnboss) != mux_msnboss) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    send_mb_command(0x10, 0x00, true);
    
}

// log Mission Boss Status to MP debug
uint8_t command_mission_boss_log(uint8_t* data)
{

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t status;
    }* packet = (struct packet*)data;

    response_rx = 1; // Received a reply

    uint16_t status = packet->status;
   
    // Print each bit with its corresponding shorthand and value
    fprintf(PC, "MB: ");
    fprintf(PC, "CABUREI: %u, ", (uint8_t)((status & (1 << 0)) ? 1 : 0));
    fprintf(PC, "OSIL: %u, ", (uint8_t)((status & (1 << 1)) ? 1 : 0));
    fprintf(PC, "EOBC: %u, ", (uint8_t)((status & (1 << 2)) ? 1 : 0));
    fprintf(PC, "TUM: %u, ", (uint8_t)((status & (1 << 3)) ? 1 : 0));
    fprintf(PC, "DET1 = %u, ", (uint8_t)((status & (1 << 4)) ? 1 : 0));
    fprintf(PC, "DET2 = %u, ", (uint8_t)((status & (1 << 5)) ? 1 : 0));
    fprintf(PC, "DET3 = %u, ", (uint8_t)((status & (1 << 6)) ? 1 : 0));
    fprintf(PC, "DET4 = %u | ", (uint8_t)((status & (1 << 7)) ? 1 : 0));
    uart_print_pc_hex(data, MSG_LENGTH_MSNBOSS);
    telemetry_time.msn_time = current_time;
    memcpy(telemetry.msn_message, data + 2, sizeof(telemetry.msn_message));

    return 0;
}

//! ===========================================================================
//! ========= MISSION PAYLOAD Commands (final place) ==========================
//! ===========================================================================

//! ============ TUM Commands ============

//! Request for TUM TUM telemetry
uint8_t command_request_tum(uint8_t* data)
{
    uart_clean(TUM);
    
    if (mux_sel(mux_tum) != mux_tum) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
        }
        return 1;
    } 
    uart_mux = 1;

    enum { cmd_size = 12 };

    uint8_t cmd[cmd_size] = { 0 };

    cmd[0] = MSG_OBC;
    cmd[1] = 0x97;

    // --- convert current_time ---
    uint8_t yy, mm, dd, hh, mi, ss;
    seconds_to_datetime(current_time, &yy, &mm, &dd, &hh, &mi, &ss);

    // --- debug print ---
   // fprintf(PC, "\r\nSending current time to TUM: 20%02u-%02u-%02u %02u:%02u:%02u and requesting telemetry...\r\n",
    //        yy, mm, dd, hh, mi, ss);

    // --- pack into cmd[2..7] ---
    cmd[2] = yy;
    cmd[3] = mm;
    cmd[4] = dd;
    cmd[5] = hh;
    cmd[6] = mi;
    cmd[7] = ss;
    
    // cmd[2] = 0x6f;
    // cmd[3] = 0x04;
    // cmd[4] = 0x20;
    // cmd[5] = 0x6f;
    // cmd[6] = 0x04;
    // cmd[7] = 0x03;

    cmd[11] = 0x0C;

    //checksum_obc(cmd, sizeof(cmd)); // checksum

    fprintf(PC,"Sending TLM request to TUM: ");
    uart_print_pc_hex(cmd, sizeof(cmd));
    fprintf(PC,"\n\r");

        for (uint8_t j = 0; j < sizeof(cmd); j++) {
        fputc(cmd[j], TUM);
    }

    //delay_ms(1000);
    // if (kbhit(TUM)) {
    //             uint8_t ACK = fgetc(TUM);
    //                 fprintf(PC, "ACK received: %02X\r\n", ACK);
    //                 break;
    //             }
     return 0;
}

//! Send command to TUM TUM UART line
uint8_t command_generic_tum(uint8_t* data)
{
    // ACK codes
    const uint8_t NORM_ACK = 0xAD;
    const uint8_t PWR_ACK  = 0xAE;

    uart_clean(TUM);

    // Ensure MUX is on TUM path
    if (mux_sel(mux_tum) != mux_tum) {
        if (verbose) {
            fprintf(PC, "\r\nMUX change failed!");
        }
        return 1;
    }

    enum { cmd_length = 12 };
    uint8_t cmd[cmd_length] = {0};

    cmd[0]  = MSG_OBC;
    // Copy payload into cmd[1..10]
    for (uint8_t i = 1; i < (cmd_length - 1); i++) {
        cmd[i] = data[i + 1];
    }

    // Calculate checksum
    checksum_obc(cmd, cmd_length);
    // fprintf(PC,"Sending TLM request to TUM: ");
    // uart_print_pc_hex(cmd, sizeof(cmd));
    // fprintf(PC,"\n\r");

    // Select expected ACK based on command type
    uint8_t expected_ack = NORM_ACK;
    bool is_power_warn   = (cmd[1] == 0x0F);
    if (is_power_warn) {
        expected_ack = PWR_ACK;
    }

    uart_print_pc_hex(cmd, sizeof(cmd));

    const uint8_t n_tries     = 3;
    uint8_t max_timeout = is_power_warn ? 1 : 3; // shorter timeout for power warn

    uint8_t got_ack = 0;

    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d\r\n", attempt + 1);

        // Send command
        for (uint8_t j = 0; j < sizeof(cmd); j++) {
            fputc(cmd[j], TUM);
        }

        fprintf(PC, "TUM: ");
        uart_print_pc_hex(cmd, sizeof(cmd));
        fprintf(PC, "\r\n");

        // wait for ACK
        uint8_t timeout_counter = 0;
        while (timeout_counter++ < 3) {
            if (kbhit(TUM)) {
                uint8_t ACK = fgetc(TUM);
                if (ACK == expected_ack) {
                    fprintf(PC, "ACK received: %02X\r\n", ACK);
                    got_ack = 1;
                    break;
                }
            }
            delay_ms(1000);
        }
        if (got_ack) break;
    }

    if (is_power_warn) {
        // Always tell MB to turn TUM off, ACK or not
        send_mb_command(0x5F, 0x00, false);
        //in EM : send Resetpic command to turn off 5V0#1
    }

    if (!got_ack) {
        fprintf(PC, "\r\nFailed to get an ACK after %d tries.\r\n", n_tries);
        return 0xFF; // failure
    }

    return 0; // success
}

uint8_t command_tum_tlm_log(uint8_t* data)
{
    response_rx = 1; //! Received a reply
    fprintf(PC, "TUM: ");
    uart_print_pc_hex(data, MSG_LENGTH_TUM);
    telemetry_time.tum_time = current_time;
    memcpy(telemetry.tum_message, data + 2, sizeof(telemetry.tum_message));
    return 0;
}

//! ============ EPS Commands ============

//! Print EPS telemetry to screen
uint8_t command_eps_tlm_log(uint8_t* data)
{
    enum { cmd_len = 7 };
    response_rx = 1; //! Received a reply
    fprintf(PC, "EPS: ");

    uint16_t voltage_hex = make16(data[46], data[47]);
    uint16_t current_hex = make16(data[50], data[51]);
    uint16_t temperature_hex = make16(data[52], data[53]);

    float voltage = voltage_hex * 3.3 * 3 / 4096;
    float current = (3812.6 * current_hex * 3.28 / 4096) - 6228.5;
    float temperature = 75 - temperature_hex * 3.256 * 30 / 4096;
    fprintf(PC, "V=%f, C=%f, T=%f | ", voltage, current, temperature);

    uint16_t heater_th_temperature = make16(data[56], data[57]);
    uint16_t heater_th_voltage = make16(data[58], data[59]);

    if ((heater_th_temperature != obc_flags.heater_th_temperature) || (heater_th_voltage != obc_flags.heater_th_voltage)) {
        //! E0 66 TH TL VH VL
        fprintf(PC, " New T=0x%04lX, V=0x%04lX | ", obc_flags.heater_th_temperature, obc_flags.heater_th_voltage);
        uint8_t cmd_temp[cmd_len];
        cmd_temp[0] = 0xC0;                                      //! COM_MSG
        cmd_temp[1] = 0x6C;                                      //! OBC_CMD
        cmd_temp[2] = 0x66;                                      //! EPS_CMD
        cmd_temp[3] = (obc_flags.heater_th_temperature >> 8);   //! TEMP_HIGH
        cmd_temp[4] = (obc_flags.heater_th_temperature & 0xFF); //! TEMP_LOW
        cmd_temp[5] = (obc_flags.heater_th_voltage >> 8);       //! V_HIGH
        cmd_temp[6] = (obc_flags.heater_th_voltage & 0xFF);     //! V_LOW
        vschedule(current_time + 5, cmd_temp);
    }

    uart_print_pc_hex(data, MSG_LENGTH_FAB);
    telemetry_time.fab_time = current_time;
    memcpy(telemetry.fab_message, data + 2, sizeof(telemetry.fab_message));
    return 0;
}

//! Passes a command from ground-station to eps pic
uint8_t command_send_data_to_eps(uint8_t* data)
{
    uint8_t i;
    enum { data_size = 5 };
    struct packet {
        uint8_t command;         //! C0: com pic message
        uint8_t eps_cmd;         //! 6C: eps message
        uint8_t data[data_size]; //! data to be passed to reset pic
    }* packet = (struct packet*)data;

    uint8_t cmd[data_size + 1] = { 0 };
    cmd[0] = 0xE0;
    for (i = 0; i < data_size; i++) {
        cmd[i + 1] = packet->data[i];
    }
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], FAB);
    }

    fprintf(PC, "EPS cmd: ");
    uart_print_pc_hex(cmd, sizeof(cmd));

    return 0;
}

//! Passes a command from ground-station to cpld
uint8_t command_send_data_to_cpld(uint8_t* data)
{
    uint8_t i;
    enum { data_size = 6 };
    struct packet {
        uint8_t command;         //! C0: com pic message
        uint8_t cmd_id;     //! E5
        uint8_t msn_header;         //! msn header MSG_OSIL MSG_EOBC MSG_APRS 
        uint8_t data[data_size]; //! data to be passed to reset pic
    }* packet = (struct packet*)data;

    uint8_t cmd[data_size + 1] = { 0 };
    cmd[0] = MSG_OSIL;
    if (packet->msn_header == MSG_EOBC || packet->msn_header == MSG_OSIL || packet->msn_header == MSG_APRS || packet->msn_header == MSG_TUM) {
        cmd[0] = packet->msn_header;
        for (i = 0; i < data_size; i++) {
            cmd[i + 1] = packet->data[i];
        }
         for (i = 0; i < sizeof(cmd); i++) {
            fputc(cmd[i], MSN);
        }
        fprintf(PC, "MSN CPLD cmd: ");
        uart_print_pc_hex(cmd, sizeof(cmd));

        return 0;
    } else {
        fprintf(PC, "header doesnt match with any mission: ");
    }

    return 0xFF; // Invalid command
}

uint8_t command_eps_set_heater_ref(uint8_t* data)
{
    struct packet {
        uint8_t command;   //! C0: com pic message
        uint8_t eps_cmd;   //! 6D: heater ref settings
        uint16_t temp_th; //! temperature reference
        uint16_t v_th;    //! voltage reference
    }* packet = (struct packet*)data;

    obc_flags.heater_th_temperature = packet->temp_th;
    obc_flags.heater_th_voltage = packet->v_th;
    save_state(packet->eps_cmd);

    fprintf(PC, "New T=0x%04lX, V=0x%04lX", obc_flags.heater_th_temperature, obc_flags.heater_th_voltage);

    return 0;
}

uint8_t command_obc_kill_on(uint8_t* data)
{
    uint8_t kill_sw_status = 0;
    if (data[2] == 0x55 && data[4] == 0x55 && data[6] == 0x55 && data[8] == 0x55 && data[3] == 0xAA && data[5] == 0xAA && data[7] == 0xAA && data[9] == 0xAA) {
        kill_sw_status = 1;
        output_high(DIO_KILL_OBC);
    }
    return kill_sw_status;
}

uint8_t command_obc_kill_off(uint8_t* data)
{
    output_low(DIO_KILL_OBC);
    return 0;
}


// =====================================================

typedef struct mission {
    time_t mission_time;                            // time when the command is sent to the mission (Unix time)
    uint16_t mission_duration;                      // how long does it take to execute the mission in seconds; 0 = do not turn off
    uint8_t adcs_mode;                              // adcs mode during mission
    uint16_t adcs_maneuver_duration;                // how early to change the ADCS before sending the mission command (in seconds); 0 = never
    uint16_t mission_on_time;                       // how early to turn on the mission before sending the mission command (in seconds)
    uint8_t turn_on_value;                          // active high will turn on the subsystem
    uint16_t pin_number;                            // pin number to turn on/off the subsystem; 0 = do not change
    uart_fn* command_port;                          // UART port to send the command
    uart_fn* on_off_command_port;                   // UART port to send on/off commands
    uint8_t mux_position_command;                   // mux position when sending mission commands
    uint8_t mux_position_on_off;                    // mux position when sending on/off commands
    uint8_t command[MSG_LENGTH_OSIL];               // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t command_size;                           // the size of the command above; 0 = do not send
    uint8_t turn_on_command[MSG_LENGTH_OSIL];       // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t turn_on_command_size;                   // the size of the command above; 0 = do not send
    uint8_t turn_off_soft_command[MSG_LENGTH_OSIL]; // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t turn_off_soft_command_size;             // the size of the command above; 0 = do not send
    uint8_t turn_off_hard_command[MSG_LENGTH_OSIL]; // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t turn_off_hard_command_size;             // the size of the command above; 0 = do not send
    uint16_t turn_off_duration;                     // time it takes between sending the turn off warning and mission power off (in seconds); 0 = never turn off
    uint8_t mux_lock;                               // if true, the mux position is locked for mux_lock_duration seconds
    uint16_t mux_lock_duration;                     // duration to lock the mux seconds
} mission;


mission camera = {
    0,                        // mission_time
    65,                       // mission_duration
    adcs_mode_nadir_pointing, // adcs_mode
    3 * 60 * 60,              // adcs_maneuver_duration (3 hours)
    60,                       // mission_on_time
    true,                     // turn_on_value
    0,                        // pin_number
    &uart_port_MSN,           // command_port
    &uart_port_MSN,           // on_off_port
    mux_osil,                 // mux_position_command
    mux_osil,                 // mux_position_on_off
    { 0 },                    // command
    MSG_LENGTH_OSIL,          // command_size
    { MSG_OBC, 0xBE, 0x01 },  // turn_on_command MSG_OBC: 0x0B
    MSG_LENGTH_OSIL,          // turn_on_command_size
    { MSG_OBC, 0xBE, 0x02 },  // turn_off_soft_command MSG_OBC: 0x0B
    MSG_LENGTH_OSIL,          // turn_off_soft_command_size
    { MSG_OBC, 0xBE, 0x00 },  // turn_off_hard_command MSG_OBC: 0x0B
    MSG_LENGTH_OSIL,          // turn_off_hard_command_size
    20,                       // turn_off_duration
    false,                    // mux_lock
    65                        // mux_lock_duration
};


uint16_t camera_still_duration(uint8_t* data)
{
    enum constants {
        camera_config_length = 11,
        camera_capture_length = 4,
        a = 1,
        b = 60
    };

    struct rpi_camera_still {
        uint8_t origin;
        uint8_t command;
        uint8_t data[camera_config_length + camera_capture_length];
    }* packet = (struct rpi_camera_still*)data;

    uint16_t bitOffset = 0;

    // Camera capture configuration command
    uint32_t shutterSpeed[3];

    for (int i = 0; i < 3; ++i) {
        uint8_t shutterSpeedMantissa = get_bits(packet->data, &bitOffset, 4);
        uint8_t shutterSpeedExponent = get_bits(packet->data, &bitOffset, 3);
        shutterSpeed[i] = (double)shutterSpeedMantissa * pow(10, shutterSpeedExponent) + 0.5; // Using base 10 exponentiation

        bitOffset += 20;
    }

    bitOffset += 7;

    // Camera capture command
    uint8_t cameraSelection[3];
    cameraSelection[0] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[1] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[2] = get_bits(packet->data, &bitOffset, 1);

    bitOffset += 16;
    uint8_t sequentialPictures = get_bits(packet->data, &bitOffset, 6);
    uint8_t intervalBetweenPictures = get_bits(packet->data, &bitOffset, 7);

    // Print human-readable output to PC
    if (verbose) {
        fprintf(PC,
            "Camera capture configuration:\r\n"
            "Camera Selection: %u, %u, %u\r\n"
            "Sequential Pictures: %u\r\n"
            "Interval Between Pictures: %u seconds\r\n"
            "Shutter Speeds: %lu, %lu, %lu (microseconds)\r\n",
            cameraSelection[0],                               // 8-bit value
            cameraSelection[1],                               // 8-bit value
            cameraSelection[2],                               // 8-bit value
            sequentialPictures,                               // 8-bit value
            intervalBetweenPictures,                          // 8-bit value
            shutterSpeed[0], shutterSpeed[1], shutterSpeed[2] // 32-bit values
        );
    }

    uint16_t delay = (uint16_t)sequentialPictures * ((uint16_t)intervalBetweenPictures + (a * cameraSelection[0] * shutterSpeed[0] / 1000000L + a * cameraSelection[1] * shutterSpeed[1] / 1000000L + a * cameraSelection[2] * shutterSpeed[2] / 1000000L)) + b;

    if (verbose) {
        fprintf(PC, "Delay: %lu\r\n", delay);
    }

    return delay;
}


uint16_t camera_image_transform_duration(uint8_t* data)
{
    enum constants {
        camera_image_transform_length = 11,
        a = 20,
        b = 20
    };

    struct rpi_camera_transform {
        uint8_t origin;
        uint8_t command;
        uint8_t data[camera_image_transform_length];
    }* packet = (struct rpi_camera_transform*)data;

    uint16_t bitOffset = 85;

    uint8_t cameraSelection[3];
    cameraSelection[0] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[1] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[2] = get_bits(packet->data, &bitOffset, 1);

    // Print human-readable output to PC
    if (verbose) {
        fprintf(PC,
            "Camera Selection: %u%u%u\r\n",
            cameraSelection[0],
            cameraSelection[1],
            cameraSelection[2]);
    }

    uint16_t delay = (uint16_t)cameraSelection[0] * a + (uint16_t)cameraSelection[1] * a + (uint16_t)cameraSelection[2] * a + b;

    if (verbose) {
        fprintf(PC, "Delay: %lu\r\n", delay);
    }

    return delay;
}


uint16_t camera_video_duration(uint8_t* data)
{
    enum constants {
        camera_video_length = 11,
        a = 20
    };

    struct rpi_camera_video {
        uint8_t origin;
        uint8_t command;
        uint8_t data[camera_video_length];
    }* packet = (struct rpi_camera_video*)data;

    uint16_t bitOffset = 21; // Skip ADCS mode and ADCS maneuver duration

    uint8_t cameraSelection[3];
    cameraSelection[0] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[1] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[2] = get_bits(packet->data, &bitOffset, 1);

    bitOffset += 16;

    uint8_t timeout[3];

    for (int i = 0; i < 3; ++i) {
        bitOffset += 10;
        timeout[i] = get_bits(packet->data, &bitOffset, 6);
    }

    // Print human-readable output to PC
    if (verbose) {
        fprintf(PC,
            "Video capture command:\r\n"
            "Camera Selection: %u%u%u\r\n"
            "Timeout: %u, %u, %u seconds\r\n",
            cameraSelection[0],
            cameraSelection[1],
            cameraSelection[2],
            timeout[0], timeout[1], timeout[2]);
    }

    uint16_t delay = (uint16_t)cameraSelection[0] * timeout[0] + (uint16_t)cameraSelection[1] * timeout[1] + (uint16_t)cameraSelection[2] * timeout[2] + a;

    if (verbose) {
        fprintf(PC, "Delay: %lu\r\n", delay);
    }

    return delay;
}

// uint8_t command_camera_mission(uint8_t* data)
// {
//     enum constants {
//         camera_config_length = 11,
//         camera_capture_length = 4,
//         camera_image_transform_length = 11,
//         camera_video_length = 11,
//         xmodem_rpi_to_bus_length = 4,
//     };

//     enum inner_commands {
//         camera_capture_configuration = 0,
//         camera_capture = 1,
//         image_transformation = 2,
//         video_capture = 3,
//         xmodem_rpi_to_bus = 4
//     };

//     uint8_t inner_command = data[2];

//     if (inner_command == camera_capture_configuration) {

//         struct packet_0 {
//             uint8_t origin;
//             uint8_t cmd;
//             uint8_t inner_command;
//             uint8_t data[camera_config_length];
//         }* packet_0 = (struct packet_0*)data;

//         fprintf(PC, "Camera capture configuration command, saving configuration...");
//         memcpy(obc_flags.camera_parameters, packet_0->data, camera_config_length);
//         save_state(packet_0->cmd);
//         fprintf(PC, " done!");

//     } else if (inner_command == camera_capture) {

//         struct packet_1 {
//             uint8_t origin;
//             uint8_t cmd;
//             uint8_t inner_command;
//             time_t start_time;
//             uint8_t data[camera_capture_length];
//             uint8_t adcs_mode;
//             uint16_t adcs_maneuver_duration;
//         }* packet_1 = (struct packet_1*)data;

//         fprintf(PC, "Camera capture command, starting state machine...");

//         camera.mission_time = packet_1->start_time;
//         camera.adcs_maneuver_duration = packet_1->adcs_maneuver_duration;
//         camera.mux_lock = false;
//         camera.adcs_mode = packet_1->adcs_mode;

//         camera.command[0] = MSG_OBC; //! OBC Message header 0x0B
//         camera.command[1] = 0xCA;
//         memcpy(&camera.command[2], obc_flags.camera_parameters, camera_config_length);            // first part of the config comes from the obc flags
//         memcpy(&camera.command[2 + camera_config_length], packet_1->data, camera_capture_length); // second part comes from this command
//         camera.mission_duration = camera_still_duration(camera.command);
//         camera.mux_lock_duration = camera.mission_duration;

//         schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

//     } else if (inner_command == image_transformation) {

//         struct packet_2 {
//             uint8_t origin;
//             uint8_t cmd;
//             uint8_t inner_command;
//             time_t start_time;
//             uint8_t data[camera_image_transform_length];
//         }* packet_2 = (struct packet_2*)data;

//         fprintf(PC, "Camera image transformation command, starting state machine...");

//         camera.mission_time = packet_2->start_time;
//         camera.adcs_maneuver_duration = 0;
//         camera.mux_lock = false;

//         camera.command[0] = MSG_OBC; //! OBC Message header 0x0B
//         camera.command[1] = 0xCB;
//         memcpy(&camera.command[2], packet_2->data, camera_image_transform_length);
//         camera.mission_duration = camera_image_transform_duration(camera.command);
//         camera.mux_lock_duration = camera.mission_duration;

//         schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

//     } else if (inner_command == video_capture) {

//         struct packet_3 {
//             uint8_t origin;
//             uint8_t cmd;
//             uint8_t inner_command;
//             time_t start_time;
//             uint8_t data[camera_video_length];
//         }* packet_3 = (struct packet_3*)data;

//         fprintf(PC, "Camera image video command, starting state machine...");

//         camera.mission_time = packet_3->start_time;
//         camera.adcs_maneuver_duration = ((packet_3->data[0] & 0xE0) >> 5);
//         camera.adcs_maneuver_duration |= ((uint16_t)packet_3->data[1] << 3);
//         camera.adcs_maneuver_duration |= ((uint16_t)(packet_3->data[2] & 0x1F) << 11);
//         camera.mux_lock = false;
//         camera.adcs_mode = packet_3->data[0] & 0x1F;

//         camera.command[0] = MSG_OBC; //! OBC Message header 0x0B
//         camera.command[1] = 0xCC;
//         memcpy(&camera.command[2], packet_3->data, camera_video_length);
//         camera.mission_duration = camera_video_duration(camera.command);
//         camera.mux_lock_duration = camera.mission_duration;

//         schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

//     } else if (inner_command == xmodem_rpi_to_bus) {

//         struct packet_4 {
//             uint8_t origin;
//             uint8_t cmd;
//             uint8_t inner_command;
//             time_t start_time;
//             uint16_t timeout;
//             uint8_t data[xmodem_rpi_to_bus_length];
//         }* packet_4 = (struct packet_4*)data;

//         fprintf(PC, "XMODEM copy command RPi to bus, starting state machine...");

//         camera.mission_time = packet_4->start_time;
//         camera.mission_duration = packet_4->timeout;
//         camera.adcs_maneuver_duration = 0;
//         camera.mux_lock = true;
//         camera.mux_lock_duration = camera.mission_duration;

//         camera.command[0] = MSG_OBC; //! OBC Message header 0x0B
//         camera.command[1] = 0xC2;
//         memcpy(&camera.command[2], packet_4->data, xmodem_rpi_to_bus_length);

//         schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

//     } else {
//         fprintf(PC, "Inner command not implemented.");
//     }

//     return inner_command;
// }





//! =====================================================

//! Used to stop the simulation
uint8_t command_debug(uint8_t* data)
{
#ifdef PC_SIM
    fprintf(PC, "Exiting simulation");
    continue_program = 0;
#else
    for (uint32_t i = 0; i < 65536; i++) {
        fprintf(PC, "%02X:%02X:%d\r\n", (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), (int8_t)gyro_to_cw((uint8_t)(i >> 8), (uint8_t)(i & 0xFF)));
    }
#endif //! !PC_SIM
    return 0;
}

command_test_uart_line(uint8_t* data)
{
    enum { test_length = 3 };
    struct packet {
        uint8_t command;         //! C0: com pic message
        uint8_t test_cmd;        //! D6: test uart line
        uint8_t uart_port;       //! port to be tested
        uint8_t test_data[test_length]; //! data to be sent to the port
    }* packet = (struct packet*)data;

    // uart_fn* port;
    // switch (packet->uart_port) {
    // case 0: port = PC; break;
    // case 1: port = APRS; break;
    // case 2: port = EOBC; break;
    // case 3: port = FAB; break;
    // case 4: port = TUM; break;
    // case 5: port = ADCS; break;
    // case 6: port = COMM; break;

    fprintf(PC, "Testing UART port %d\r\n", PC);

    packet->test_data[0] = 0xAA;
    packet->test_data[1] = 0xBB;
    packet->test_data[2] = 0xCC;

    // default:
    //     fprintf(PC, "Invalid UART port number %d\r\n", packet->uart_port);
    //     return 1;
    // }

    fprintf(PC, "Testing UART port %d\r\n", packet->uart_port);
    for (uint8_t i = 0; i < test_length; i++) {
    switch (packet->uart_port) {
        case 0: fputc(packet->test_data[i], PC); break;
        case 1: fputc(packet->test_data[i], APRS); break;
        case 2: fputc(packet->test_data[i], EOBC); break;
        case 3: fputc(packet->test_data[i], FAB); break;
        case 4: fputc(packet->test_data[i], TUM); break;
        case 5: fputc(packet->test_data[i], ADCS); break;
        case 6: fputc(packet->test_data[i], COMM); break;
        default: break;
        }
    }

    return 0;
}

//! =====================================================

//! Function that executes a command, by looking up on the table of available commands.
void command_execute(uint8_t* data, uint8_t origin, uint8_t log_enabled)
{
    disable_interrupts(global);
    uint8_t error = 0;

    if (origin == data[0] || origin == MSG_WILDCARD) {

        //! COM reply
        if (data[0] == 0xC0) {
            if (data[1] == 0x42) {
                if (data[4] != 0x00 ) { //! 0x00 is a request for memory dump
                    send_com_ack(data + 2);
                } else {
#ifndef TARGET_INTEL_X86
                    uint32_t* ptr = *(uint32_t*)&data[5];
                    //a debug printf
                    //fprintf(PC, "%04X ", (ptr+ &current_time));
                    send_com_ack(ptr + &current_time);
#endif
                }
            }
        }

        //! Satellite log
        log_entry log;
        log.origin = data[0];
        log.command = data[1];

        struct_tm* local_time = localtime(&current_time);

        //! Execute command
        if (log_enabled)
            fprintf(PC, "%04ld/%02d/%02d %02d:%02d:%02d | %02d | %02X %02X | ", local_time->tm_year + 1900,
                (uint8_t)local_time->tm_mon + 1,
                local_time->tm_mday,
                local_time->tm_hour,
                local_time->tm_min,
                local_time->tm_sec,
                scheduled_command_count(),
                data[0],
                data[1]);

        uint16_t command = make16(data[0], data[1]);

        //! The table of available commands.
        switch (command) {
        case 0xAB02: error = command_mux_lock_unlock(data); break;
        case 0xAD90: error = command_adcs_telemetry(data); break;
        case 0xADDA: error = command_adcs_gps_time(data); break;
        case 0xB0A0: error = command_reset_telemetry(data); break;
        case 0xB0A2: error = command_reset_warning(data); break;
        case 0xB070: error = command_time_change_ack(data); break;
        case 0xC000: error = command_print_memory_address(data); break;
        case 0xC001: error = command_set_clock(data); break;
        case 0xC002: error = command_mux_lock_unlock(data); break;
        //case 0xC003: error = command_set_obc_variable(data); break;
        case 0xC004: error = command_update_obcFlags_variable(data); break;
        // case 0xC005: reserved for COM
        // case 0xC006: reserved for COM
        // case 0xC007: reserved for COM
        // case 0xC008: reserved for COM
        // case 0xC009: reserved for COM
        case 0xC00A: error = command_copy_memory_page(data); break;
        case 0xC00B: error = command_copy_memory_sector(data); break;
        case 0xC00C: error = command_erase_memory_page(data); break;
        case 0xC00D: error = command_erase_memory_sector(data); break;
        // case 0xC00E: reserved for COM
        // case 0xC00F: reserved for COM
        // case 0xC010: reserved for COM
        // case 0xC011: reserved for COM
        // case 0xC012: reserved for COM
        // case 0xC013: reserved for COM
        // case 0xC014: reserved for COM
        // case 0xC015: reserved for COM
        // case 0xC016: reserved for COM
        // case 0xC017: reserved for COM
        // case 0xC020: reserved for COM
        // case 0xC021: reserved for COM
        // case 0xC022: reserved for COM
        // case 0xC023: reserved for COM
        // case 0xC024: reserved for COM
        case 0xC025: error = command_boot_cmd_clear_nth(data); break;
        case 0xC026: error = command_boot_cmd_clear_all(data); break;
        // case 0xC027: reserved for COM
        case 0xC028: error = command_boot_cmd_add(data); break;

        // case 0xC030: reserved for COM
        case 0xC040: error = command_msn_access_change(data); break;
        case 0xC041: error = command_msn_access_request(data); break;
        case 0xC042: error = command_uhf_message(data); break;
        // case 0xC044: reserved for COM
        case 0xC050: error = command_com_cw(data); break; 
       // case 0xC051: error = 
        case 0xC054: error = command_nuhf_message(data); break;
        case 0xC055: error = command_save_state(data); break;
        case 0xC058: error = command_com_access_change(data); break;
        case 0xC059: error = command_com_access_request(data); break;
        case 0xC060: error = command_obc_kill_on(data); break;
        case 0xC061: error = command_obc_kill_off(data); break;
        case 0xC06C: error = command_send_data_to_eps(data); break;
        case 0xC06D: error = command_eps_set_heater_ref(data); break;
        case 0xC070: error = command_stm32_raw_8_16(data); break;
        case 0xC071: error = command_stm32_raw_uhf32(data); break;
        case 0xC072: error = command_stm32_raw_uhf32(data); break;
        case 0xC090: error = command_request_reset(data); break;
        case 0xC091: error = command_request_eps(data); break;
        case 0xC092: error = command_request_mission_boss(data); break;
        case 0xC093: error = command_request_adcs(data); break;
        case 0xC094: error = command_request_osil(data); break;
        case 0xC095: error = command_request_aprs(data); break;
        case 0xC096: error = command_request_eobc(data); break;
        case 0xC097: error = command_request_tum(data); break;
        // case 0xC098: error = 
        case 0xC0A1: error = command_stm32_raw_uhf32_tle(data); break;
        case 0xC0A2: error = command_stm32_raw_uhf32_tle(data); break;
        case 0xC0A5: error = command_request_gps_time(data); break;
        case 0xC0A6: error = command_adcs_default_mode(data); break;
        case 0xC0A8: error = command_adcs_gps_copy(data); break;
        // case 0xC0A9: error = command_adcs_hs_copy(data); break;
        case 0xC0AC: error = command_adcs_request_gps(data); break;
        case 0xC0AD: error = command_adcs_mode(data); break;
        case 0xC0AE: error = command_ocp_state(data); break;
        case 0xC0AF: error = command_adcs_raw(data); break;
        case 0xC0B0: error = command_send_data_to_com(data); break;
        // case 0xC0B1: reserved for COM
        // case 0xC0B2: reserved for COM
        // case 0xC0B3: reserved for COM
        case 0xC0B4: error = command_generic_aprs(data); break;
        // case 0xC0B5: reserved for COM
        // case 0xC0B6: reserved for COM
        // case 0xC0B7: reserved for COM
        // case 0xC0B8: reserved for COM
        // case 0xC0B9: reserved for COM
        // case 0xC0BA: reserved for COM
        // case 0xC0BB: reserved for COM
        case 0xC0BC: error = command_generic_eobc(data); break;
        case 0xC0BD: error = command_generic_tum(data); break;
        // case 0xC0BE: reserved for COM
        // case 0xC0BF: reserved for COM
        // case 0xC0CA: reserved for COM
        case 0xC0CB: error = command_generic_osil_camera(data); break;
        //case 0xC0CC: error = command_change_cw_trx_source_flags(data); break;
        case 0xC0CD: error = command_change_cw_mode_flags(data); break;
        case 0xC0C5: error = command_clear_state(data); break;
        case 0xC0D0: error = command_missionboss(data); break;
        case 0xC0D9: error = command_deploy_admm_antenna(data); break;
        case 0xC0DA: error = command_deploy_antenna(data); break;
        case 0xC0DB: error = command_debug(data); break;
        case 0xC0DC: error = command_error_message(data); break;
        case 0xC0DD: error = command_dump_memory(data); break;
        case 0xC0DE: error = command_dummy_memory_write(data); break;//for testing FM write 
        case 0xC0DF: error = command_get_tris(data); break;
        case 0xC0E4: error = command_copy_adcs_data_to_uhf(data); break;
        case 0xC0EA: error = command_copy_mission_to_com(data); break;
        // case 0xBC00: 
        case 0xC0F0: error = command_mux_sel_sfm(data); break;
        case 0xC0F5: error = command_send_data_to_reset(data); break;
        case 0xC0F6: error = command_schedule_anything(data); break;
        case 0xC0F7: error = command_schedule_mode(data); break;
        case 0xC0F8: error = command_save_telemetry(data); break;
        case 0xC0F9: error = command_clear_all_schedule_commands(data); break;
        // case 0xC0FA: reserved for COM
        // case 0xC0FB: reserved for COM
        case 0xC0A7: error = command_tum_copy(data); break;
        case 0xC0D6: error = command_test_uart_line(data); break;
        case 0xC0FC: error = command_print_flags(data); break;
        case 0xC0FE: error = command_boot_flag_set(data); break;
        case 0xC0FF: error = command_reset_log(data); break;
        case 0x0E00: error = command_mission_boss_log(data); break;
        // case 0xB400:reserved for COM
        case 0xCB33: error = command_osil_tlm_log (data); break;
        case 0xE033: error = command_eps_tlm_log(data); break;
        case 0xE066: error = command_eps_set_heater_ref(data); break;
        case 0xBC33: error = command_eobc_tlm_log (data); break;
        case 0xBD33: error = command_tum_tlm_log(data); break;//BD33
       // case 0xBD0F: error = command_tum_tlm_log(data); break;//BD33
        case 0xB433: error = command_aprs_tlm_log(data); break;
        // case 0xC0A4: error = send_data_periodically_to_cpld(data); break;
        
        default: error = command_error_message(data); break;
        }

        if (log_enabled) {
            fprintf(PC, " (Ret: %02X)", error);
        }

        //! Satellite log

        log.time = current_time;
        log.return_value = error;

        if (log_enabled 
            && command != 0xB0A0  // reset pic
            && command != 0xE033  // eps pic
            && command != 0x0E00  // mission boss
            && command != 0xAD90  // adcs pic
            && command != 0xC050  // com pic 
            && command != 0xBC33  // eobc pic
            && command != 0xCB33  // osil pic
            && command != 0xBD33  // tum
            && command != 0xBD0F  // tum optionb
            && command != 0xB433  // aprs
            && command != 0xC0F8) // save telemetry command
            log_add(log);

        if (log_enabled)
            fprintf(PC, "\r\n");
    }
    enable_interrupts(global);
}

#endif /* INTERPRETER_H */
