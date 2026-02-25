#ifndef MAIN_H
#define MAIN_H

#include "variable_definitions.h"
#include "memory_setup.h"
#include "interpreter.h"

void satellite_init()
{

    fprintf(PC, "\r\n");
    fprintf(PC," ____ _____ _____  _____   _____       _____  _____  __  __ \r\n");
    fprintf(PC,"|  _ \\_   _|  __ \\|  __ \\ / ____|     |  __ \\|  __ \\|  \\/  |\r\n");
    fprintf(PC,"| |_) || | | |__) | |  | | (___ ______| |__) | |__) | \\  / |\r\n");
    fprintf(PC,"|  _ < | | |  _  /| |  | |\\___ \\______|  _  /|  ___/| |\\/| |\r\n");
    fprintf(PC,"| |_) || |_| | \\ \\| |__| |____) |     | | \\ \\| |    | |  | |\r\n");
    fprintf(PC,"|____/_____|_|  \\_\\_____/|_____/      |_|  \\_\\_|    |_|  |_|\r\n");
    fprintf(PC, "\r\n");
    fprintf(PC, "Compiled on: "__DATE__
                " "__TIME__
                "\r\n\r\n");
    //! Ascii art generated with: https://!patorjk.com/software/taag/#p=display&f=Nancyj&t=BIRDS-RPM%20

    //! Satellite time initialization
    current_time = time(0);
    //! In case RTC registers have invalid data, load the initial time
    if (current_time < T0 || current_time > Tn) {
        fprintf(PC, "> Invalid clock, recovering...\r\n");
        current_time = T0;
        SetTimeSec(T0);
    }
    previous_time = current_time;
    //! ctime(&current_time, &current_time_str);

    //! > RTC & Timers
    setup_rtc(RTC_ENABLE | RTC_CLOCK_SOSC, 0x00); //! enables internal RTC
    setup_timer_0(T0_INTERNAL | T0_DIV_32);

    //! Telemetry initialization
    initialize_telemetry();
    //! Multiplexer of Flash Memory
    output_low(MUX_SEL_COM_SHARED_FM);  //! COM SHARED FM to OBC side
    output_high(MUX_SEL_MSN_SHARED_FM); //! Mission SHARED FM

    memory_setup();
    fprintf(PC, "\r\n");
    print_flags();
    
    //! Multiplexer of CPLD 
    mux_sel(mux_msnboss); //! Set MUX to MISSION boss

    //! Interruputions
    enable_interrupts(INT_TIMER0);
    enable_interrupts(GLOBAL);
    enable_interrupts(INT_RDA);  //! MISSION
    enable_interrupts(INT_RDA2); //! COMM
    enable_interrupts(INT_RDA3); //! FAB
    enable_interrupts(INT_RDA4); //! RST

    //! OCPs
    output_bit(OCP_EN_ADCS, obc_flags.adcs_on_off);           //! ADCS
    output_bit(OCP_EN_MSNBOSS, obc_flags.msnboss_on_off);     //! Mission Boss
    
    execute(1, { MSG_PC, 0xFF }); //! Reset log
}

void add_new_commands()
{
    static uint8_t buffer[MAX_LENGTH];
    if (uart_ready(COMM)) {
        memcpy(buffer, uart_message(COMM), MSG_LENGTH_COMM);
        command_execute(buffer, MSG_COMM, 1);
        uart_clean(COMM);
    }
    if (uart_ready(APRS)) {
        memcpy(buffer, uart_message(APRS), MSG_LENGTH_APRS);
        command_execute(buffer, MSG_APRS, 1);
        uart_clean(APRS);
    }
    if (uart_ready(EOBC)) {
        memcpy(buffer, uart_message(EOBC), MSG_LENGTH_EOBC);
        command_execute(buffer, MSG_EOBC, 1);
        uart_clean(EOBC);
    }
    if (uart_ready(TUM)) {
        memcpy(buffer, uart_message(TUM), MSG_LENGTH_TUM);
        command_execute(buffer, MSG_TUM, 1);
        uart_clean(TUM);
    }
    if (uart_ready(ADCS)) {
        memcpy(buffer, uart_message(ADCS), MSG_LENGTH_ADCS);
        command_execute(buffer, MSG_ADCS, 1);
        uart_clean(ADCS);
    }
    if (uart_ready(OSIL)) {
        memcpy(buffer, uart_message(OSIL), MSG_LENGTH_OSIL);
        command_execute(buffer, MSG_OSIL, 1);
        uart_clean(OSIL);
    }
    if (uart_ready(FAB)) {
        memcpy(buffer, uart_message(FAB), MSG_LENGTH_FAB);
        command_execute(buffer, MSG_FAB, 1);
        uart_clean(FAB);
    }
    if (uart_ready(RST)) {
        memcpy(buffer, uart_message(RST), MSG_LENGTH_RST);
        command_execute(buffer, MSG_RST, 1);
        uart_clean(RST);
    }
    if (uart_ready(MSNBOSS)) {
        memcpy(buffer, uart_message(MSNBOSS), MSG_LENGTH_MSNBOSS);
        command_execute(buffer, MSG_MSNBOSS, 1);
        uart_clean(MSNBOSS);
    }
    if (uart_ready(PC)) {
        memcpy(buffer, uart_message(PC), MSG_LENGTH_PC);
        command_execute(buffer, MSG_WILDCARD, 1);
        uart_clean(PC);
    }
}

void periodic_requests_commands()
{
    const uint8_t period = 115;
    periodic_command_clear_rx_flag(period, 0);           //! Reset
    periodic_command(period, 0, 0, { MSG_COMM, 0x90 });  //! Reset
    periodic_command(period, 1, 0, { MSG_COMM, 0x90 });  //! Reset
    periodic_command(period, 2, 0, { MSG_COMM, 0x90 });  //! Reset
    periodic_command(period, 3, 0, { MSG_COMM, 0x90 });  //! Reset
    periodic_command(period, 4, 0, { MSG_COMM, 0x90 }); //! Reset

    periodic_command_clear_rx_flag(period, 7);          //! EPS
    periodic_command(period, 7, 0, { MSG_COMM, 0x91 }); //! EPS
    periodic_command(period, 10, 0, { MSG_COMM, 0x91 }); //! EPS
    periodic_command(period, 13, 0, { MSG_COMM, 0x91 }); //! EPS
    periodic_command(period, 16, 0, { MSG_COMM, 0x91 }); //! EPS
    periodic_command(period, 19, 0, { MSG_COMM, 0x91 }); //! EPS

    periodic_command_clear_rx_flag(period, 22);          //! MSN BOSS
    periodic_command(period, 22, 0, { MSG_COMM, 0x92 }); //! MSN BOSS
    periodic_command(period, 25, 0, { MSG_COMM, 0x92 }); //! MSN BOSS
    periodic_command(period, 28, 0, { MSG_COMM, 0x92 }); //! MSN BOSS
    periodic_command(period, 31, 0, { MSG_COMM, 0x92 }); //! MSN BOSS
    periodic_command(period, 34, 0, { MSG_COMM, 0x92 }); //! MSN BOSS

    periodic_command_clear_rx_flag(period, 37);          //! ADCS MISSION
    periodic_command(period, 37, 0, { MSG_COMM, 0x93 }); //! ADCS MISSION
    periodic_command(period, 40, 0, { MSG_COMM, 0x93 }); //! ADCS MISSION
    periodic_command(period, 43, 0, { MSG_COMM, 0x93 }); //! ADCS MISSION
    periodic_command(period, 46, 0, { MSG_COMM, 0x93 }); //! ADCS MISSION
    periodic_command(period, 49, 0, { MSG_COMM, 0x93 }); //! ADCS MISSION
    
    periodic_command_clear_rx_flag(period, 52);         //! APRS MISSION
    periodic_command(period, 52, 0, { MSG_COMM, 0x95 }); //! APRS MISSION
    periodic_command(period, 55, 0, { MSG_COMM, 0x95 }); //! APRS MISSION
    periodic_command(period, 58, 0, { MSG_COMM, 0x95 }); //! APRS MISSION
    periodic_command(period, 61, 0, { MSG_COMM, 0x95 }); //! APRS MISSION
    periodic_command(period, 64, 0, { MSG_COMM, 0x95 }); //! APRS MISSION

    periodic_command_clear_rx_flag(period, 67);         //! EOBC MISSION
    periodic_command(period, 67, 0, { MSG_COMM, 0x96 }); //! EOBC MISSION
    periodic_command(period, 70, 0, { MSG_COMM, 0x96 }); //! EOBC MISSION
    periodic_command(period, 73, 0, { MSG_COMM, 0x96 }); //! EOBC MISSION
    periodic_command(period, 76, 0, { MSG_COMM, 0x96 }); //! EOBC MISSION
    periodic_command(period, 79, 0, { MSG_COMM, 0x96 }); //! EOBC MISSION

    periodic_command_clear_rx_flag(period, 82);         //! TUM MISSION
    periodic_command(period, 82, 0, { MSG_COMM, 0x97 }); //! TUM MISSION
    periodic_command(period, 85, 0, { MSG_COMM, 0x97 }); //! TUM MISSION
    periodic_command(period, 88, 0, { MSG_COMM, 0x97 }); //! TUM MISSION
    periodic_command(period, 91, 0, { MSG_COMM, 0x97 }); //! TUM MISSION
    periodic_command(period, 94, 0, { MSG_COMM, 0x97 }); //! TUM MISSION

    periodic_command_clear_rx_flag(period, 97);          //! OSIL MISSION
    periodic_command(period, 97, 0, { MSG_COMM, 0x94 }); //! OSIL MISSION
    periodic_command(period, 100, 0, { MSG_COMM, 0x94 }); //! OSIL MISSION
    periodic_command(period, 103, 0, { MSG_COMM, 0x94 }); //! OSIL MISSION
    periodic_command(period, 106, 0, { MSG_COMM, 0x94 }); //! OSIL MISSION
    periodic_command(period, 109, 0, { MSG_COMM, 0x94 }); //! OSIL MISSION

    periodic_command_clear_rx_flag(period,112);   
    periodic_command(period, 112, 1, { MSG_COMM, 0xF8 }); //! Telemetry generation
}

void periodic_tasks()
{
    if (clock_update) {
        disable_interrupts(GLOBAL); // Disable interrupts to avoid race conditions
        clock_update = 0;
        current_time = time(0);
        //! ctime(&current_time, &current_time_str);
        if (obc_flags.led_on_off) 
            ledBlinking();        

        add_new_commands();
        if (current_time != previous_time) {
            scheduled_command_check();
            periodic_requests_commands();
        }
        struct_tm* local_time = localtime(&current_time);
        if (local_time->tm_hour == 23 && local_time->tm_min == 59 && local_time->tm_sec == 59) {
            execute(1, { MSG_COMM, 0x55 }); //! Save state
            fprintf(PC, "No 24h reset happened, doing soft reset instead.\r\n");
            reset_cpu();
        }

        previous_time = current_time;
        if(current_time % 30 == 0) {
            uart_clean(COMM);
            uart_clean(MSNBOSS);
            uart_clean(APRS);
            uart_clean(EOBC);
            uart_clean(OSIL);
            uart_clean(TUM);
            uart_clean(ADCS);
            uart_clean(FAB);
            uart_clean(RST);
            uart_clean(PC);
         }
        enable_interrupts(GLOBAL); // Re-enable interrupts after critical section
        }
}

#endif //! MAIN_H
