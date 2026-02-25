#include "hal/board_configuration.h"
#include "main.h"


int main(int argc, char** argv)
{
    init_all_uart();
    init_all_spi();
    satellite_init();         //! General satellite initialization routine
    hardware_specific_init(); //! Hardware specific initialization routine

    while (true) {
        hardware_specific_loop();
        periodic_tasks();
        software_uart_tasks();
    }
    return 0;
}
