/**
 * @file main.c
 *
 * @brief Main function
 */

#include "distance_sensors.h"
#include "led.h"
#include "line_sensors.h"
#include "mcu.h"
#include "motors.h"

#include "fsm_control.h"
#include "fsm_main.h"

/*****************************************
 * GLOBAL VARIABLES
 *****************************************/

/*****************************************
 * MAIN LOOP
 *****************************************/

int main(void) {
    /*
     * Initialization
     */
    mcu_init();
    adc_init();
    motors_init();
    infrared_receiver_init();
    line_sensors_read_enable();

    /*
     * State Machine to run
     */
    fsm_main_init();

    for (;;) {
        fsm_main_run();
    }
}
