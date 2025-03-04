/**
 * @file fsm_main.c
 *
 * @brief Main Finite State Machine related
 */

#include "fsm_main.h"
#include "fsm_control.h"
#include "led.h"

/*****************************************
 * DEFINES
 *****************************************/

#define MODE_TEST (true)

/*****************************************
 * GLOBAL VARIABLES
 *****************************************/

// FSM_MAIN VARIABLES
fsm_main_state_type fsm_main_state_current;
uint8_t strategy_1, strategy_2, startState = 0;
uint8_t strategy;

bool changeNumBits;

bool modeTest = MODE_TEST;

// INFRARED_RECEIVER VARIABLES
extern uint8_t ir_value3bits, ir_value2bits;

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

void fsm_main_init() {
    fsm_main_state_init(MAIN_WAIT_STRATEGY_1);
}

void fsm_main_state_running() {
    switch (fsm_main_state_current) {
        case MAIN_WAIT_STRATEGY_1:
            led_control(LED_1, LED_TOGGLE);
            HAL_Delay(100);
            break;

        case MAIN_WAIT_STRATEGY_2:
            led_control(LED_2, LED_TOGGLE);
            HAL_Delay(100);
            break;

        case MAIN_WAIT:
            led_control(LED_1, LED_TOGGLE);
            led_control(LED_2, LED_TOGGLE);
            HAL_Delay(500);
            break;

        case MAIN_SET:
            break;

        case MAIN_RUNNING:
            fsm_control_run();
            break;

        case MAIN_STOPPED:
            fsm_control_stop();
            break;

        default:
            break;
    }
}

void fsm_main_state_init(fsm_main_state_type next_state) {
    switch (next_state) {
        case MAIN_WAIT_STRATEGY_1:
            changeNumBits = true;
            ir_value2bits = 0;
            ir_value3bits = 0;
            strategy_1 = 0;
            strategy_2 = 0;
            strategy = 0;
            fsm_main_state_current = MAIN_WAIT_STRATEGY_1;

            led_control(LED_1, LED_RESET);
            led_control(LED_2, LED_SET);
            break;

        case MAIN_WAIT_STRATEGY_2:
            ir_value2bits = 0;
            ir_value3bits = 0;
            fsm_main_state_current = MAIN_WAIT_STRATEGY_2;

            led_control(LED_1, LED_SET);
            led_control(LED_2, LED_RESET);
            break;

        case MAIN_WAIT:
            changeNumBits = false;
            ir_value2bits = 0;
            ir_value3bits = 0;
            fsm_main_state_current = MAIN_WAIT;

            led_control(LED_1, LED_RESET);
            led_control(LED_2, LED_RESET);
            break;

        case MAIN_SET:
            ir_value2bits = 0;
            ir_value3bits = 0;
            fsm_main_state_current = MAIN_SET;

            // Blink Sinalization
            for (int i = 0; i < 5; i++) {
                led_control(LED_1, LED_TOGGLE);
                led_control(LED_2, LED_TOGGLE);
                HAL_Delay(20);
            }

            led_control(LED_1, LED_SET);
            led_control(LED_2, LED_RESET);

            break;

        case MAIN_RUNNING:
            ir_value2bits = 0;
            ir_value3bits = 0;
            fsm_main_state_current = MAIN_RUNNING;

            led_control(LED_1, LED_SET);
            led_control(LED_2, LED_SET);

            fsm_control_init();
            break;

        case MAIN_STOPPED:
            ir_value2bits = 0;
            ir_value3bits = 0;
            fsm_main_state_current = MAIN_STOPPED;

            led_control(LED_1, LED_RESET);
            led_control(LED_2, LED_SET);

            fsm_control_stop();
            break;

        default:
            break;
    }
}

void fsm_main_transition() {
    switch (fsm_main_state_current) {
        case MAIN_WAIT_STRATEGY_1:

            if (ir_value3bits != 0) {
                strategy_1 = ir_value3bits;
                HAL_Delay(2000); // Delay to WAIT_STRATEGY_2
                fsm_main_state_init(MAIN_WAIT_STRATEGY_2);
            }

            break;

        case MAIN_WAIT_STRATEGY_2:

            if (ir_value3bits != 0) {
                strategy_2 = ir_value3bits;
                HAL_Delay(2000); // Delay to WAIT

                // Set strategy code number
                strategy = strategy_1 * 10 + strategy_2;
                fsm_main_state_init(MAIN_WAIT);
            }

            break;

        case MAIN_WAIT:

            if (ir_value2bits == 1) {
                startState = ir_value2bits;
                fsm_main_state_init(MAIN_SET);
            } else if (ir_value2bits == 4) {
                startState = ir_value2bits;
                ir_value2bits = 0;
                ir_value3bits = 0;
                HAL_Delay(2000); // Delay to WAIT_STRATEGY_1
                fsm_main_state_init(MAIN_WAIT_STRATEGY_1);
            }

            break;

        case MAIN_SET:

            if (ir_value2bits == 2) {
                startState = ir_value2bits;
                fsm_main_state_init(MAIN_RUNNING);
            } else if (ir_value2bits == 1) {
                startState = ir_value2bits;
                fsm_main_state_init(MAIN_SET);
            } else if (ir_value2bits == 4) {
                startState = ir_value2bits;
                ir_value2bits = 0;
                ir_value3bits = 0;
                HAL_Delay(2000); // Delay to WAIT
                fsm_main_state_init(MAIN_WAIT);
            }

            break;

        case MAIN_RUNNING:

            if (ir_value2bits == 3) {
                startState = ir_value2bits;
                fsm_main_state_init(MAIN_STOPPED);
            }

            break;

        case MAIN_STOPPED:

            if ((modeTest == true) && (ir_value2bits == 4)) {
                startState = ir_value2bits;
                ir_value2bits = 0;
                ir_value3bits = 0;
                HAL_Delay(2000); // Delay to WAIT
                fsm_main_state_init(MAIN_WAIT);
            }

            break;

        default:
            fsm_main_state_init(MAIN_STOPPED);
            break;
    }
}

void fsm_main_run() {
    fsm_main_state_running();
    fsm_main_transition();
}
