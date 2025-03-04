/**
 * @file fsm_main.h
 *
 * @brief Main Finite State Machine related
 */

#ifndef __FSM_MAIN_H__
#define __FSM_MAIN_H__

#include <stdint.h>
#include <stdio.h>

/*****************************************
 * DEFINES
 *****************************************/

typedef enum {
    MAIN_WAIT_STRATEGY_1,
    MAIN_WAIT_STRATEGY_2,
    MAIN_WAIT,
    MAIN_SET,
    MAIN_RUNNING,
    MAIN_STOPPED
} fsm_main_state_type;

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

/**
 * @brief init main state machine
 */
void fsm_main_init();

/**
 * @brief defines action within each state in main state machine
 */
void fsm_main_state_running();

/**
 * @brief defines action within state initialization, after transition defined
 */
void fsm_main_state_init(fsm_main_state_type next_state);

/**
 * @brief set the main state machine event transitions handler
 */
void fsm_main_transition();

/**
 * @brief set the main state machine to run
 */
void fsm_main_run();

#endif // __FSM_MAIN_H__
