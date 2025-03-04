/**
 * @file fsm_control.h
 *
 * @brief Motors control Finite State Machine related
 */

#ifndef __FSM_CONTROL_H__
#define __FSM_CONTROL_H__

#include <stdint.h>
#include <stdio.h>

/*****************************************
 * DEFINES
 *****************************************/

typedef enum {
    CONTROL_RC,
    CONTROL_INIT_MOVE,
    CONTROL_SEARCH,
    CONTROL_SIDE,
    CONTROL_FOLLOW,
    CONTROL_ATTACK,
    CONTROL_FRONT_LINE,
} fsm_control_state_type;

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

/**
 * @brief init robot motors control state machine
 */
void fsm_control_init();

/**
 * @brief stop robot motors control state machine
 */
void fsm_control_stop();

/**
 * @brief defines action within each state in control state machine
 */
void fsm_control_state_running();

/**
 * @brief defines action within each state in control state machine
 */
void fsm_control_state_init(fsm_control_state_type next_state);

/**
 * @brief set the control state machine event transitions handler
 */
void fsm_control_transition();

/**
 * @brief set the control state machine to run
 */
void fsm_control_run();

#endif // __FSM_CONTROL_H__
