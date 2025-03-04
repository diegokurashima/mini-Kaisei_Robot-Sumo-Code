/**
 * @file fsm_control.c
 *
 * @brief Motors control Finite State Machine related
 */

#include "fsm_control.h"
#include "fsm_main.h"

#include "strategies.h"

#include "motors.h"
#include "distance_sensors.h"
#include "line_sensors.h"
#include "led.h"

/*****************************************
 * DEFINES
 *****************************************/

#define TRUE (1)
#define FALSE (0)

/*****************************************
 * GLOBAL VARIABLES
 *****************************************/

// FSM_CONTROL VARIABLES
fsm_control_state_type fsm_control_state_current;

// FSM_MAIN VARIABLES
extern fsm_main_state_type fsm_main_state_current;
extern uint8_t strategy;
extern uint8_t strategy_1, strategy_2;

// STRATEGIES VARIABLES
extern bool init_move_transition_enabled;
extern bool init_move_done;

extern bool front_line_transition_enabled;
extern bool front_line_done;

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

void fsm_control_init() {
    fsm_control_state_init(CONTROL_INIT_MOVE);
}

void fsm_control_stop() {
    fsm_control_state_current = -1;
    motors_control(0, 0);
}

void fsm_control_state_running() {
    switch (fsm_control_state_current) {
        case CONTROL_INIT_MOVE:
            strategy_init_move_run();
            break;

        case CONTROL_SEARCH:
            strategy_search_run();
            break;

        case CONTROL_SIDE:
            strategy_side_run();
            break;

        case CONTROL_FOLLOW:
            strategy_follow_run();
            break;

        case CONTROL_ATTACK:
            strategy_attack_run();
            break;

        case CONTROL_FRONT_LINE:
            strategy_front_line_run();
            break;

        default:
            motors_control(0, 0);
            break;
    }
}

void fsm_control_state_init(fsm_control_state_type next_state) {
    switch (next_state) {
        case CONTROL_INIT_MOVE:
            fsm_control_state_current = CONTROL_INIT_MOVE;
            strategy_init_move_init();
            break;

        case CONTROL_SEARCH:
            fsm_control_state_current = CONTROL_SEARCH;
            strategy_search_init();
            break;

        case CONTROL_SIDE:
            fsm_control_state_current = CONTROL_SIDE;
            strategy_side_init();
            break;

        case CONTROL_FOLLOW:
            fsm_control_state_current = CONTROL_FOLLOW;
            strategy_follow_init();
            break;

        case CONTROL_ATTACK:
            fsm_control_state_current = CONTROL_ATTACK;
            strategy_attack_init();
            break;

        case CONTROL_FRONT_LINE:
            fsm_control_state_current = CONTROL_FRONT_LINE;
            strategy_front_line_init();
            break;

        default:
            break;
    }
}

void fsm_control_transition() {
    switch (fsm_control_state_current) {
        case CONTROL_INIT_MOVE:

            // Transition must be ENABLED for this state
            if (!init_move_transition_enabled) {
                break;
            }

            // -> ATTACK
            if (opponent_status() == ATTACK) {
                fsm_control_state_init(CONTROL_ATTACK);
                break;
            }

            // -> FRONT_LINE
            if (frontal_on_line() == TRUE) {
                fsm_control_state_init(CONTROL_FRONT_LINE);
                break;
            }

            // -> SIDE
            if ((opponent_status() == LEFT_SIDE) || (opponent_status() == RIGHT_SIDE)) {
                fsm_control_state_init(CONTROL_SIDE);
                break;
            }

            // -> FOLLOW
            if (opponent_status() == FRONTAL) {
                fsm_control_state_init(CONTROL_FOLLOW);
                break;
            }

            // Done -> SEARCH
            if (init_move_done) {
                fsm_control_state_init(CONTROL_SEARCH);
                break;
            }

            break;

        case CONTROL_SEARCH:

            // -> ATTACK
            if (opponent_status() == ATTACK) {
                fsm_control_state_init(CONTROL_ATTACK);
                break;
            }

            // -> FRONT_LINE
            if (frontal_on_line() == TRUE) {
                fsm_control_state_init(CONTROL_FRONT_LINE);
                break;
            }

            // -> SIDE
            if ((opponent_status() == LEFT_SIDE) || (opponent_status() == RIGHT_SIDE)) {
                fsm_control_state_init(CONTROL_SIDE);
                break;
            }

            // -> FOLLOW
            if (opponent_status() == FRONTAL) {
                fsm_control_state_init(CONTROL_FOLLOW);
                break;
            }

            break;

        case CONTROL_SIDE:

            // -> ATTACK
            if (opponent_status() == ATTACK) {
                fsm_control_state_init(CONTROL_ATTACK);
                break;
            }

            // -> FRONT_LINE
            if (frontal_on_line() == TRUE) {
                fsm_control_state_init(CONTROL_FRONT_LINE);
                break;
            }

            // -> FOLLOW
            if (opponent_status() == FRONTAL) {
                fsm_control_state_init(CONTROL_FOLLOW);
                break;
            }

            // -> SEARCH
            if (opponent_status() == HIDDEN) {
                fsm_control_state_init(CONTROL_SEARCH);
                break;
            }

            break;

        case CONTROL_FOLLOW:

            // -> ATTACK
            if (opponent_status() == ATTACK) {
                fsm_control_state_init(CONTROL_ATTACK);
                break;
            }

            // -> FRONT_LINE
            if (frontal_on_line() == TRUE) {
                fsm_control_state_init(CONTROL_FRONT_LINE);
                break;
            }

            // -> SIDE
            if ((opponent_status() == LEFT_SIDE) || (opponent_status() == RIGHT_SIDE)) {
                fsm_control_state_init(CONTROL_SIDE);
                break;
            }

            // -> SEARCH
            if (opponent_status() == HIDDEN) {
                fsm_control_state_init(CONTROL_SEARCH);
                break;
            }

            break;

        case CONTROL_ATTACK:

            // -> FRONT_LINE
            if (frontal_on_line() == TRUE) {
                fsm_control_state_init(CONTROL_FRONT_LINE);
                break;
            }

            // -> SIDE
            if ((opponent_status() == LEFT_SIDE) || (opponent_status() == RIGHT_SIDE)) {
                fsm_control_state_init(CONTROL_SIDE);
                break;
            }

            // -> FOLLOW
            if (opponent_status() == FRONTAL) {
                fsm_control_state_init(CONTROL_FOLLOW);
                break;
            }

            // -> SEARCH
            if (opponent_status() == HIDDEN) {
                fsm_control_state_init(CONTROL_SEARCH);
                break;
            }

            break;

        case CONTROL_FRONT_LINE:

            // Transition must be ENABLED for this state
            if (!front_line_transition_enabled) {
                break;
            }

            // -> ATTACK
            if (opponent_status() == ATTACK) {
                fsm_control_state_init(CONTROL_ATTACK);
                break;
            }

            // -> SIDE
            if ((opponent_status() == LEFT_SIDE) || (opponent_status() == RIGHT_SIDE)) {
                fsm_control_state_init(CONTROL_SIDE);
                break;
            }

            // -> FOLLOW
            if (opponent_status() == FRONTAL) {
                fsm_control_state_init(CONTROL_FOLLOW);
                break;
            }

            // Done -> SEARCH
            if (front_line_done) {
                fsm_control_state_init(CONTROL_SEARCH);
                break;
            }

            break;

        default:
            break;
    }
}

void fsm_control_run() {
    fsm_control_state_running();
    fsm_control_transition();
}
