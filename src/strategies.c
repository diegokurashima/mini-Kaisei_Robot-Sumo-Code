/**
 * @file strategies.c
 *
 * @brief Robot Strategies related
 */

#include "strategies.h"
#include "pid.h"
#include "routines.h"

#include "fsm_control.h"

#include "distance_sensors.h"
#include "line_sensors.h"
#include "mcu.h"
#include "motors.h"
#include "led.h"

/*****************************************
 * DEFINES
 *****************************************/

// GENERAL DEFINES
#define MAX_SPEED (100)

// INIT MOVE DEFINES
#define AHEAD_AHEAD_TIME (500)
#define AHEAD_AHEAD_SPEED (MAX_SPEED)

// FOLLOW DEFINES
#define FOLLOW_PID_KP (40)
#define FOLLOW_PID_KI (0.01)
#define FOLLOW_PID_KD (0)
#define FOLLOW_PID_MAX (100)
#define FOLLOW_PID_MIN (-100)

#define FOLLOW_OUTPUT_MIN (10)

// ATTACK DEFINES
#define ATTACK_PID_KP (10)
#define ATTACK_PID_KI (0)
#define ATTACK_PID_KD (0)
#define ATTACK_PID_MAX (100)
#define ATTACK_PID_MIN (-100)

// FRONT LINE DEFINES
#define FRONT_LINE_REVERSION_TIME (50)
#define FRONT_LINE_REVERSION_SPEED (MAX_SPEED)

#define FRONT_LINE_FULL_TURN_TIME (45)
#define FRONT_LINE_HALF_TURN_TIME (35)
#define FRONT_LINE_TURN_SPEED (MAX_SPEED)

#define FRONT_LINE_TRANSITION_ENABLED_TIME (FRONT_LINE_REVERSION_TIME + 10)

/*****************************************
 * GLOBAL VARIABLES
 *****************************************/

// STRATEGIES VARIABLES
bool init_move_transition_enabled = 0;
bool init_move_done = 0;

bool front_line_transition_enabled = 0;
bool front_line_done = 0;

// FSM_MAIN VARIABLES
extern uint8_t strategy_1, strategy_2;
extern uint8_t strategy;

/*****************************************
 * PRIVATE VARIABLES
 *****************************************/

// INIT MOVE VARIABLES
uint32_t init_move_timer;

// SEARCH VARIABLES
uint32_t search_timer;

// SIDE VARIABLES
uint32_t side_timer;

// FOLLOW VARIABLES
uint32_t follow_timer;
pid_type follow_pid;

// ATTACK VARIABLES
uint32_t attack_timer;
pid_type attack_pid;

// FRONT LINE VARIABLES
uint32_t front_line_timer;
bool front_line_turn_direction; // 0 - RIGHT Turn, 1 - LEFT Turn
bool front_line_turn_full;      // 0 - HALF Turn, 1 - FULL Turn

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

/*****************
 * INIT MOVE MODULES
 *****************/

void strategy_init_move_init() {
    reset_timer(init_move_timer);
    init_move_transition_enabled = 0;
    init_move_done = 0;
}

void strategy_init_move_run() {
    switch (strategy) {
        // STOPPED
        case 11 ... 12:
            init_move_transition_enabled = 1;
            init_move_done = 1;
            motors_control(0, 0);
            break;

        // AHEAD
        case 13 ... 14:
            init_move_transition_enabled = 1;

            // AHEAD Move
            if (get_timer(init_move_timer) < AHEAD_AHEAD_TIME) {
                motors_control(AHEAD_AHEAD_SPEED, AHEAD_AHEAD_SPEED);
            } else {
                init_move_done = 1;
                motors_control(0, 0);
            }

            break;

        // STOPPED : Same as 11 ... 12
        default:
            init_move_transition_enabled = 1;
            init_move_done = 1;
            motors_control(0, 0);
            break;
    }
}

/*****************
 * SEARCH MODULES
 *****************/

void strategy_search_init() {
    reset_timer(search_timer);
    routine_step_reset();
}

void strategy_search_run() {
    routine_step_run();
}

/*****************
 * SIDE MODULES
 *****************/

void strategy_side_init() {
    reset_timer(side_timer);
}

void strategy_side_run() {
    // OPPONENT at LEFT SIDE
    if (opponent_status() == LEFT_SIDE) {
        motors_control(-MAX_SPEED, 0);

        // OPPONENT at RIGHT SIDE
    } else if (opponent_status() == RIGHT_SIDE) {
        motors_control(0, -MAX_SPEED);
    }
}

/*****************
 * FOLLOW MODULES
 *****************/

void strategy_follow_init() {
    reset_timer(follow_timer);
    routine_step_reset();
    pid_reset(&follow_pid, 0, FOLLOW_PID_KP, FOLLOW_PID_KI, FOLLOW_PID_KD, FOLLOW_PID_MAX, FOLLOW_PID_MIN);
}

void strategy_follow_run() {
    float follow_pid_output;
    follow_pid_output = pid_update(&follow_pid, opponent_position());

    // NOT ALIGNED
    if ((follow_pid_output > FOLLOW_OUTPUT_MIN) || (follow_pid_output < -FOLLOW_OUTPUT_MIN)) {
        motors_control(follow_pid_output, -follow_pid_output);

        // ALIGNED
    } else {
        routine_step_run();
    }
}

/*****************
 * ATTACK MODULES
 *****************/

void strategy_attack_init() {
    reset_timer(attack_timer);
    pid_reset(&attack_pid, 0, ATTACK_PID_KP, ATTACK_PID_KI, ATTACK_PID_KD, ATTACK_PID_MAX, ATTACK_PID_MIN);
}

void strategy_attack_run() {
    float attack_pid_output;
    attack_pid_output = pid_update(&attack_pid, opponent_position());
    motors_control(MAX_SPEED + attack_pid_output, MAX_SPEED - attack_pid_output);
}

/*****************
 * FRONT LINE MODULES
 *****************/

void strategy_front_line_init() {
    reset_timer(front_line_timer);
    front_line_transition_enabled = 0;
    front_line_done = 0;

    // CONFIG TURN SETTINGS
    // Both Sensors on Line
    if ((line_sensor_on_line(LINE_SENSOR_FRONT_RIGHT) == 1) && (line_sensor_on_line(LINE_SENSOR_FRONT_LEFT) == 1)) {
        front_line_turn_full = 0;
        front_line_turn_direction = 1;

        // Right Sensor Only
    } else if (line_sensor_on_line(LINE_SENSOR_FRONT_RIGHT) == 1) {
        front_line_turn_full = 1;
        front_line_turn_direction = 1;

        // Left Sensor Only
    } else if (line_sensor_on_line(LINE_SENSOR_FRONT_LEFT) == 1) {
        front_line_turn_full = 1;
        front_line_turn_direction = 0;
    }
}

void strategy_front_line_run() {
    // REVERSION
    if (get_timer(front_line_timer) < FRONT_LINE_REVERSION_TIME) {
        motors_control(-FRONT_LINE_REVERSION_SPEED, -FRONT_LINE_REVERSION_SPEED);

        // TURN - Full or Half Turn
    } else if (((get_timer(front_line_timer) < FRONT_LINE_REVERSION_TIME + FRONT_LINE_FULL_TURN_TIME) &&
                (front_line_turn_full == 1)) ||
               ((get_timer(front_line_timer) < FRONT_LINE_REVERSION_TIME + FRONT_LINE_HALF_TURN_TIME) &&
                (front_line_turn_full == 0))) {
        if (front_line_turn_direction == 1) {
            // Turn LEFT
            motors_control(-FRONT_LINE_TURN_SPEED, FRONT_LINE_TURN_SPEED);
        } else {
            // Turn RIGHT
            motors_control(FRONT_LINE_TURN_SPEED, -FRONT_LINE_TURN_SPEED);
        }

        // ACTION DONE
    } else {
        motors_control(0, 0);
        front_line_done = 1;
    }

    // TRANSITION ENABLED TIME
    if (get_timer(front_line_timer) < FRONT_LINE_TRANSITION_ENABLED_TIME) {
        front_line_transition_enabled = 1;
    }
}
