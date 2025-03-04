/**
 * @file strategies.h
 *
 * @brief Robot strategies related
 */

#ifndef __STRATEGIES_H__
#define __STRATEGIES_H__

#include <stdint.h>
#include <stdio.h>

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

/*****************
 * INIT MOVE MODULES
 *****************/

/**
 * @brief Set the robot initial conditions for the INIT MOVE STATE
 */
void strategy_init_move_init();

/**
 * @brief Set the robot strategies for the INIT MOVE STATE
 */
void strategy_init_move_run();

/*****************
 * SEARCH MODULES
 *****************/

/**
 * @brief Set the robot initial conditions for the SEARCH STATE
 */
void strategy_search_init();

/**
 * @brief Set the robot strategy control for the SEARCH STATE
 */
void strategy_search_run();

/*****************
 * SIDE MODULES
 *****************/

/**
 * @brief Set the robot initial conditions for the SIDE STATE
 */
void strategy_side_init();

/**
 * @brief Set the robot strategy control for the SIDE STATE
 */
void strategy_side_run();

/*****************
 * FOLLOW MODULES
 *****************/

/**
 * @brief Set the robot initial conditions for the FOLLOW STATE
 */
void strategy_follow_init();

/**
 * @brief Set the robot strategy control for the FOLLOW STATE
 */
void strategy_follow_run();

/*****************
 * ATTACK MODULES
 *****************/

/**
 * @brief Set the robot initial conditions for the WATCH STATE
 */
void strategy_attack_init();

/**
 * @brief Set the robot strategy control for the WATCH STATE
 */
void strategy_attack_run();

/*****************
 * FRONT LINE MODULES
 *****************/

/**
 * @brief Set the robot initial conditions for the FRONT LINE STATEA
 */
void strategy_front_line_init();

/**
 * @brief Set the robot strategy control for the FRONT LINE STATEA
 */
void strategy_front_line_run();

#endif // __STRATEGIES_H__
