/**
 * @file distance_sensors.h
 *
 * @brief distance sensors related
 */

#ifndef __DISTANCE_SENSORS_H__
#define __DISTANCE_SENSORS_H__

#include "gpio.h"
#include <stdbool.h>
#include <stdint.h>

/*****************************************
 * DEFINES
 *****************************************/

typedef enum {
    WATCH_LEFT_SIDE,
    WATCH_LEFT_WIDE,
    WATCH_LEFT_UPPER,
    WATCH_LEFT_MID,
    WATCH_RIGHT_MID,
    WATCH_RIGHT_UPPER,
    WATCH_RIGHT_WIDE,
    WATCH_RIGHT_SIDE,
    ATTACK_SENSOR
} distance_sensor_type;

typedef enum {
    HIDDEN,
    LEFT_SIDE,
    FRONTAL,
    RIGHT_SIDE,
    ATTACK
} opponent_status_type;

/*****************************************
 * LOW LEVEL MODULES
 *****************************************/

/**
 * @brief Read distance sensor value from GPIO pin.
 *
 * @param sensor uint16_t
 *
 * @return Value of the sensor
 *
 */
bool distance_sensor_read(distance_sensor_type sensor);

/*****************************************
 * MID LEVEL MODULES
 *****************************************/

/**
 * @brief Read value from all watch distance sensors
 *
 * @return Bit-to-bit mapping of the sensors status
 *
 */
uint8_t watch_sensor_status();

/**
 * @brief return if watch sensor is seeing
 *
 * @param sensor watch sensor
 *
 * @return if watch sensor is seeing
 *
 */
bool watch_sensor_seeing(distance_sensor_type sensor);

/**
 * @brief return if attack sensor is seeing
 *
 * @return if attack sensor is seeing
 *
 */
bool attack_sensor_seeing();

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

/**
 * @brief return opponent position based on watch sensors angles (except side sensors) Position is Angle divided by 25 (25° -> 1)
 *
 * @return opponent positio
 *
 */
float opponent_position();

/**
 * @brief return opponent position based on watch sensors
 *
 * @return opponent position based on opponent_position_type
 *
 */
opponent_status_type opponent_status();

#endif // __DISTANCE_SENSORS_H__
