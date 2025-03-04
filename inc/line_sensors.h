/**
 * @file line_sensors.h
 *
 * @brief line_sensors
 */

#ifndef __LINE_SENSORS_H__
#define __LINE_SENSORS_H__

#include "adc.h"
#include <stdbool.h>
#include <stdint.h>

/*****************************************
 * DEFINES
 *****************************************/

typedef enum {
    LINE_SENSOR_FRONT_RIGHT,
    LINE_SENSOR_FRONT_LEFT,
    LINE_SENSOR_REAR_RIGHT,
    LINE_SENSOR_REAR_LEFT
} line_sensor_type;

/*****************************************
 * LOW LEVEL MODULES
 *****************************************/

/**
 * @brief Enable line sensor reading
 *
 * @return void. Value is retrieved from DMA
 *
 */
void line_sensors_read_enable();

/**
 * @brief Callback of line sensor
 *
 * @return void.
 *
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/**
 * @brief Read line sensor value
 *
 * @param sensor - line sensor to get value
 *
 * @return Line sensor reading value.
 *
 */
uint32_t line_sensor_read(line_sensor_type sensor);

/*****************************************
 * MID LEVEL MODULES
 *****************************************/

/**
 * @brief Get if line sensor value is on line
 *
 * @param sensor - line sensor to check
 *
 * @return If line sensor is on line.
 *
 */
bool line_sensor_on_line(line_sensor_type sensor);

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

/**
 * @brief Get if frontal sensors are on line
 *
 * @return If frontal of robot is on line.
 *
 */
bool frontal_on_line();

/**
 * @brief Get if rear sensors are on line
 *
 * @return If rear of robot is on line.
 *
 */
bool rear_on_line();

#endif // __LINE_SENSORS_H__
