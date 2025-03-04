/**
 * @file line_sensors.c
 *
 * @brief line sensors related
 */

#include "line_sensors.h"

/*****************************************
 * DEFINES
 *****************************************/

#define ADC_BUFFER_SIZE (256)

#define WHITE_THRESHOLD (100)
#define FRONT_RIGHT_WHITE_THRESHOLD (500)
#define FRONT_LEFT_WHITE_THRESHOLD (500)
#define REAR_RIGHT_WHITE_THRESHOLD (100)
#define REAR_LEFT_WHITE_THRESHOLD (100)

/*****************************************
 * GLOBAL VARIABLES
 *****************************************/

// LINE_SENSORS VARIABLES
uint32_t line_sensor_values[4];
uint32_t adc_buffer[256]; // ADC_BUFFER_SIZE

/*****************************************
 * LOW LEVEL MODULES
 *****************************************/

void line_sensors_read_enable() {
    HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUFFER_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // Aux array
    uint32_t val[4] = {0};

    for (int i = 0; i < 4; i++) {
        val[i] = 0;
    }

    // Sum of values
    for (int i = 0; i < ADC_BUFFER_SIZE / 4; i++) {
        for (int j = 0; j < 4; j++) {
            val[j] += adc_buffer[4 * i + j];
        }
    }

    // Average calculation
    for (int i = 0; i < 4; i++) {
        val[i] /= ADC_BUFFER_SIZE / 4;
    }

    // Line sensor values handling
    for (int i = 0; i < 4; i++) {
        line_sensor_values[i] = val[i];
    }
}

uint32_t line_sensor_read(line_sensor_type sensor) {
    return line_sensor_values[sensor];
}

/*****************************************
 * MID LEVEL MODULES
 *****************************************/

bool line_sensor_on_line(line_sensor_type sensor) {
    switch (sensor) {
        case LINE_SENSOR_FRONT_RIGHT:
            return line_sensor_values[sensor] < FRONT_RIGHT_WHITE_THRESHOLD;
            break;

        case LINE_SENSOR_FRONT_LEFT:
            return line_sensor_values[sensor] < FRONT_LEFT_WHITE_THRESHOLD;
            break;

        case LINE_SENSOR_REAR_RIGHT:
            return line_sensor_values[sensor] < REAR_RIGHT_WHITE_THRESHOLD;
            break;

        case LINE_SENSOR_REAR_LEFT:
            return line_sensor_values[sensor] < REAR_LEFT_WHITE_THRESHOLD;
            break;

        default:
            return 0;
            break;
    }
}

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

bool frontal_on_line() {
    return (line_sensor_on_line(LINE_SENSOR_FRONT_RIGHT) == 1 || line_sensor_on_line(LINE_SENSOR_FRONT_LEFT) == 1);
}

bool rear_on_line() {
    return (line_sensor_on_line(LINE_SENSOR_REAR_RIGHT) == 1 || line_sensor_on_line(LINE_SENSOR_REAR_LEFT) == 1);
}
