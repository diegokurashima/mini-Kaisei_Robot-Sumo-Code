/**
 * @file distance_sensors.c
 *
 * @brief distance sensors related
 */

#include "distance_sensors.h"
#include "led.h"

/*****************************************
 * DEFINES
 *****************************************/
#define WATCH_LEFT_SIDE_ANGLE (-75)
#define WATCH_LEFT_WIDE_ANGLE (-50)
#define WATCH_LEFT_MID_ANGLE (-25)
#define WATCH_LEFT_UPPER_ANGLE (-10)
#define WATCH_CENTER_ANGLE (0)
#define WATCH_RIGHT_UPPER_ANGLE (10)
#define WATCH_RIGHT_MID_ANGLE (25)
#define WATCH_RIGHT_WIDE_ANGLE (50)
#define WATCH_RIGHT_SIDE_ANGLE (75)
#define NUM_DIST_SENSOR 9

/*****************************************
 * LOW LEVEL MODULES
 *****************************************/

bool distance_sensor_read(distance_sensor_type sensor) {
    switch (sensor) {
        case ATTACK_SENSOR:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
            break;

        case WATCH_LEFT_SIDE:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
            break;

        case WATCH_LEFT_WIDE:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
            break;

        case WATCH_LEFT_MID:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
            break;

        case WATCH_LEFT_UPPER:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
            break;

        case WATCH_RIGHT_UPPER:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
            break;

        case WATCH_RIGHT_MID:
            return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
            break;

        case WATCH_RIGHT_WIDE:
            return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
            break;

        case WATCH_RIGHT_SIDE:
            return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
            break;

        default:
            return GPIO_PIN_SET;
            break;
    }
}

/*****************************************
 * MID LEVEL MODULES
 *****************************************/

uint8_t watch_sensor_status() {
    uint8_t values = 0b0;

    for (int i = 0; i < NUM_DIST_SENSOR - 1; i++) {
        if (distance_sensor_read(i) == GPIO_PIN_RESET) {
            // Seeing Bit -> Set
            values |= (1 << (i - 1));
        } else if (distance_sensor_read(i) == GPIO_PIN_SET) {
            // Not Seeing Bit -> Reset
            values &= ~(1 << (i - 1));
        }
    }

    return values;
}

bool watch_sensor_seeing(distance_sensor_type sensor) {
    return distance_sensor_read(sensor) == GPIO_PIN_RESET;
}

bool attack_sensor_seeing() {
    // attack_sensor_state = distance_sensor_read(ATTACK_SENSOR);
    return distance_sensor_read(ATTACK_SENSOR) == GPIO_PIN_RESET;
}

/*****************************************
 * HIGH LEVEL MODULES
 *****************************************/

float opponent_position() {
    // position is sum of angles divided by active sensors (except side sensors)
    float angle = (watch_sensor_seeing(WATCH_LEFT_WIDE) * WATCH_LEFT_WIDE_ANGLE +
                   watch_sensor_seeing(WATCH_LEFT_UPPER) * WATCH_LEFT_UPPER_ANGLE +
                   watch_sensor_seeing(WATCH_LEFT_MID) * WATCH_LEFT_MID_ANGLE +
                   watch_sensor_seeing(WATCH_RIGHT_MID) * WATCH_RIGHT_MID_ANGLE +
                   watch_sensor_seeing(WATCH_RIGHT_UPPER) * WATCH_RIGHT_UPPER_ANGLE +
                   watch_sensor_seeing(WATCH_RIGHT_WIDE) * WATCH_RIGHT_WIDE_ANGLE) /
                  (watch_sensor_seeing(WATCH_LEFT_WIDE) + watch_sensor_seeing(WATCH_LEFT_UPPER) +
                   watch_sensor_seeing(WATCH_LEFT_MID) + watch_sensor_seeing(WATCH_RIGHT_MID) +
                   watch_sensor_seeing(WATCH_RIGHT_UPPER) + watch_sensor_seeing(WATCH_RIGHT_WIDE));

    return angle / 25;
}

opponent_status_type opponent_status() {
    // ATTACK
    if ((attack_sensor_seeing() == 1) &&
        ((watch_sensor_seeing(WATCH_LEFT_MID) == 1) && (watch_sensor_seeing(WATCH_RIGHT_MID) == 1))) {
        return ATTACK;
    }

    // SIDES
    if (watch_sensor_seeing(WATCH_LEFT_SIDE) == 1) {
        return LEFT_SIDE;
    } else if (watch_sensor_seeing(WATCH_RIGHT_SIDE) == 1) {
        return RIGHT_SIDE;

        // HIDDEN
    } else if (watch_sensor_status() == 0b0) {
        return HIDDEN;

        // FRONTAL
    } else {
        return FRONTAL;
    }
}
