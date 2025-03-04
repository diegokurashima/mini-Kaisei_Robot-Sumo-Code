/**
 * @file led.h
 *
 * @brief led related
 */

#ifndef __LED_H__
#define __LED_H__

#include "gpio.h"
#include <stdbool.h>
#include <stdint.h>

/*****************************************
 * DEFINITIONS
 *****************************************/
#define LED_1 (1)
#define LED_2 (2)

#define LED_RESET (0)
#define LED_SET (1)
#define LED_TOGGLE (2)

/*****************************************
 * LOW LEVEL MODULES
 *****************************************/

/**
 * @brief Control led GPIO pin.
 *
 * @param led led to be controlled
 * @param led_action action defined for the led
 *
 */
void led_control(uint16_t led, uint16_t led_action);

#endif // __LED_H__
