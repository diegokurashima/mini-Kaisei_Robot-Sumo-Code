/**
 * @file led.c
 *
 * @brief led related
 */

#include "led.h"

/*****************************************
 * LOW LEVEL MODULES
 *****************************************/

void led_control(uint16_t led, uint16_t led_action) {
    GPIO_TypeDef* port;
    uint16_t pin;

    if (led == LED_1) {
        port = GPIOB;
        pin = GPIO_PIN_5;
    } else if (led == LED_2) {
        port = GPIOB;
        pin = GPIO_PIN_7;
    } else {
        return;
    }

    switch (led_action) {
        case LED_RESET:
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            break;

        case LED_SET:
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
            break;

        case LED_TOGGLE:
            HAL_GPIO_TogglePin(port, pin);
            break;

        default:
            break;
    }
}
