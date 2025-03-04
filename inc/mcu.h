/**
 * @file mcu.h
 *
 * @brief MCU related
 */

#ifndef __MCU_H__
#define __MCU_H__

#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include <stdint.h>

/*****************************************
 * DEFINES
 *****************************************/
#define reset_timer(ticks) (ticks = HAL_GetTick())
#define get_timer(ticks) (HAL_GetTick() - ticks)

/*****************************************
 * INITIALIZATION MODULES
 *****************************************/

/**
 * @brief Initializes MCU and some peripherals.
 */
void mcu_init(void);

/**
 * @brief Initializes System Clock.
 * @note  Defined by cube.
 */
void SystemClock_Config(void);

/**
 * @brief Initializes ADC channels.
 * @note
 */
void adc_init();

/**
 * @brief Initializes motors.
 * @note
 */
void motors_init();

/**
 * @brief Initializes servo motor.
 * @note
 */
void servo_motor_init();

/**
 * @brief Initializes infrared receiver.
 * @note
 */
void infrared_receiver_init();

#endif // __INIT_H__
