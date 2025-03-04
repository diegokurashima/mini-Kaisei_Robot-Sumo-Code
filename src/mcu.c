/**
 * @file mcu.c
 *
 * @brief MCU related
 */

#include "mcu.h"

#include "motors.h"

/*****************************************
 * INITIALIZATION MODULES
 *****************************************/

void mcu_init(void) {
    /*
     * Standard initialization
     */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
}

void adc_init() {
    MX_DMA_Init();
    MX_ADC1_Init();
}

void motors_init() {
    // Init TIMER 1
    MX_TIM1_Init();

    // Enable PWM Generation
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Setup Configuration
    motors_direction_setup();
}

void infrared_receiver_init() {
    // Init TIMER 4
    MX_TIM4_Init();
    HAL_TIM_Base_Start(&htim4);
}
