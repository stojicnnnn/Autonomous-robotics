/*
 * pwm.h
 *
 *  Created on: Feb 21, 2024
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_PWM_H_
#define INC_BIBLIOTEKA_PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

void init_PWM();
void motor1_set_PWM(int16_t duc);
void motor2_set_PWM(int16_t duc);
void set_PWM_duty_cycle(TIM_TypeDef *TIMx, uint8_t channel, uint32_t duty_cycle_percentage) ;

#endif /* INC_BIBLIOTEKA_PWM_H_ */
