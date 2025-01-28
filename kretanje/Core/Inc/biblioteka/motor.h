/*
 * motor.h
 *
 *  Created on: Feb 21, 2024
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_MOTOR_H_
#define INC_BIBLIOTEKA_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

void motor_init();
void enable_motors();

#endif /* INC_BIBLIOTEKA_MOTOR_H_ */
