/*
 * servo.h
 *
 *  Created on: 22.02.2025.
 *      Author: Nikola
 */

#ifndef INC_BIBLIOTEKA_SERVO_H_
#define INC_BIBLIOTEKA_SERVO_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "biblioteka/i2c.h"

void servo_set_position(uint8_t motor, uint8_t position);
void servo_set_freq(uint16_t freq);

#endif /* INC_BIBLIOTEKA_SERVO_H_ */
