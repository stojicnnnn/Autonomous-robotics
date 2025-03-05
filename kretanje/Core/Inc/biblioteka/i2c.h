/*
 * i2c.h
 *
 *  Created on: 22.02.2025.
 *      Author: Nikola
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "biblioteka/servo.h"


void I2C1_Init(void);
void i2c1_BurstWrite(char slave_address, char memory_address, int number_bytes_to_write, char *data);

#endif /* INC_I2C_H_ */
