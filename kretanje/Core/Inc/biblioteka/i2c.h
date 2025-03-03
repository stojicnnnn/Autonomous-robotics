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
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);
uint8_t I2C1_Read(uint8_t ack);
void I2C1_WriteRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C1_ReadRegister(uint8_t deviceAddr, uint8_t regAddr);
void pca9685_init();
uint8_t I2C1_CheckDevice(uint8_t address);
#endif /* INC_I2C_H_ */
