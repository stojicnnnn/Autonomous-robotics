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


#define DEVICE_ADDR        0x40  // Default I2C address,From datasheet
#define MODE1_REG           0x00  // Mode register 1
#define PRE_SCALE_REG       0xFE  // Prescaler for PWM frequency
#define ALL_LED_OFF_L       0xFC  // Turns all LEDs OFF (Low byte)
#define ALL_LED_OFF_H       0xFD  // Turns all LEDs OFF (High byte)

 void PCA9685_WriteReg(char regAddr, char data);
 void PCA9685_SetPWM(char channel, int on, int off);
 void PCA9685_Init();
 void SetServoPWM(char channel, int pulseLength);
 void write (uint8_t reg, char value);
#endif /* INC_BIBLIOTEKA_SERVO_H_ */
