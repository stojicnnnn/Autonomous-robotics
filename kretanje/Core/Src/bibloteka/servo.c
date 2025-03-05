/*
 * servo.c
 *
 *  Created on: 22.02.2025.
 *      Author: Nikola
 */
#include "biblioteka/servo.h"
#include "biblioteka/i2c.h"

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>


char data;
 uint8_t data_rec[6];

 void PCA9685_WriteReg(char regAddr, char data) {
     write(regAddr,data);
 }

 void PCA9685_SetPWM(char channel, int on, int off) {
     char data[4];
     data[0] = on & 0xFF;       // Lower 8 bits of ON time
     data[1] = (on >> 8) & 0x0F; // Upper 4 bits of ON time
     data[2] = off & 0xFF;       // Lower 8 bits of OFF time
     data[3] = (off >> 8) & 0x0F; // Upper 4 bits of OFF time

     write(0x06 + 4 * channel,data);

 }

 void PCA9685_Init() {
     PCA9685_WriteReg(0x00, 0x20);  // MODE1: Enable auto-increment
     PCA9685_WriteReg(0x01, 0x04);  // MODE2: Configure output mode
 }

 void SetServoPWM(char channel, int pulseLength) {
     PCA9685_SetPWM(channel, 0, pulseLength);
 }

 void write (uint8_t reg, char value)
 {
 	char data[1];
 	data[0] = value;

 	i2c1_BurstWrite( DEVICE_ADDR, reg,1, data) ;
 }
