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

void pca9685_init() {
    I2C1_Start();
    I2C1_Write(0x40 << 1);  // PCA9685 I2C Address
    I2C1_Write(0x00);  // MODE1 register
    I2C1_Write(0x10);  // Sleep mode to set prescaler
    I2C1_Stop();
}


void servo_set_position(uint8_t motor, uint8_t position)
{
    uint8_t data[4];
    uint16_t pwm_val = ((position * (410 - 205)) / 180) + 205 ;  // Map position (0-180°) to PWM range (0-4095)

    data[0] = 0x00;  // ON_L
    data[1] = 0x00;  // ON_H
    data[2] = pwm_val & 0xFF;  // OFF_L
    data[3] = pwm_val >> 8;    // OFF_H

    uint8_t register_address = 0x06 + (motor * 4);

    I2C1_Start();
    I2C1_Write(0x40 << 1);  // PCA9685 I2C Address
    I2C1_Write(register_address);  // Servo motor register
    for (int i = 0; i < 4; i++) {
        I2C1_Write(data[i]);
    }
    I2C1_Stop();
}

void servo_set_freq(uint16_t freq)
{
    uint8_t prescale = (25000000 / (4096 * freq)) - 1;

    I2C1_Start();
    I2C1_Write(0x40 << 1);
    I2C1_Write(0x00);  // MODE1 register
    I2C1_Write(0xFE);  // PRE_SCALE register
    I2C1_Write(prescale);
    I2C1_Stop();
}
