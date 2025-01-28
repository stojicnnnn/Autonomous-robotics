/*
 * motor.c
 *
 *  Created on: Feb 21, 2024
 *      Author: HP
 */

#include <stdint.h>
#include <stdbool.h>
#include "biblioteka/pwm.h"
#include "stm32f4xx.h"
#include "biblioteka/motor.h"

void motor_init(){

	RCC->AHB1ENR |= (1 << 0);
	RCC->AHB1ENR |= (1 << 1);

	GPIOA->MODER &= ~(0b11 << 8 * 2);
	GPIOA->MODER |=  (0b01 << 8 * 2);  // DIR A8

	GPIOB->MODER &= ~(0b11 << 10 * 2);
	GPIOB->PUPDR &= ~(0b11 << 10 * 2);
	GPIOB->PUPDR |=  (0b01 << 10 * 2);  // FLT B10

	GPIOA->MODER &= ~(0b11 << 10 * 2);
	GPIOA->MODER |=  (0b01 << 10 * 2);
	GPIOA->ODR |= (0b1 << 10); 			//SLP A10


	GPIOA->MODER &= ~(0b11 << 9 * 2);	// DIR A9
	GPIOA->MODER |=  (0b01 << 9 * 2);

	GPIOA->MODER &= ~(0b11 << 6 * 2);    // FLT A6
	GPIOA->PUPDR &= ~(0b11 << 6 * 2);
	GPIOA->PUPDR |=  (0b01 << 6 * 2);

	GPIOB->MODER &= ~(0b11 << 8 * 2);
	GPIOB->MODER |=  (0b01 << 8 * 2); 	// SLP B8
	GPIOB->ODR |= (0b1 << 8);

}

void enable_motors(){
	init_PWM();
	motor1_set_PWM(2000);
    motor2_set_PWM(2000);

}
