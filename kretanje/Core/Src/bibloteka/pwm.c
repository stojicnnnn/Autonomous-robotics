/*
 * pwm.c
 *
 *  Created on: Feb 21, 2024
 *      Author: HP
 */

#include <stdint.h>
#include <stdbool.h>
#include "biblioteka/tajmer.h"
#include "biblioteka/pwm.h"
#include "stm32f4xx.h"
#include <math.h>

static uint32_t jedan;
static uint32_t dva;


void init_PWM()
{

	//M1_PWM -> PC7 ( TIM3_CH2), AF2
	//M2_PWM -> PB6 ( TIM4_CH1), AF2
	RCC->AHB1ENR |= (1 << 1);
	RCC->AHB1ENR |= (1 << 2);

	GPIOB->MODER &= ~(0b11 << 6*2);   // PB6
	GPIOB->MODER |= (0b10 << 6*2);

	GPIOB->AFR[0] &= ~(0b1111 << 6 * 4);
	GPIOB->AFR[0] |= (0b0010 << 6*4);

	GPIOC->MODER &= ~(0b11 << 7*2);   //PC7
	GPIOC->MODER |= (0b10 << 7*2);

	GPIOC->AFR[0] &= ~(0b1111 << 7 * 4);
	GPIOC->AFR[0] |= (0b0010 << 7*4);

	//TIM3

	RCC->APB1ENR |= (1 << 1);
		                    // 84MHZ
	TIM3->PSC = 1-1; //84MHZ
	TIM3->ARR = 2100-1; // 20KHZ
	TIM3->CCR2 = 0.0;
	///TIM4->CCR1 = (500-1)*0.5; //FAKTOR ISPUNE ?
	TIM3->CCMR1 &= ~(0b111 << 12);
	TIM3->CCMR1 |= (0b110 << 12);
	TIM3->CCMR1 |= (1 << 11);

	TIM3->CCR2 = 0UL; // NE ZZNAM STA JE
	TIM3->EGR |= (1 << 0);
	TIM3->CCER |=(1 << 4);
	TIM3->CR1 |= (1 << 7);
	TIM3->CR1 |= (0b1 << 0);

	// TIM4

	RCC->APB1ENR |= (1 << 2);
			                    // 84MHZ
	TIM4->PSC = 1-1; //84MHZ
	TIM4->ARR = 2100-1; // 20KHZ
	TIM4->CCR1 = 0.0;
	///TIM4->CCR1 = (500-1)*0.5; //FAKTOR ISPUNE ?
	TIM4->CCMR1 &= ~(0b111 << 4);
	TIM4->CCMR1 |= (0b110 << 4);
	TIM4->CCMR1 |= (1 << 3);

	TIM4->CCR1 = 0UL; // NE ZZNAM STA JE
	TIM4->EGR |= (1 << 0);
	TIM4->CCER |=(1 << 0);
	TIM4->CR1 |= (1 << 7);
	TIM4->CR1 |= (0b1 << 0);

}


// TODO UINXT promeniti tip duc
void motor1_set_PWM(int16_t duc)
{
	 if(duc<0) {
			// GPIOA->MODER &=  ~(0b01 << 8 * 2);
		 GPIOA->ODR &= ~(0b01 << 8);
		}
	 else if(duc>=0){
		 GPIOA->ODR |= (0b01 << 8);
	 }
	 duc = fabs(duc);
	if (duc > 2100-1){
		duc = 2100-1;

	}
	TIM3->CCR2 =  duc;
	jedan = TIM3->CCR2;
}

void motor2_set_PWM(int16_t duc)
{

	if(duc<0) {
				GPIOA->ODR &= ~(0b01 << 9);
			}
	else if(duc>=0){
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
		GPIOA->ODR |=  (0b01 << 9);
	}
	duc = fabs(duc);
	if (duc > 2100-1){
			duc = 2100-1;

		}
	TIM4->CCR1 = duc;
	dva = TIM4->CCR1;
}
