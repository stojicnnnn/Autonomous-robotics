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
//Reserved for DC motors
	GPIOA->MODER &= ~(0b11 << 7*2); // Clear mode bits for PA7
	GPIOA->MODER |= (0b10 << 7*2);  // Set PA7 to alternate function mode
	GPIOA->AFR[0] &= ~(0b1111 << 7 * 4); // Clear alternate function bits for PA7
	GPIOA->AFR[0] |= (0b0010 << 7*4);    // Set alternate function to AF2 (TIM3_CH2) 
//Reserved for servos
	GPIOA->MODER &= ~(0b11 << 6*2); // Clear mode bits for PA6
	GPIOA->MODER |= (0b10 << 6*2);  // Set PA6 to alternate function mode
	GPIOA->AFR[0] &= ~(0b1111 << 6 * 4); // Clear alternate function bits for PA6
	GPIOA->AFR[0] |= (0b0010 << 6*4);    // Set alternate function to AF2 (TIM3_CH1)

	GPIOB->MODER &= ~(0b11 << 0*2); // Clear mode bits for PB0
	GPIOB->MODER |= (0b10 << 0*2);  // Set PB0 to alternate function mode
	GPIOB->AFR[0] &= ~(0b1111 << 0 * 4); // Clear alternate function bits for PB0
	GPIOB->AFR[0] |= (0b0010 << 0*4);    // Set alternate function to AF2 (TIM3_CH3)

	GPIOB->MODER &= ~(0b11 << 1*2); // Clear mode bits for PB1
	GPIOB->MODER |= (0b10 << 1*2);  // Set PB1 to alternate function mode
	GPIOB->AFR[0] &= ~(0b1111 << 1 * 4); // Clear alternate function bits for PB1
	GPIOB->AFR[0] |= (0b0010 << 1*4);    // Set alternate function to AF2 (TIM3_CH4)

	GPIOC->MODER &= ~(0b11 << 7*2);   //PC7
	GPIOC->MODER |= (0b10 << 7*2);
	GPIOC->AFR[0] &= ~(0b1111 << 7 * 4);
	GPIOC->AFR[0] |= (0b0010 << 7*4);

	//TIM3

	RCC->APB1ENR |= (1 << 1);
		                    // 84MHZ
	TIM3->PSC = 1-1; //84MHZ
	TIM3->ARR = 2100-1; // 20KHZ
	/ Set initial duty cycle for each channel
	TIM3->CCR1 = 0.0;
	TIM3->CCR2 = 0.0;
	TIM3->CCR3 = 0.0;
	TIM3->CCR4 = 0.0;
	///TIM4->CCR1 = (500-1)*0.5; //FAKTOR ISPUNE ?

	// Configure PWM mode for each channel
	TIM3->CCMR1 &= ~(0b111 << 4);  // Clear CCMR1 bits for channel 1
	TIM3->CCMR1 |= (0b110 << 4);   // Set PWM mode 1 for channel 1
	TIM3->CCMR1 |= (1 << 3);       // Enable preload register for channel 1

	TIM3->CCMR1 &= ~(0b111 << 12); // Clear CCMR1 bits for channel 2
	TIM3->CCMR1 |= (0b110 << 12);  // Set PWM mode 1 for channel 2
	TIM3->CCMR1 |= (1 << 11);      // Enable preload register for channel 2

	TIM3->CCMR2 &= ~(0b111 << 4);  // Clear CCMR2 bits for channel 3
	TIM3->CCMR2 |= (0b110 << 4);   // Set PWM mode 1 for channel 3
	TIM3->CCMR2 |= (1 << 3);       // Enable preload register for channel 3

	TIM3->CCMR2 &= ~(0b111 << 12); // Clear CCMR2 bits for channel 4
	TIM3->CCMR2 |= (0b110 << 12);  // Set PWM mode 1 for channel 4
	TIM3->CCMR2 |= (1 << 11);      // Enable preload register for channel 4


	TIM3->CCR2 = 0UL; // NE ZZNAM STA JE
	TIM3->EGR |= (1 << 0);         // Generate an update event
	TIM3->CCER |= (1 << 0);        // Enable capture/compare for channel 1
	TIM3->CCER |= (1 << 4);        // Enable capture/compare for channel 2
	TIM3->CCER |= (1 << 8);        // Enable capture/compare for channel 3
	TIM3->CCER |= (1 << 12);       // Enable capture/compare for channel 4

	TIM3->CR1 |= (1 << 7); // Enable auto-reload preload
	TIM3->CR1 |= (0b1 << 0);  // Enable TIM3

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
//Used for servo control,right now possible to control 3 different servos with 3 different PWM values
//If needed,can add 3 more channels to TIM4,and init a new timer with 4 channels,making 10 different PWM channels
void set_PWM_duty_cycle(TIM_TypeDef *TIMx, uint8_t channel, uint32_t duty_cycle_percentage) {
    uint32_t ccr_value = (TIMx->ARR + 1) * duty_cycle_percentage / 100;

    switch (channel) {
        case 1:
            TIMx->CCR1 = ccr_value;
            break;
        case 3:
            TIMx->CCR3 = ccr_value;
            break;
        case 4:
            TIMx->CCR4 = ccr_value;
            break;
        default:
            //Invalid channel
            break;
    }
}
