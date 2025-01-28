/*
 * step.c
 *
 *  Created on: 17. mar 2024.
 *      Author: HP
 */
#include "biblioteka/tajmer.h"
#include "biblioteka/step.h"

static void init_pwm_gpio();


void init_pwm(){
	init_pwm_gpio();


	//RCC->APB1ENR |= (1 << 2);
	RCC->APB2ENR |= (1 << 16);
	                    // 84MHZ
	TIM9->PSC = 1000-1; //84KHZ
	TIM9->ARR = 420-1; // 300HZ  600  1000  200

	//TIM4->CCR1 = (280-1)*0.7; //FAKTOR ISPUNE ?
	TIM9->CCMR1 &= ~(0b111 << 4);
	TIM9->CCMR1 |= (0b110 << 4);
	TIM9->CCMR1 |= (1 << 3);
	TIM9->CR1 |= (1 << 7);

	TIM9->EGR |= (1 << 0);
	TIM9->CCER |=(1 << 0);




}
void init_pwm_gpio(){


	GPIOA->MODER &= ~(0b11 << 2*2);
	GPIOA->MODER |= (0b10 << 2*2);   // A2
	//GPIOB->MODER |= (0b01 << 4*2);

	GPIOA->OSPEEDR |= (0b11 << 2*2);
	//GPIOB->OSPEEDR |= (0b11 << 4*2);

	GPIOA->AFR[0] |= (0b0011 << 2*4);
	//GPIOB->AFR[0] |= (0b0010 << 4*4);

}

void start_pwm(){
	TIM9->CR1 |= (1 << 0);


}

void faktor_ispune(float a){
	TIM9->CCR1 = (420-1)*a; //FAKTOR ISPUNE ?
}

void frekvencija(int a){
	//84KHZ
	TIM9->ARR = a-1;
}

void STOP_pwm(){
	TIM9->CR1 &= ~(1 << 0);
}


void dizanje_gore (int pocetno_vreme, int arr){
	frekvencija(arr);
	faktor_ispune(0.5);
	GPIOB->ODR |= (1 << 5);//input 1 //1    D4   SLP  sad je ddir

	while(sistemsko_vreme < pocetno_vreme + 1300);

	faktor_ispune(0.0);

}

void dizanje_malo (int pocetno_vreme, int arr){


    frekvencija(arr);  //200h
	faktor_ispune(0.5);
	GPIOB->ODR |= (1 << 5);//input 1 //1    D4   SLP  sad je ddir

	while(sistemsko_vreme < pocetno_vreme + 100);

	faktor_ispune(0.0);

}


void spustanje (int pocetno_vreme, int arr){

	frekvencija(arr);  //100h
	faktor_ispune(0.5);
	GPIOB->ODR &= ~(1 << 5);//input 1 //1    D4   SLP  sad je ddir

	while(sistemsko_vreme < pocetno_vreme + 2300);

	faktor_ispune(0.0);

}


