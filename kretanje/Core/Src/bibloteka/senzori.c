/*
 * senzori.c
 *
 *  Created on: 1. apr 2024.
 *      Author: HP
 */
#include <stdint.h>
#include <stdbool.h>
#include "biblioteka/tajmer.h"
#include "biblioteka/senzori.h"

void init_senzor(){


	RCC->AHB1ENR |= (1 << 2); // c port
	RCC->AHB1ENR |= (1 << 0);  // a port
	//1.  C12
			GPIOC->MODER &= ~(0b11 << 12*2);  // kliruje
			GPIOC->MODER |= (0b00 << 12*2);  // stavlja ga na input
			GPIOC->PUPDR &= ~(0b11 << 12*2);  // kliruje pin za input bitova
			GPIOC->PUPDR |= (0b10 << 12*2);  // Setujem pull-down
		//2.   C3
			GPIOC->MODER &= ~(0b11 << 3*2);  // kliruje
			GPIOC->MODER |= (0b00 << 3*2);  // stavlja ga na input
			GPIOC->PUPDR &= ~(0b11 << 3*2);  // kliruje pin za input bitova
			GPIOC->PUPDR |= (0b10 << 3*2);  // Setujem pull-down
		//3.  A15
			GPIOA->MODER &= ~(0b11 << 15*2);  // kliruje
			GPIOA->MODER |= (0b00 << 15*2);  // stavlja ga na input
			GPIOA->PUPDR &= ~(0b11 << 15*2);  // kliruje pin za input bitova
			GPIOA->PUPDR |= (0b10 << 15*2);  // Setujem pull-down

			//ZADNJA STRANA
			//1.  c13
			GPIOC->MODER &= ~(0b11 << 13*2);  // kliruje
			GPIOC->MODER |= (0b00 << 13*2);  // stavlja ga na input
			GPIOC->PUPDR &= ~(0b11 << 13*2);  // kliruje pin za input bitova
			GPIOC->PUPDR |= (0b10 << 13*2);  // Setujem pull-down
			//2.   C0
			GPIOC->MODER &= ~(0b11 << 0*2);  // kliruje
			GPIOC->MODER |= (0b00 << 0*2);  // stavlja ga na input
			GPIOC->PUPDR &= ~(0b11 << 0*2);  // kliruje pin za input bitova
			GPIOC->PUPDR |= (0b10 << 0*2);  // Setujem pull-down
			//3.  C2
			GPIOC->MODER &= ~(0b11 << 2*2);  // kliruje
			GPIOC->MODER |= (0b00 << 2*2);  // stavlja ga na input
			GPIOC->PUPDR &= ~(0b11 << 2*2);  // kliruje pin za input bitova
			GPIOC->PUPDR |= (0b10 << 2*2);  // Setujem pull-down*/
}


/*    senzor
	  if((GPIOA->IDR & (1<<11)))
	  {
		  a=1;
	  }
*/
