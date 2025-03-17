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
	//1.  C12 front sensors
			GPIOC->MODER &= ~(0b11 << 12*2); //Clears the mode bits for pin 12 of GPIO port C.
			GPIOC->MODER |= (0b00 << 12*2);  //Sets the mode bits for pin 12 to 00, which configures it as an input.
			GPIOC->PUPDR &= ~(0b11 << 12*2);  //Clears the pull-up/pull-down bits for pin 12 of GPIO port C.
			GPIOC->PUPDR |= (0b10 << 12*2);  //Sets the pull-up/pull-down bits for pin 12 to 10, which configures it with a pull-down resistor.
		//2.   C3
			GPIOC->MODER &= ~(0b11 << 3*2); 
			GPIOC->MODER |= (0b00 << 3*2);  
			GPIOC->PUPDR &= ~(0b11 << 3*2);  
			GPIOC->PUPDR |= (0b10 << 3*2);  
		//3.  A15
			GPIOA->MODER &= ~(0b11 << 15*2); 
			GPIOA->MODER |= (0b00 << 15*2);  
			GPIOA->PUPDR &= ~(0b11 << 15*2);  
			GPIOA->PUPDR |= (0b10 << 15*2);  

			//rear sensors
			//1.  c13
			GPIOC->MODER &= ~(0b11 << 13*2); 
			GPIOC->MODER |= (0b00 << 13*2);  
			GPIOC->PUPDR &= ~(0b11 << 13*2);  
			GPIOC->PUPDR |= (0b10 << 13*2);  
			//2.   C0
			GPIOC->MODER &= ~(0b11 << 0*2); 
			GPIOC->MODER |= (0b00 << 0*2);  
			GPIOC->PUPDR &= ~(0b11 << 0*2);  
			GPIOC->PUPDR |= (0b10 << 0*2);  
			//3.  C2
			GPIOC->MODER &= ~(0b11 << 2*2); 
			GPIOC->MODER |= (0b00 << 2*2);  
			GPIOC->PUPDR &= ~(0b11 << 2*2);  
			GPIOC->PUPDR |= (0b10 << 2*2);  */
}


/*    senzor
	  if((GPIOA->IDR & (1<<11)))
	  {
		  a=1;
	  }
*/
