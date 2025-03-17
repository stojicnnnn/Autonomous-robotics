/*
 * tajmer.c
 *
 *  Created on: Feb 20, 2024
 *      Author: HP
 */


/*
 * timer.c
 *
 *  Created on: Jan 24, 2024
 *      Author: HP
 */

#include <stdint.h>
#include <stdbool.h>
#include "biblioteka/tajmer.h"
#include "biblioteka/odometrija.h"
#include "biblioteka/pid.h"
#include "stm32f4xx.h"

static void tim2_init();
static void enc1_init();
static void enc2_init();


volatile int faza1=1;
static uint32_t vreme_pozicija = 10;
volatile bool flag_100s=false;
static float x;
static float y;
static float theta;
static int brojac=0;
static int temp=0;
static int p=0;
static int z=0;
static int det=0;
static int test=0;

static float a=0;
static float  b=0;
static float c=0;



void tajmer_init()
{
  tim2_init ();

}

void enc_init(){
	 enc1_init();
	 enc2_init();
}

static void
tim2_init ()
{
  RCC->APB2ENR |= (0b1 << 0);

  // 84MHz -> 1MHz
  TIM1->PSC = 84 - 1;
  // 1MHz -> 1kHz
  TIM1->ARR = 1000 - 1;

  TIM1->CR1 &= ~(0b1 << 1);
  TIM1->CR1 &= ~(0b1 << 2);

  TIM1->EGR |= (0b1 << 0);

  TIM1->DIER |= (0b1 << 0);

  while (!(TIM1->SR & (0b1 << 0)))
    {
      __NOP();
    }
  TIM1->SR &= ~(0b1 << 0);

  TIM1->CR1 |= (0b1 << 2);

  uint8_t const TIM1_PREKID = 25;
  NVIC->ISER[0] |= (0b1 << TIM1_PREKID);

  TIM1->CR1 |= (0b1 << 0);
}

static void enc1_init()
{

	RCC->AHB1ENR |= (1 << 0); // A port
	RCC->AHB1ENR |= (1 << 1); // B port



	GPIOB->MODER |=  (0b10 << 3*2); //B3
	GPIOA->MODER |=  (0b10 << 5*2); //A5

	GPIOB->AFR[0] |=  (0b0001 << 3* 4);
	GPIOA->AFR[0] |=  (0b0001 << 5* 4);


	RCC->APB1ENR |= (0b1 << 0);

	TIM2->PSC = 0;
	TIM2->ARR = 0xFFFF;


	TIM2->SMCR &= ~(0b111 << 0 * 3);
	TIM2->SMCR |=  (0b011 << 0 * 3);

	TIM2->CCMR1 &= ~(0b11 << 0 | 0b11 << 8);
	TIM2->CCMR1 |=  (0b01 << 0 | 0b01 << 8);

	TIM2->CR1 |= (0b1 << 0);
	TIM2->EGR |= (0b1 << 0);

	TIM2->CNT = 65536/2;
}

int16_t enc1_GetInc()
{
	int16_t cnt = TIM2->CNT- 65536/2;
	TIM2->CNT = 65536/2;

	return cnt;
}


static void enc2_init()
{

	RCC->AHB1ENR |= (1 << 0); // a port

		GPIOA->MODER |= (0b10 << 0*2); // A0
		GPIOA->MODER |= (0b10 << 1*2); // A1

		GPIOA->OSPEEDR |= (0b11 << 0*2);
		GPIOA->OSPEEDR |= (0b11 << 1*2);

		GPIOA->AFR[0] |= (0b0010<< 0*4);
		GPIOA->AFR[0] |= (0b0010 << 1*4);


	RCC->APB1ENR |= (0b1 << 3);

	TIM5->PSC = 0;
	TIM5->ARR = 0xFFFF;

	TIM5->SMCR &= ~(0b111 << 0 * 3);
	TIM5->SMCR |=  (0b011 << 0 * 3);

	TIM5->CCMR1 &= ~(0b11 << 0 | 0b11 << 8);
	TIM5->CCMR1 |=  (0b01 << 0 | 0b01 << 8);

	TIM5->CR1 |= (0b1 << 0);
	TIM5->EGR |= (0b1 << 0);

	TIM5->CNT = 65536/2;
}

int16_t enc2_GetInc()
{
	int16_t cnt = TIM5->CNT -65536/2;
	TIM5->CNT = 65536/2;

	return cnt;
}


void set_x(float nova){
	x=nova;
}

void set_det(float nova){
	det=nova;
}

void set_y(float nova){
	y=nova;
}

void set_theta(float nova){
	theta=nova;
}
void set_test(int nova){
	theta=nova;
}


void TIM1_UP_TIM10_IRQHandler()
{

  if ((TIM1->SR & (0b1 << 0)) == (0b1 << 0))
    {

	  odometrija(1);

	  regulacija_brzine();

	  a = get_x();
	  b = get_y();
	  c = get_theta();
	  if(test==1){
		  pid_init();
		  racunanje_brzine(0,0);
	  }
	  else{
	  if (sistemsko_vreme % vreme_pozicija == 0){
		 if(faza1==0){
		  		  regulacija_pozicije(get_x(), get_y(), get_theta(), get_x(),  get_y(),  get_theta());
		  	  }
		  else
		  regulacija_pozicije(x, y, theta, get_x(),  get_y(),  get_theta());

		  }
	  }
	  if(!(GPIOC->IDR & (1<<11))){
		  sistemsko_vreme++;
		  if(sistemsko_vreme == 100000)
		     	 flag_100s=true;
	  }

	  if((GPIOC->IDR & (1<<0)) | (GPIOC->IDR & (1<<2)) | (GPIOC->IDR & (1<<14))){ //zadnja strana
		if(det==1){
			if(brojac==0){

			 temp=faza1;
		 }
		  faza1=0;
		  brojac=1;
		  z=0;
		  test=1;
		}

	  }
	  else{
		  z=1;
	  }



	  if((GPIOC->IDR & (1<<3)) | (GPIOC->IDR & (1<<12)) | (GPIOA->IDR & (1<<15))){ //prednja strana
		if(det==0){
			if(brojac==0){
			 temp=faza1;
		 }
		   faza1=0;
		   brojac=1;
		   p=0;
		   test=1;
		}
	  }
	  else{
		  p=1;
	  }

	  if((brojac==1) & ((z==1) & (p==1))){
		  faza1=temp;
		  temp=0;
		  brojac=0;
		  pid_init();
		  test=0;
	  	  }

	/*  if((GPIOC->IDR & (1<<0)))
	  	  {
	  		  Z3=1;
	  	  }
	  else
		  Z3=0;

	  if((GPIOC->IDR & (1<<2)))
	  	  	  {
	  	  		  Z1=1;
	  	  	  }
	  else
		  Z1=0;

	  if((GPIOC->IDR & (1<<3)))
	  	  	  {
	  	  		  P2=1;
	  	  	  }
	  else
		  P2=0;

	  if((GPIOC->IDR & (1<<13)))
	  	  	  {
	  	  		  Z2=1;
	  	  	  }
	  else
		  Z2=0;

	  if((GPIOC->IDR & (1<<12)))
	  	  	  {
	  	  		  P3=1;
	  	  	  }
	  else
		  P3=0;

	  if((GPIOA->IDR & (1<<15)))
	  	  	  {
	  	  		  P1=1;
	  	  	  }
	  else
		  P1=0;
*/
	  }

      TIM1->SR &= ~(0b1 << 0);
      if(sistemsko_vreme == 100000)
    	  flag_100s=true;



}
