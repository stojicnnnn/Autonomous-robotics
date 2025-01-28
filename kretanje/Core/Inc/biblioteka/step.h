/*
 * step.h
 *
 *  Created on: 17. mar 2024.
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_STEP_H_
#define INC_BIBLIOTEKA_STEP_H_

#include "stm32f4xx.h"
#include <stdint.h>


// prototip
void init_pwm();
void start_pwm();
void STOP_pwm();
void faktor_ispune(float a);
void frekvencija(int a);
void dizanje_gore (int pocetno_vreme, int arr);
void dizanje_malo (int pocetno_vreme, int arr);
void spustanje (int pocetno_vreme, int arr);

#endif /* INC_BIBLIOTEKA_STEP_H_ */
