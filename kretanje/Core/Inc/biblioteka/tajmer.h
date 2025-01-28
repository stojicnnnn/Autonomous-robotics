/*
 * tajmer.h
 *
 *  Created on: Feb 20, 2024
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_TAJMER_H_
#define INC_BIBLIOTEKA_TAJMER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

void tajmer_init ();
void enc_init();
int16_t enc1_GetInc();
int16_t enc2_GetInc();
void set_x(float nova);
void set_y(float nova);
void set_theta(float nova);
void set_det(float nova);
void set_test(int nova);

volatile int faza1;
volatile bool flag_100s;
volatile uint32_t sistemsko_vreme;

#endif /* INC_BIBLIOTEKA_TAJMER_H_ */
