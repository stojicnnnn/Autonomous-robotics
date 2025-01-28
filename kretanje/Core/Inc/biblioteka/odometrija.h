/*
 * odometrija.h
 *
 *  Created on: Feb 20, 2024
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_ODOMETRIJA_H_
#define INC_BIBLIOTEKA_ODOMETRIJA_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "biblioteka/tajmer.h"
#include <math.h>

#define d_tocka 56 //79.51 // mm  // 77.72
#define razmak_tockova 300 //297.78 // mm  //297,27
#define obim_tocka (M_PI * d_tocka)

volatile float v_l;
volatile float v_r;

float inc_2_rad();
 float inc_2_mm();

void odometrija_init();
void odometrija();
float get_x();
float get_y();
float get_theta();
float get_w ();
float get_v ();
void set_theta_smer();


#endif /* INC_BIBLIOTEKA_ODOMETRIJA_H_ */
