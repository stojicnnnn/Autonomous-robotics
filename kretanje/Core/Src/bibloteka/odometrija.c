/*
 * odometrija.c
 *
 *  Created on: Feb 20, 2024
 *      Author: HP
 */

#include "biblioteka/odometrija.h"
#include "biblioteka/tajmer.h"
#include "biblioteka/pid.h"

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include <math.h>



static float w_l;
static float w_r;
static int enc_l;
static int enc_r;
static float dt_sec;

static float x;
static	float y;
static	float theta;
static	float v;
static	float w;

 float inc_2_rad(){
	return ((2*M_PI)/(2048*4));
}

 float inc_2_mm(){
	return (obim_tocka/(2048*4));
}

void odometrija_init(){
	x=0;
	y=0;
	theta=0;
	v=0;
	w=0;
}

void odometrija(int dt){

	/*float Vd_enc= enc1_GetInc(); // impulsa u ms
	float Vl_enc= enc2_GetInc();

	float Vd = Vd_enc * inc_2_mm();  // mm u ms
	float Vl = Vl_enc * inc_2_mm();


	  w = (Vd - Vl) / razmak_tockova;
	  v = (Vd + Vl) / 2;


	  theta += w;
	  x+= v*(cos(theta));
	  y+= v*(sin(theta));*/

	    dt_sec = dt / 1000.;

		 enc_r = enc1_GetInc();
		 enc_l = enc2_GetInc();



		// INC * RAD/INC * 1/s * m
		w_r = enc_r * inc_2_rad() / dt_sec;
		w_l = enc_l * inc_2_rad() / dt_sec;

		v_r = w_r * (d_tocka/2);
		v_l = w_l * (d_tocka/2);

		 v = (v_r + v_l)/2;
		 w = (v_r - v_l)/razmak_tockova;

		 theta = normalize_angle(theta+ w * dt_sec);
		x +=v* cos(theta + (w*dt_sec/2)) * dt_sec;
		y += v * sin(theta + (w*dt_sec/2)) * dt_sec;


}

float get_x (){
	return x;
}

float get_y (){
	return y;
}

float get_theta (){
	return theta;
}

void set_theta_smer (){
	 theta=+M_PI;
}

float get_w (){
	return w;
}

float get_v (){
	return v;
}
