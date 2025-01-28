/*
 * kretanje.c
 *
 *  Created on: 21. mar 2024.
 *      Author: HP
 */
#include "biblioteka/kretanje.h"
#include "biblioteka/odometrija.h"
#include "biblioteka/tajmer.h"
#include "biblioteka/pid.h"

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"


void napred(float x,float y,float theta,float v_cr, float w_cr, _Bool smer ){
	pid_init();
	if(smer == 1){
		set_theta_smer();
		set_dist(-1);
		set_or(1);

	}
	else{
		set_or(0);
		set_dist(1);
	}
	set_det(smer);

		//set_dist(1);
	while(get_faza()<3){
	  set_x(x);
	  set_y(y);
	  set_theta(theta);
	  set_v_cr(v_cr);
	  set_w_cr(w_cr);
	}
}


void rotacija(float theta,float v_cr, float w_cr){
	pid_init();
	while(get_faza()<3){
	  set_x(0);
	  set_y(0);
	  set_theta(theta);
	  set_v_cr(v_cr);
	  set_w_cr(w_cr);
	  }
}
