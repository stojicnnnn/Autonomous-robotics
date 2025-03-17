/*
 * pid.c
 *
 *  Created on: Mar 2, 2024
 *      Author: HP
 */

#include "biblioteka/odometrija.h"
#include "biblioteka/tajmer.h"
#include "biblioteka/motor.h"
#include "biblioteka/pwm.h"


#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include <math.h>

//pid

static float e=0;
static float ei=0;
static float ed=0;
static float e_pre=0;
static float greska;
static float greska1;
static float fleg;
static int brojac=0;
static int smer=1;//bilo =1
static int or=0;
static float u=0;
static int izlazak=0;
static int test1=0;


static float const EI_MAX = 1000;
static float const EI_MIN = -1000;
static float const U_MIN = -2100; //bilo 2100 isto dole
static float const U_MAX = 2100;

//pozicija

static int faza=0;
static float w;
static float v;
static float v_pid;
static float w_pid;
volatile float v_cr;
volatile float w_cr;
static float dist;
static float phi;
static float phi_error;
static float phi_prim_error;
static float eps_dist;
static float eps_theta;
static float Kp_w;
static float Kp_d;
static float uslov;

//brzina

volatile float v_dref;
volatile float v_lref;
static float u_d;
static float u_l;
volatile float vel_l;
volatile float vel_d;


static float const v_max = 2100;
static float const v_min = -2100;
static float const w_max = 6;
static float const w_min = -6;
static float const vel_max = 2100;
static float const vel_min = -2100;

void pid_init ()
{
	e=0;
	ei=0;
	ed=0;
	e_pre=0;
	u=0;
	faza=0;
	v_dref=0;
	v_lref=0;
	w_cr=0;
	v_cr=0;
	brojac=0;
	izlazak=0;

}

float  normalize_angle(float angle){
	if (angle > M_PI){
        angle = angle - M_PI*2;
    }
	else if (angle < - M_PI){
        angle +=M_PI*2 ;
    }

    return angle;
}

float normalize_angle_nikola(float zeljena,float trenutna){
	float angle;
	if ((zeljena - trenutna)>M_PI)
		angle = zeljena - 2*M_PI;
	else
		angle=zeljena;
    return angle;
}


float pid_brzina (float Kp,float Ki, float Kd,float ref_v,float mes_v)
{


	e=  ref_v - mes_v;
	ei += e;
	ed = e - e_pre;

	 if (ei > EI_MAX)
	    {
	      ei = EI_MAX;
	    }
	  else if (ei < EI_MIN)
	    {
	      ei = EI_MIN;
	    }

	  u = Kp * e + Ki * ei + Kd * ed;

	  //smer
	   if (u > U_MAX)
	   {
	       u = U_MAX;
	   }
	   else if (u < U_MIN)
	   {
	  	   u = U_MIN;
	   }


			 return u;
	   e_pre = e;

}

void racunanje_brzine(float v, float w){
	  if (v > v_max){
		       v = v_max;
		   }
	  else if (v < v_min){
		  	   v = v_min;
		   }
	  if (w > w_max){
		       w = w_max;
		   }
	  else if (w < w_min){
		  	   w = w_min;
		   }

	  vel_d= v + w*(razmak_tockova/2);
	  vel_l= v - w*(razmak_tockova/2);
	}





void regulacija_pozicije(float x_ref, float y_ref, float theta_ref, float x, float y, float theta){
//dodati mozda i regulator zbog prepucavanja ugla pri brzoj rotaciji

		v_pid=0;
		w_pid=0;
		theta=theta+(M_PI*or);
		dist =smer* sqrt(pow((x_ref-x),2)+pow((y_ref-y),2));
		phi = atan2(y_ref - y, x_ref - x);
		phi_error = normalize_angle(phi - theta ); //promjeni ovo
		phi_prim_error = normalize_angle(  theta_ref - theta );

	eps_dist = 6;
	eps_theta = (4*M_PI)/180;


	Kp_w =9; //6 bilo 3.5 bilo
	Kp_d = 2.8;
	greska=(phi_prim_error*180/M_PI);

	if(faza == 0){
test1=1;
		v_pid=0;
		if(w_cr<=2)
			Kp_w=14;
		w_pid=Kp_w * phi_error;

		greska1=(phi_error*180/M_PI);
		uslov = get_w ();
		//if((fabs(phi_prim_error) <= eps_theta))
		//	faza=1;
		if((fabs(phi_error) <= eps_theta) & (fabs(uslov) <= 1)){
			//0.001 greska bila
			faza=1;
			test1=2;

		}
	}
	else if (faza ==1){
	if(v_cr > 400){
			Kp_d=2.3;
		}
	if(v_cr > 800){
				Kp_d=1.7;
			}
	if(v_cr > 1190){
					Kp_d=1.69;
				}
		w_pid = Kp_w * phi_error;
		v_pid = Kp_d * dist;
	/*	fleg=dist;
		if((fleg<50) & (fleg>-50)){
			izlazak++;
			Kp_w=0;
			w_pid = Kp_w * phi_error;
			if(izlazak>=80){
			faza=2;
			}
		}
		*/


		greska1=(phi_error*180/M_PI);
		if ((fabs(dist) <= eps_dist) & (fabs(get_v()) <= 10))
			faza =2;
		//else if( (fabs(distx) <= eps_distx) & (fabs(get_v()) <= 10))
			//faza= 2;
		//else if( (fabs(disty) <= eps_disty) & (fabs(get_v()) <= 10))
				//	faza= 2;

	}
	else if (faza ==2){
		v_pid=0;
		if(w_cr<=2)
					Kp_w=11;
		w_pid=Kp_w * phi_prim_error;


		uslov = get_w ();
		if((fabs(phi_prim_error) <= eps_theta) & (fabs(uslov) <= 1)){
			faza=3;

		}
	}



	if( (v_pid < v_cr) & (v_pid > -v_cr))
		v = v_pid;
	else if (v_pid>v_cr)
		v=v_cr;
	else if(v_pid<-v_cr)
		v = -v_cr;



	if( (w_pid < w_cr) & (w_pid > -w_cr))
		w = w_pid;
	else if (w_pid>w_cr)
		w=w_cr;
	else if(w_pid<-w_cr)
		w = -w_cr;

	racunanje_brzine(v,w);

}



void regulacija_brzine(){


//saturacija trazene
	 if (vel_d > vel_max){
		 vel_d = vel_max;
	 }
     else if (vel_d < vel_min){
    	 vel_d = vel_min;
     }

	 if (vel_l > vel_max){
			 vel_l = vel_max;
		 }
	 else if (vel_l < vel_min){
	    	 vel_l = vel_min;
	     }

// ubrzanje
	//

	 if(v_dref < vel_d)
		 v_dref +=0.7;
	 else if(v_dref > vel_d)
		//v_dref = vel_d - 0.6;
		 v_dref -= 0.7;
	else
		 v_dref=vel_d;

	 if(v_lref < vel_l)
		v_lref += 0.7;
	 else if(v_lref > vel_l)
		// v_lref=vel_l - 0.6;
		 v_lref -= 0.7;
	 else
		 v_lref=vel_l;




	 u_l=pid_brzina(4, 0.105, 0, v_lref, v_l); //kp 4.3 bilo
	 u_d=pid_brzina(4, 0.105, 0, v_dref, v_r);

	 motor1_set_PWM((int16_t)u_d); 
	 motor2_set_PWM((int16_t)u_l);
}


int get_faza(){
	return faza;
}

void set_faza(){
	faza=0;
}

void set_v_cr(float novo){
	v_cr=novo;
}
void set_w_cr(float novo){
	w_cr=novo;
}

void set_dist(int a){
	smer=a;
}
void set_or(novo){
	or=novo;
}
