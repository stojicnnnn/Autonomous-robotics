/*
 * pid.h
 *
 *  Created on: Mar 2, 2024
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_PID_H_
#define INC_BIBLIOTEKA_PID_H_


int get_faza();
void set_faza();
void pid_init ();
float pid_brzina (float Kp,float Ki, float Kd,float ref_v,float mes_v);
void regulacija_pozicije(float x_ref, float y_ref, float theta_ref, float x, float y, float theta);
void racunanje_brzine(float v, float w);
void regulacija_brzine();
float normalize_angle(float angle);
void set_v_cr(float novo);
void set_w_cr(float novo);
float normalize_angle_nikola(float zeljena,float trenutna);
void set_dist(int a);
void set_or();
void nazad(float x_zad,float pwm);

volatile float vel_l;
volatile float vel_d;

#endif /* INC_BIBLIOTEKA_PID_H_ */
