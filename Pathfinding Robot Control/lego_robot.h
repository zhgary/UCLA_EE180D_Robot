/*
 * lego_robot.h
 *
 * Author: In Hwan "Chris" Baek
 *         chris.inhwan.baek@gmail.com
 */
 
#ifndef __LEGO_ROBOT_H__
#define __LEGO_ROBOT_H__

#include <stdio.h>
#include <mraa.h>

mraa_pwm_context pwm1, pwm2;
mraa_gpio_context A1, A2, B1, B2, standbyPin;

void initialize();
void straight(float);
void set_A(float);
void set_B(float);
void standby(int);
void brake_A();
void brake_B();
void stop();
void set_speed(float);

#endif //LEGO_ROBOT_H