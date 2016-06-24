/*
 * lego_robot.c
 *
 * Author: In Hwan "Chris" Baek
 *         chris.inhwan.baek@gmail.com
 */
 
#include <stdio.h>
#include <mraa.h>
#include "lego_robot.h"

mraa_pwm_context pwm1, pwm2;
mraa_gpio_context A1, A2, B1, B2, standbyPin;

void forward_A();
void reverse_A();
void forward_B();
void reverse_B();

void initialize()
{
	pwm1 = mraa_pwm_init(20);
	mraa_pwm_period_us(pwm1, 870);
	mraa_pwm_enable(pwm1, 1);

	pwm2 = mraa_pwm_init(14);
	mraa_pwm_period_us(pwm2, 870);
	mraa_pwm_enable(pwm2, 1);

	mraa_pwm_write(pwm1, 0.01);
	mraa_pwm_write(pwm2, 0.01);
	mraa_pwm_write(pwm1, 0.0);
	mraa_pwm_write(pwm2, 0.0);

	A1 = mraa_gpio_init(48);
	A2 = mraa_gpio_init(36);
	mraa_gpio_dir(A1, MRAA_GPIO_OUT);
	mraa_gpio_mode(A1, MRAA_GPIO_STRONG);
	mraa_gpio_write(A1, 1);
	mraa_gpio_dir(A2, MRAA_GPIO_OUT);
	mraa_gpio_mode(A2, MRAA_GPIO_STRONG);
	mraa_gpio_write(A2, 1);

	B1 = mraa_gpio_init(33);
	B2 = mraa_gpio_init(46);
	mraa_gpio_dir(B1, MRAA_GPIO_OUT);
	mraa_gpio_mode(B1, MRAA_GPIO_STRONG);
	mraa_gpio_write(B1, 1);
	mraa_gpio_dir(B2, MRAA_GPIO_OUT);
	mraa_gpio_mode(B2, MRAA_GPIO_STRONG);
	mraa_gpio_write(B2, 1);

	standbyPin = mraa_gpio_init(47);
	mraa_gpio_dir(standbyPin, MRAA_GPIO_OUT);
	mraa_gpio_mode(standbyPin, MRAA_GPIO_STRONG);
	mraa_gpio_write(standbyPin, 1);
}

void straight (float speed)
{	
	if (speed >= 0 && speed <= 1.0f) {
		mraa_pwm_write(pwm1, speed);
		mraa_pwm_write(pwm2, speed);

		forward_A();
		forward_B();
	}
	
	else if (speed < 0 && speed >= -1.0f) {
		mraa_pwm_write(pwm1, -speed);
		mraa_pwm_write(pwm2, -speed);

		reverse_A();
		reverse_B();
	}

	else
		printf("Wrong speed value. Choose between -1.0 and 1.0\n");
}

void set_A(float speed)
{
    if (speed > 0 && speed <= 1.0f) {
		mraa_pwm_write(pwm2, speed);
		forward_A();
    }
	else if (speed < 0 && speed >= -1.0f) {
		mraa_pwm_write(pwm2, -speed);
		reverse_A();
    }
    else
    {
		mraa_pwm_write(pwm2, 0.);
        brake_A();
    }
}

void set_B(float speed)
{
    if (speed > 0 && speed <= 1.0f) {
		mraa_pwm_write(pwm1, speed);
		forward_B();
    }
	else if (speed < 0 && speed >= -1.0f) {
		mraa_pwm_write(pwm1, -speed);
		reverse_B();
    }
    else
    {
		mraa_pwm_write(pwm1, 0.);
        brake_B();
    }
}

void forward_A()
{
	mraa_gpio_write(A1, 0);
	mraa_gpio_write(A2, 1);
}

void reverse_A()
{
	mraa_gpio_write(A1, 1);
	mraa_gpio_write(A2, 0);
}

void forward_B()
{
	mraa_gpio_write(B1, 0);
	mraa_gpio_write(B2, 1);
}

void reverse_B()
{
	mraa_gpio_write(B1, 1);
	mraa_gpio_write(B2, 0);
}

void standby(int disable)
{
	if (disable == 1)
		mraa_gpio_write(standbyPin, 0);
	else
		mraa_gpio_write(standbyPin, 1);
}

void brake_A()
{
	mraa_gpio_write(A1, 1);
	mraa_gpio_write(A2, 1);
}

void brake_B()
{
	mraa_gpio_write(B1, 1);
	mraa_gpio_write(B2, 1);
}

void stop()
{
	brake_A();
	brake_B();
}


void set_speed(float speed)
{
	mraa_pwm_write(pwm1, speed);
	mraa_pwm_write(pwm2, speed);
}
