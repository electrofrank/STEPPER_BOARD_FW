/*
 * ISD.c
 *
 *  Created on: 17 Feb 2021
 *      Author: Team DIANA
 */

#include "ISD.h"
#include "math.h"
#include "stdlib.h"

#define T_CLOCK 	64000000 //apb1 timer clock
#define RES			1600
#define MAX_SPEED	3600
#define INITIAL_PERIOD 4

void makeNSteps(StepperMotor *motor, int steps, GPIO_PinState direction) {
	//exit if:
	// 1. Speed is zero.
	// 2. Zero number of steps.
	// 3. Direction is set wrong.
	if (steps == 0 || motor->speed == 0 || direction < 0 || direction > 1)
		return;

	//if steps is negative, change direction and continue
	if (steps < 0) {
		steps = steps * (-1);
		if (direction == 1) {
			direction = 0;
		} else {
			direction = 1;
		}
	}

	// set given direction
	HAL_GPIO_WritePin(motor->dir_Port, motor->dir_Pin, direction);

	while (steps > 256) {
		motor->htim->Instance->RCR = 255;			// set number of PWM signals
		motor->htim->Instance->EGR = TIM_EGR_UG;		// update timer settings
		TIM_CHANNEL_STATE_SET_ALL(motor->htim, HAL_TIM_CHANNEL_STATE_READY);// set channel state to ready
		motor->htim->Instance->SR &= 0xFFFE;// clear UIF - update interrupt flag
		HAL_TIM_PWM_Start(motor->htim, motor->stepChannel);	// generate PWM signals
		steps -= 256;

		while ((motor->htim->Instance->SR & 0x01) == 0) {
			//wait until 256 pulses are generated.
		}
	}

	motor->htim->Instance->RCR = steps - 1;			// set number of PWM signals
	motor->htim->Instance->EGR = TIM_EGR_UG;			// update timer settings
	TIM_CHANNEL_STATE_SET_ALL(motor->htim, HAL_TIM_CHANNEL_STATE_READY);// set channel state to ready
	motor->htim->Instance->SR &= 0xFFFE;	// clear UIF - update interrupt flag
	HAL_TIM_PWM_Start(motor->htim, motor->stepChannel);	// generate PWM signals
	while ((motor->htim->Instance->SR & 0x01) == 0) {
		//wait until pulses are generated.
	}

}

float rotate(StepperMotor *motor, float rotationAngle, GPIO_PinState direction) {
	float stepAngle = (float) 360 / RES;		// rotation angle of each step

	int steps = round(rotationAngle * 30 * 2 / stepAngle);// whole number of steps for given rotation angle

	//if rotation angle is less than one step angle, return rotationAngle
	if (steps == 0 || motor->speed == 0) {
		return rotationAngle;
	} else {
		//else make calculated whole number of steps
		makeNSteps(motor, steps, direction);
	}
	//return remaminder of the rotation angle
	return rotationAngle - steps * stepAngle;
}

void setSpeed(StepperMotor *motor, float speed) {
	float pulseFreq;								// frequency of single pulse
	int period = 0;
	int tmpPeriod = INITIAL_PERIOD;					// temporary period
	int prescalar;
	int tmpPrescalar;								// temporary prescalar 
	float tmpDeviation;								// temporary deviation
	float deviation = speed;
	uint8_t flag = 1;

	// exit if speed is negative
	if (speed < 0)
		return;
	//if speed is zero, assign speed and exit
	if (speed == 0) {
		motor->speed = 0;
		return;
	}
	// if speed is greater than MAX_SPEED, decrease speed value to MAX_SPEED
	if (speed > MAX_SPEED) {
		speed = MAX_SPEED;
	}
	//frequency of pwm signal
	pulseFreq = (float) speed * RES / 60;

	//searching suitable value of period and prescalar with less deviation.
	while (tmpPeriod < 65535 && flag) {
		tmpPrescalar = round(T_CLOCK / (pulseFreq * tmpPeriod));//calculating temporary Prescalar
		tmpDeviation =
				abs(
						(int) (pulseFreq
								- (float) T_CLOCK / (tmpPeriod * tmpPrescalar)));//calculating deviation from desired speed

		//if calculated deviation is less than previous deviation, save prescalar and period
		if (tmpDeviation < deviation && tmpPrescalar < 65535) {
			deviation = tmpDeviation;
			prescalar = tmpPrescalar;
			period = tmpPeriod;
		}

		//exit from while loop if speed value deviation is zero.
		if (deviation >= 0.0001) {
			tmpPeriod++;
		} else {
			flag = 0;
		}
	}

	//set calculated timer settings
	motor->htim->Instance->PSC = prescalar - 1;
	motor->htim->Instance->ARR = period - 1;
	motor->htim->Instance->CCR1 = period / 2;
	motor->speed = speed;
}

void stopMoving(StepperMotor *motor) {
	//stop motor
	HAL_TIM_PWM_Stop(motor->htim, motor->stepChannel);
}

