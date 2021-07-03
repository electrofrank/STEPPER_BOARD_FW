/*
 * ISD.h
 *
 *  Created on: 17 Feb 2021
 *      Author: Team DIANA
 */

#ifndef INC_ISD_H_
#define INC_ISD_H_

#include "math.h"

#define 	STM32F4					// Comment if H7 is used
//#define	STM32H7					// Comment if F4 is used

#ifdef	STM32F4
	#include "stm32f4xx.h"
	#include "stm32f4xx_hal.h"
	#include "stm32f4xx_hal_tim.h"
#elif 	STM32H7
	#include "stm32h7xx.h"
	#include "stm32h7xx_hal.h"
	#include "stm32h7xx_hal_tim.h"
#endif

/**
  * @brief  Motor structure to control stepper motor
  */
typedef struct
{

	uint16_t 			dir_Pin;
	GPIO_TypeDef* 		dir_Port;
	uint32_t 			stepChannel;
	TIM_HandleTypeDef 	*htim;
	float 	  			speed;

}StepperMotor;

/**
  * @brief  Makes/Executes given number of steps in a given direction (with predefined speed)
  * @param	motor structure to be controlled
  * @param	number of steps
  * @param	direction of the rotation
  * @retval none
  */

void makeNSteps(StepperMotor *motor, int steps, GPIO_PinState direction);

/**
  * @brief  Rotates given amount of angle in a given direction
  * @param	motor structure to be controlled
  * @param	rotation angle
  * @param 	direction of the rotation
  * @retval remainder of the rotation angle
  */
float rotate(StepperMotor *motor, float rotationAngle, GPIO_PinState direction);

/**
  * @brief  Sets speed of the motor
  * @param	motor structure to be controlled
  *	@param	desirable speed of the motor in RPM
  * @retval none
  */
void setSpeed(StepperMotor *motor, float speed);

/**
  * @brief  Stops the motor
  * @param	motor structure to be stopped
  * @retval none
  */
void stopMoving(StepperMotor *motor);


/**
  * @brief  Performs trapezoidal velocity profile
  * @param  motor Stepper motor to be controlled
  * @param	desired position
  * @param 	current position
  * @param 	max speed
  * @param 	acceleration
  * @retval none
  */
float trapezoidalVelocity(StepperMotor *motor, float desiredPosition, float currentPosition, float maxSpeed, float acceleration);


#endif /* INC_ISD_H_ */
