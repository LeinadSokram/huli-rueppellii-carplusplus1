/*
 * servo_control.h
 *
 *  Created on: 2019. febr. 6.
 *      Author: Dávid
 */

#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_
#include "stm32f7xx_hal.h"

typedef enum direction {
    LEFT, RIGHT
} direction_t;

void turn(direction_t direction, int rate, TIM_HandleTypeDef HTIM);

void straight(TIM_HandleTypeDef HTIM);

#endif /* SERVO_CONTROL_H_ */
