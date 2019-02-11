/*
 * servo_control.c

 *
 *  Created on: 2019. febr. 6.
 *      Author: Dávid
 */
#include "servo_control.h"
#define straight_value 935

void turn(direction_t direction, int rate, TIM_HandleTypeDef HTIM) //rate must be between 1 and 10. HTIM = htim1
{
    if (rate > 10 || rate < 1) {
        return;
    }

    if (direction == LEFT) {
        HTIM.Instance->CCR1 = straight_value - rate;
    } else {
        HTIM.Instance->CCR1 = straight_value + rate;
    }
}

void straight(TIM_HandleTypeDef HTIM) {
    if (HTIM.Instance->CCR1 != straight_value) {
        HTIM.Instance->CCR1 = straight_value;
    }
}
