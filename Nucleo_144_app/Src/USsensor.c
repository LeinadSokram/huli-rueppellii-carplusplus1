/*
 * USsensor.c
 *
 *  Created on: 2019. febr. 4.
 *      Author: Judit
 */
#include "USsensor.h"

void init_us(US_values_t* sensors){
    for (int i = 0; i < 3; ++i) {
        sensors[i].state = RISING;
        sensors[i].start = 0;
        sensors[i].end = 0;
        sensors[i].overflows = 0;
        sensors[i].distance = 0;
    }
    sensors[0].err = 1.2;
    sensors[1].err = 1;
    sensors[2].err = 1;
}


void US_calculate_distance(US_values_t* sensor, uint32_t period, uint32_t clk){
    uint32_t steps = sensor->overflows * period + sensor->end - sensor->start;
  //  sensor->overflows = 0;
    float T = 1 /(float) clk * steps;
    sensor->distance= T * 340 / (float) 2  * 100 * sensor->err;
}
