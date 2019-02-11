/*
 * USsensor.h
 *
 *  Created on: 2019. febr. 4.
 *      Author: Judit
 */
#include <stdint.h>

#ifndef USSENSOR_H_
#define USSENSOR_H_


typedef enum state {
    RISING,
    FALLING
}state_t;

typedef struct US_values{
    int overflows;
    uint32_t start;
    uint32_t end;
    uint32_t distance;
    state_t state;
    float err;
}US_values_t;

US_values_t sensors[3];

void init_us(US_values_t* sensors);

void US_calculate_distance(US_values_t* sensor, uint32_t period, uint32_t clk);

#endif /* USSENSOR_H_ */
