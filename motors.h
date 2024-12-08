#pragma once
#include "nrf.h"
#include "stdbool.h"
#include <nrfx_pwm.h>
#include "nrf_gpio.h"
#include "microbit_v2.h"





void encoder_init(void);

void pwm_init(void);



void set_motor_speed(uint8_t motor, uint8_t duty_cycle);


void change_motor_direction(uint8_t motor, bool forward);


void accelerate_motors(uint8_t start_duty, uint8_t end_duty, uint8_t step, uint16_t delay_ms);



void decelerate_motors(uint8_t start_duty, uint8_t end_duty, uint8_t step, uint16_t delay_ms);


void read_encoder_values(void);

void calculate_rpm(void);




