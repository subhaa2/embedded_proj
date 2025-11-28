#ifndef SPIDER_MAIN_H
#define SPIDER_MAIN_H

#include "pico/stdlib.h"
#include <stdio.h>


// Delcare variables
extern float servo_offsets[16];

// PCA9685 / servo functions
int pca9685_write8(uint8_t reg, uint8_t data);
bool pca9685_init(void);
void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);
float servo_check_invert(uint8_t channel, float angle);
void servo_offset_int(void);
void set_servo_angle(uint8_t channel, float angle);


// Spider leg states
void servo_leg_state_standby(void);
void servo_leg_state_move_forward(void);
void servo_leg_state_move_backward(void);
void servo_leg_state_move_left(void);
void servo_leg_state_move_right(void);
void servo_leg_state_rotate_left(void);
void servo_leg_state_rotate_right(void);
void servo_leg_state(int state, int ms_time);

// Initialize
void initialize_spider(void);

#endif