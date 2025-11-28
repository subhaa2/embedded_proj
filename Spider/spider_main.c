#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include "spider_main.h"

#define I2C_PORT i2c1
#define SDA_PIN 6
#define SCL_PIN 7
#define PCA9685_ADDR 0x40

#define SPIDER_90DEGREE -1
#define SPIDER_STANDBY 0
#define SPIDER_MOVE_FORWARD 1
#define SPIDER_MOVE_BACKWARD 2
#define SPIDER_MOVE_LEFT 3
#define SPIDER_MOVE_RIGHT 4
#define SPIDER_ROTATE_LEFT 5
#define SPIDER_ROTATE_RIGHT 6

#define DEFAULT_WAIT_TIME 1000
#define DEFAULT_WARM_UP_SPEED 5

#pragma region 
#define SERVO_LEFT_FRONT_LEG 0 // SERVO 5 (INVERTED)
#define SERVO_LEFT_FRONT_KNEE 1 // SERVO 6 (INVERTED)

#define SERVO_RIGHT_FRONT_LEG 3 // SERVO 8
#define SERVO_RIGHT_FRONT_KNEE 2 // SERVO 7

#define SERVO_LEFT_BACK_LEG 15 // SERVO 4 (INVERTED)
#define SERVO_LEFT_BACK_KNEE 14 // SERVO 3

#define SERVO_RIGHT_BACK_LEG 12 // SERVO 1 
#define SERVO_RIGHT_BACK_KNEE 13 // SERVO 2
#pragma endregion



float servo_offsets[16] = {0};

int pca9685_write8(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_write_blocking(I2C_PORT, PCA9685_ADDR, buf, 2, false);
}

bool pca9685_init() {
    int ret;

    // Reset PCA9685
    ret = pca9685_write8(0x00, 0x00); // Write 0x00 to MODE1 register: set to normal mode (all bits cleared)
    if (ret < 0) {printf("Failed to write MODE1 reset\n"); return false; }
    sleep_ms(10);

    // Set PWM frequency to 50 Hz
    float freq = 50.0f;

    // calculate the prescale value based on formula: prescale = round(osc_clock) / 4096 * update_rate) - 1
    // where osc_clock = 25 MHz, update_rate = desired PWM frequency
    uint8_t prescale = (uint8_t)(25000000.0 / (4096 * freq) - 1);
    uint8_t oldmode;
    

    // read current  MODE1 register value
    uint8_t reg = 0x00;
    ret = i2c_write_blocking(I2C_PORT, PCA9685_ADDR, &reg, 1, false); // Select MODE1 register (send address)
    if (ret < 0) {printf("Failed to select MODE1 register\n"); return false; }
    ret = i2c_read_blocking(I2C_PORT, PCA9685_ADDR, &oldmode, 1, false); // Read byte 1 from MODE1 register
    if (ret < 0) {printf("Failed to read MODE1 register\n"); return false; }

    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Set the sleep bit to enable prescale change
    ret = pca9685_write8(0x00, newmode); // Put device to sleep first
    if (ret < 0) {printf("Failed to put device to sleep\n"); return false; }
    
    ret = pca9685_write8(0xFE, prescale); // Write prescale value
    if (ret < 0) {printf("Failed to set prescale\n"); return false; }

    ret = pca9685_write8(0x00, oldmode); // Wake up the device
    if (ret < 0) {printf("Failed to wake device\n"); return false; }
    
    sleep_ms(5);

    ret = pca9685_write8(0x00, oldmode | 0xA1); // Auto-increment + restart device    
    if (ret < 0) {printf("Failed to restart device\n"); return false; }

    return true; // All is okay
}

void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[5];

    // The PCA9685 has 4 registers per channel starting from 0x06:
    // LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H
    // So to reach channel 'n', we do 0x06 + 4 * n
    data[0] = 0x06 + 4 * channel;
    
    // Split the 16-bit ON value into low and high bytes
    data[1] = on & 0xFF; // LEDn_ON_L
    data[2] = on >> 8; // LEDn_ON_H

    // Split the 16-bit OFF value into low and high bytes
    data[3] = off & 0xFF; // LEDn_ON_L
    data[4] = off >> 8; // LEDn_ON_H

    // Write all 5 bytes (register address + 4 data bytes) over I2C
    // The PCA9685 auto-increments register addresses during multi-byte writes
    i2c_write_blocking(I2C_PORT, PCA9685_ADDR, data, 5, false);
}

// Check through if servo is inverted and update the angle to be the inverted value
float servo_check_invert(uint8_t channel, float angle){
    float adjustedAngle = 0;

    switch (channel)
    {
    case SERVO_LEFT_FRONT_LEG:
    case SERVO_LEFT_FRONT_KNEE:
    case SERVO_RIGHT_BACK_LEG:
    case SERVO_LEFT_BACK_KNEE:
        adjustedAngle = 180 - angle;
        break;
    
    default:
        adjustedAngle = angle;
        break;
    }

    return adjustedAngle;
}

// Set offset for any of the servo channels
void servo_offset_init(){
   // servo_offsets[3] = 10;
}

void set_servo_angle(uint8_t channel, float angle) {
    // Check for invert. And return the adjusted angle value
    float adjustedAngle = (float)servo_check_invert(channel,angle);

    // Offset values
    float correctAngle =  adjustedAngle + servo_offsets[channel];

    // Clamp to 0-180° to avoid invalid servo angles
    if (correctAngle < 0) correctAngle = 0;
    if (correctAngle > 180) correctAngle = 180;

    // Define the tick range corresponding to servo pulse width
    // min_tick = pulse width for 0°  (around 0.5 ms)
    // tick_range = difference between 0° and 180° pulse widths
    // typical range: 130 (≈0.5ms) to 480 (≈2.4ms)
    const uint16_t min_tick = 130;
    const uint16_t tick_range = 350;

    // Map the angle (0–180°) to a tick value (130–480)
    uint16_t pulse = (uint16_t)(min_tick + (correctAngle / 180.0f) * tick_range);

    // Send calculated pulse width to PCA9685 for the selected servo channel to set the rotation angle
    pca9685_set_pwm(channel, 0, pulse);
}

void servo_leg_state_standby(){
        set_servo_angle(SERVO_LEFT_FRONT_LEG, 20);
        set_servo_angle(SERVO_RIGHT_FRONT_LEG, 20);
        set_servo_angle(SERVO_LEFT_BACK_LEG, 20);
        set_servo_angle(SERVO_RIGHT_BACK_LEG,20);
}

void servo_leg_state_move_forward(){
    // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,135);

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,135);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
    // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,135);

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,135);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);

    // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,135);

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,135);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
    // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,135);

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,135);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
}

void servo_leg_state_move_backward(){
    // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 45);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    set_servo_angle(SERVO_RIGHT_BACK_KNEE,45); // New 

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_KNEE,45); // New

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);

        // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 45);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    set_servo_angle(SERVO_RIGHT_BACK_KNEE,45); // New 

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_KNEE,45); // New

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);

            // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 45);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    set_servo_angle(SERVO_RIGHT_BACK_KNEE,45); // New 

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_KNEE,45); // New

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);

        // New Cycle

    // Lifts and moves the Left-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 45);

    sleep_ms(500);

    // Sets the Left-Back-Leg back on the ground (leg still forward)
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);
    
    sleep_ms(500);

    // Lifts the Left-Front-Leg & Right-Back-Leg forward
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    set_servo_angle(SERVO_RIGHT_BACK_KNEE,45); // New 

    sleep_ms(500);

    // Moves the Left-Back-Knee to neutral
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);

    // Moves Both Front and Back Knees on the Right to parallel
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);

    sleep_ms(500);

    // Sets the Left-Front-Leg & Right-Back-Leg on the ground
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);

    sleep_ms(500);

    // Lifts the Left-back-Leg & Right-Front-Leg, and moves Right-Front-Knee forward
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 90);

    sleep_ms(500);

    // Moves Left-Front-Knee backwards (parallel), and Right-Back-Knee backwards (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(500);
    
    // Sets the Right-Front-Leg on the ground
    // Sets the Left-Back-Leg on the ground
    set_servo_angle(SERVO_LEFT_BACK_KNEE,45); // New

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(500);
    
    // Moves the Left-Front-Knee front (neutral)
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(500);
}

void servo_leg_state_move_left(){
    // New Cycle
    set_servo_angle(SERVO_LEFT_FRONT_LEG,80);
    set_servo_angle(SERVO_LEFT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,160);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_LEFT_FRONT_LEG,80);
    set_servo_angle(SERVO_LEFT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,160);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_LEFT_FRONT_LEG,80);
    set_servo_angle(SERVO_LEFT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,160);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_LEFT_FRONT_LEG,80);
    set_servo_angle(SERVO_LEFT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,160);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,45);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);
}

void servo_leg_state_move_right(){
    // New Cycle
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,80);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,160);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,80);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,160);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,80);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,160);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,80);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,80);

    sleep_ms(1000);

    set_servo_angle(SERVO_LEFT_FRONT_LEG,10);
    set_servo_angle(SERVO_LEFT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,160);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,20);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,45);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,135);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,10);

    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);

    sleep_ms(1000);

    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);

    sleep_ms(1000);
}

void servo_leg_state_rotate_left(){
    // New Cycle
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,0);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,180);

    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);

    sleep_ms(1000);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    
    set_servo_angle(SERVO_RIGHT_BACK_KNEE, 180);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,0);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,180);

    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);

    sleep_ms(1000);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    
    set_servo_angle(SERVO_RIGHT_BACK_KNEE, 180);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);

    // New Cycle
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,0);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,180);

    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);

    sleep_ms(1000);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    
    set_servo_angle(SERVO_RIGHT_BACK_KNEE, 180);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,0);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,180);

    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);

    sleep_ms(1000);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    
    set_servo_angle(SERVO_RIGHT_BACK_KNEE, 180);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    // New Cycle
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,0);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,180);

    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);

    sleep_ms(1000);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    
    set_servo_angle(SERVO_RIGHT_BACK_KNEE, 180);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
        // New Cycle
    set_servo_angle(SERVO_LEFT_BACK_LEG,90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,0);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,180);

    set_servo_angle(SERVO_LEFT_BACK_LEG,10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG,10);

    sleep_ms(1000);
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    
    set_servo_angle(SERVO_RIGHT_BACK_KNEE, 180);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);

}

void servo_leg_state_rotate_right(){
     // New Cycle
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,0);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,180);
    
    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(1000);
    set_servo_angle(SERVO_LEFT_BACK_LEG, 90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 180);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,0);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,180);
    
    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(1000);
    set_servo_angle(SERVO_LEFT_BACK_LEG, 90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 180);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,0);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,180);
    
    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(1000);
    set_servo_angle(SERVO_LEFT_BACK_LEG, 90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 180);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,0);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,180);
    
    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(1000);
    set_servo_angle(SERVO_LEFT_BACK_LEG, 90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 180);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,0);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,180);
    
    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(1000);
    set_servo_angle(SERVO_LEFT_BACK_LEG, 90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 180);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);

    sleep_ms(1000);
    
    // New Cycle
    set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
    set_servo_angle(SERVO_LEFT_FRONT_LEG,90);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,0);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,180);
    
    set_servo_angle(SERVO_RIGHT_BACK_LEG, 10);
    set_servo_angle(SERVO_LEFT_FRONT_LEG, 10);

    sleep_ms(1000);
    set_servo_angle(SERVO_LEFT_BACK_LEG, 90);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 90);

    sleep_ms(100);
    set_servo_angle(SERVO_LEFT_BACK_KNEE, 180);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE, 0);

    set_servo_angle(SERVO_LEFT_BACK_LEG, 10);
    set_servo_angle(SERVO_RIGHT_FRONT_LEG, 10);

    sleep_ms(100);
    set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
    set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
    set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
    set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
}

void servo_leg_state(int state, int ms_time){
    switch (state)
    {
    case SPIDER_STANDBY:
        servo_leg_state_standby();
        break;
        
    case SPIDER_MOVE_FORWARD:
        servo_leg_state_move_forward();
        break;
    
    case SPIDER_MOVE_BACKWARD:
        servo_leg_state_move_backward();
        break;

    case SPIDER_MOVE_LEFT:
        servo_leg_state_move_left();
        break;

    case SPIDER_MOVE_RIGHT:
        servo_leg_state_move_right();
        break;

    case SPIDER_ROTATE_LEFT:
        servo_leg_state_rotate_left();
        break;

    case SPIDER_ROTATE_RIGHT:
        servo_leg_state_rotate_right();
        break;

    case SPIDER_90DEGREE:
    default:
        // Set all servos to Middle
        set_servo_angle(SERVO_LEFT_FRONT_LEG,90);
        set_servo_angle(SERVO_LEFT_FRONT_KNEE,90);
        set_servo_angle(SERVO_RIGHT_FRONT_LEG,90);
        set_servo_angle(SERVO_RIGHT_FRONT_KNEE,90);
        set_servo_angle(SERVO_LEFT_BACK_LEG,90);
        set_servo_angle(SERVO_LEFT_BACK_KNEE,90);
        set_servo_angle(SERVO_RIGHT_BACK_LEG,90);
        set_servo_angle(SERVO_RIGHT_BACK_KNEE,90);
        break;
    }

    sleep_ms(ms_time);
}

void servo_leg_warmup(int speed){
    if (speed <= 0) // Reject negative or 0 value
    {
        printf("Warmup speed value rejected. Please insert a non-negative and non zero number,");
        return;
    }    

    printf("Testing Servo Rotate Back and Forth\n");
    for (float a = 90; a <= 180; a += speed) { 
        set_servo_angle(SERVO_LEFT_BACK_LEG, a);  // Servo 4
        set_servo_angle(SERVO_LEFT_BACK_KNEE, a);  // Servo 3
        set_servo_angle(SERVO_RIGHT_BACK_KNEE, a);  // Servo 2
        set_servo_angle(SERVO_RIGHT_BACK_LEG, a);  // Servo 1
        set_servo_angle(SERVO_RIGHT_FRONT_LEG, a); // Sevro 8
        set_servo_angle(SERVO_RIGHT_FRONT_KNEE, a); // Servo 7
        set_servo_angle(SERVO_LEFT_FRONT_KNEE, a); // Servo 6
        set_servo_angle(SERVO_LEFT_FRONT_LEG, a); // Servo 5
        sleep_ms(50); 
    }

    sleep_ms(500);

    for (float a = 180; a >= 0; a -= speed) { 
        set_servo_angle(SERVO_LEFT_BACK_LEG, a);  // Servo 4
        set_servo_angle(SERVO_LEFT_BACK_KNEE, a);  // Servo 3
        set_servo_angle(SERVO_RIGHT_BACK_KNEE, a);  // Servo 2
        set_servo_angle(SERVO_RIGHT_BACK_LEG, a);  // Servo 1
        set_servo_angle(SERVO_RIGHT_FRONT_LEG, a); // Sevro 8
        set_servo_angle(SERVO_RIGHT_FRONT_KNEE, a); // Servo 7
        set_servo_angle(SERVO_LEFT_FRONT_KNEE, a); // Servo 6
        set_servo_angle(SERVO_LEFT_FRONT_LEG, a); // Servo 5
        sleep_ms(50); 
    } 

    sleep_ms(500);

    printf("Returning all servos to neutral position...\n");
    for (float a = 0; a <= 90; a += speed) {
        set_servo_angle(SERVO_LEFT_BACK_LEG, a);
        set_servo_angle(SERVO_LEFT_BACK_KNEE, a);
        set_servo_angle(SERVO_RIGHT_BACK_KNEE, a);
        set_servo_angle(SERVO_RIGHT_BACK_LEG, a);
        set_servo_angle(SERVO_RIGHT_FRONT_LEG, a);
        set_servo_angle(SERVO_RIGHT_FRONT_KNEE, a);
        set_servo_angle(SERVO_LEFT_FRONT_KNEE, a);
        set_servo_angle(SERVO_LEFT_FRONT_LEG, a);
        sleep_ms(30);
    }
    sleep_ms(500);
}

void initialize_spider(){
    stdio_init_all();

    printf("STARTING SPIDER SERVOS\n");

    // Init I2C (Must run after Temperature Sensor init)
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz I2C
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("Scanning I2C bus...\n");
    
    uint8_t dummy = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        int ret = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (ret >= 0) {
            printf("I2C device found at address 0x%02X\n", addr);
        }
    }

    printf("Initializing PCA9685...\n");
    
    if(pca9685_init()) {
        printf("PCA9685 initialized OK\n");
    } else {
        printf("PCA9685 init failed!\n");
    }

    // set up servo offset if needed
    servo_offset_init();

    // Testing Servo Legs
    servo_leg_warmup(DEFAULT_WARM_UP_SPEED);

    // Set the Spider at Standby
    servo_leg_state(SPIDER_STANDBY, DEFAULT_WAIT_TIME);

    printf("INTIALIZED SPIDER SERVOS. SPIDER NOW ON STANDBY MODE\n");
}
