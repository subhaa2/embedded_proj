#include <stdio.h>
#include <math.h>
#include <stdlib.h> // For abs()
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/multicore.h"

// -----------------------------------------------------------------------------
// --- PINS & CONFIGURATION ---
// -----------------------------------------------------------------------------
// I2C (MLX90614 Temperature Sensor)
#define I2C_PORT i2c0
#define I2C_SDA_PIN 6 // Moved to GP6 to avoid UART conflict
#define I2C_SCL_PIN 7 // Moved to GP7
#define MLX90614_ADDRESS 0x5A
#define MLX90614_REGISTER_TA 0x06
#define MLX90614_REGISTER_TOBJ1 0x07

// Microphone ADC Configuration
#define ADC_PIN 26
#define ADC_CHANNEL 0

// LD2450 mmWave Radar UART Configuration
#define RADAR_UART_ID uart1
#define RADAR_TX_PIN 4
#define RADAR_RX_PIN 5
#define RADAR_BAUD_RATE 115200

// LoRa UART Configuration (UART0 is used for serial debug/output, use UART1 pins for Lora)
#define LORA_UART_ID uart0 // Assuming LoRa module connects to UART0 for PC interfacing
#define LORA_TX_PIN 0      // GP0 (LoRa TX)
#define LORA_RX_PIN 1      // GP1 (LoRa RX)
#define LORA_BAUD_RATE 9600 // LoRa modules are typically 9600

#define FRAME_HEADER1 0xFD
#define FRAME_HEADER2 0xFC

// -----------------------------------------------------------------------------
// --- DSP/FFT CONFIGURATION (Simulation for High-Freq Detection) ---
// -----------------------------------------------------------------------------
#define SAMPLE_COUNT 128            // Number of samples for analysis (power of 2 is best for FFT)
#define SAMPLE_RATE_HZ 8000         // Target sample rate (8kHz is sufficient for human voice/shrieks)
#define HIGH_FREQ_THRESHOLD 1500    // High energy needed in the "shriek" band
#define AMPLITUDE_THRESHOLD 100     // Minimum overall volume to consider the sound

// -----------------------------------------------------------------------------
// --- DATA STRUCTURES & GLOBALS ---
// -----------------------------------------------------------------------------
typedef struct {
    int16_t x;
    int16_t y;
    int16_t speed;
    float distance;
    float angle;
} Target;

// Global radar target storage
Target radar_targets[3];
int num_targets = 0;


// -----------------------------------------------------------------------------
// --- MLX90614 Temperature Functions ---
// -----------------------------------------------------------------------------
void mlx90614_init() {
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

uint16_t mlx90614_read_reg(uint8_t reg) {
    uint8_t buffer[3];
    int ret = i2c_write_blocking(I2C_PORT, MLX90614_ADDRESS, &reg, 1, true); 
    if (ret != 1) return 0xFFFF;
    ret = i2c_read_blocking(I2C_PORT, MLX90614_ADDRESS, buffer, 3, false);
    if (ret != 3) return 0xFFFF;

    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

float convert_to_celsius(uint16_t raw_temp) {
    if (raw_temp == 0xFFFF) return -255.0f;
    return ((float)raw_temp * 0.02f) - 273.15f;
}

// -----------------------------------------------------------------------------
// --- Microphone DSP Functions ---
// -----------------------------------------------------------------------------
void adc_mic_init() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_CHANNEL);
}

/**
 * @brief Collects samples and performs a simplified analysis to detect shrieks.
 * * NOTE: For a true FFT, a library (like CMSIS DSP) is required. This
 * function uses the Zero Crossing Rate (ZCR) and peak-to-peak amplitude
 * over a collected sample window to distinguish high-frequency sounds (shrieks) 
 * from general noise (claps/low voice).
 * * @return 1 if a high-frequency shriek is detected, 0 otherwise.
 */
int analyze_mic_spectrum() {
    uint16_t samples[SAMPLE_COUNT];
    uint64_t start_time = time_us_64();
    
    // 1. Collect Samples
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = adc_read();
        // Wait for the next sample time to achieve the target SAMPLE_RATE
        uint64_t target_us = start_time + (i + 1) * (1000000 / SAMPLE_RATE_HZ);
        while (time_us_64() < target_us) {
            tight_loop_contents();
        }
    }

    // 2. Perform Simplified Frequency Analysis (Zero Crossing Rate & Energy)
    uint16_t min_val = 4096;
    uint16_t max_val = 0;
    int zero_crossings = 0; // Crossing the center point (2048 for full 12-bit ADC)
    
    // The mic's quiescent point is usually around 0.5V, which is 1365 (4095/3.3*1.1).
    // We use the raw ADC value from the previous working code as the center point.
    const int CENTER_POINT = 45; // Based on your previous low-range ADC center

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Track min/max for overall amplitude
        if (samples[i] < min_val) min_val = samples[i];
        if (samples[i] > max_val) max_val = samples[i];

        // Zero-crossing detection (high frequency = high zero crossings)
        if (i > 0) {
            // Check if the signal crossed the center point
            if ((samples[i-1] > CENTER_POINT && samples[i] <= CENTER_POINT) ||
                (samples[i-1] < CENTER_POINT && samples[i] >= CENTER_POINT)) {
                zero_crossings++;
            }
        }
    }

    int peak_to_peak = max_val - min_val;
    
    // Normalize ZCR by total samples (ZCR will be roughly 0-128)
    int normalized_zcr = (zero_crossings * 100) / SAMPLE_COUNT;
    
    // Decision Logic: 
    // Is it loud enough AND is the frequency high enough?
    
    // ZCR is a proxy for frequency. A shriek should have a high ZCR and a decent amplitude.
    
    printf("[MIC-DSP] Pk-Pk: %d | ZCR: %d\n", peak_to_peak, normalized_zcr);
    
    // High-Frequency Condition: High ZCR AND sufficient volume (peak-to-peak)
    if (normalized_zcr > 40 && peak_to_peak > AMPLITUDE_THRESHOLD) {
        return 1; // High-Frequency event (Shriek) detected
    }
    
    return 0; // No high-frequency shriek detected
}

// -----------------------------------------------------------------------------
// --- Radar (HLK-LD2450) Functions (Using user's provided structure) ---
// -----------------------------------------------------------------------------
void radar_init() {
    uart_init(RADAR_UART_ID, RADAR_BAUD_RATE);
    gpio_set_function(RADAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RADAR_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(RADAR_UART_ID, false, false);
}

void parse_target(uint8_t *data, Target *t) {
    t->x = (int16_t)(data[0] | (data[1] << 8));
    t->y = (int16_t)(data[2] | (data[3] << 8));
    t->speed = (int16_t)(data[4] | (data[5] << 8));
    t->distance = sqrtf((float)t->x * t->x + (float)t->y * t->y);
    t->angle = atan2f((float)t->x, (float)t->y) * (180.0f / M_PI);
}

void read_and_process_radar() {
    static uint8_t buffer[64];
    static int index = 0;
    static bool in_frame = false;

    while (uart_is_readable(RADAR_UART_ID)) {
        uint8_t byte = uart_getc(RADAR_UART_ID);

        // Detect start of frame
        if (!in_frame) {
            if (index == 0 && byte == FRAME_HEADER1) {
                buffer[index++] = byte;
            } else if (index == 1 && byte == FRAME_HEADER2) {
                buffer[index++] = byte;
                in_frame = true;
            } else {
                index = 0;
            }
            continue;
        }

        // Store data
        buffer[index++] = byte;

        // Full frame (Example: 47 bytes typical for 3 targets + end bytes)
        if (index >= 47) { 
            in_frame = false;
            index = 0;

            // Update Global Targets
            num_targets = 0;
            parse_target(&buffer[8], &radar_targets[0]);
            parse_target(&buffer[16], &radar_targets[1]);
            parse_target(&buffer[24], &radar_targets[2]);

            // Simple check: if X or Y is non-zero, assume target is valid
            if (abs(radar_targets[0].x) > 10 || abs(radar_targets[0].y) > 10) num_targets = 1;
            if (abs(radar_targets[1].x) > 10 || abs(radar_targets[1].y) > 10) num_targets = 2;
            if (abs(radar_targets[2].x) > 10 || abs(radar_targets[2].y) > 10) num_targets = 3;
        }
    }
}


// -----------------------------------------------------------------------------
// --- LoRa Functions ---
// -----------------------------------------------------------------------------
void lora_init() {
    uart_init(LORA_UART_ID, LORA_BAUD_RATE);
    gpio_set_function(LORA_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LORA_RX_PIN, GPIO_FUNC_UART);
    printf("[LORA] Initialized on GP%d (TX) at %d baud.\n", LORA_TX_PIN, LORA_BAUD_RATE);
}

/**
 * @brief Sends the consolidated sensor data string via LoRa.
 */
void transmit_sensor_data(float temp_ambient, float temp_object, int mic_status) {
    
    // We only send the highest priority target's data (Target 0, if available)
    int16_t target_x = 0;
    int16_t target_y = 0;
    int16_t target_speed = 0;
    
    if (num_targets >= 1) {
        target_x = radar_targets[0].x;
        target_y = radar_targets[0].y;
        target_speed = radar_targets[0].speed;
    }

    // --- CONSOLIDATED DATA STRING FORMAT ---
    // T_Amb, T_Obj, Mic_Freq_Status, R_X, R_Y, R_Speed
    // Example: 27.50,24.10,1,1200,-500,150
    char transmit_buffer[128];
    int len = snprintf(transmit_buffer, sizeof(transmit_buffer),
                       "%.2f,%.2f,%d,%d,%d,%d\n",
                       temp_ambient, temp_object, mic_status, 
                       target_x, target_y, target_speed);

    // Send the string via LoRa UART
    uart_puts(LORA_UART_ID, transmit_buffer);
    
    // Also print to USB serial for debug
    printf("[SENT] %s", transmit_buffer);
}


// -----------------------------------------------------------------------------
// --- Initialization ---
// -----------------------------------------------------------------------------
void init_peripherals() {
    // NOTE: This uses UART0 for both standard I/O (stdio) and LoRa. 
    // This is typically okay if you connect the LoRa module to a USB-to-UART 
    // converter on the PC and listen there, but it means you can't read the 
    // debug messages simultaneously if you're using a single USB port on the Pico.
    // If you need debug output, consider using UART1 for LoRa.
    stdio_init_all();
    mlx90614_init();
    adc_mic_init();
    radar_init();
    lora_init();

    printf("\n--- Pico #1 Sensor Hub Initialized ---\n");
    printf("I2C GP6/7 | Radar UART1 GP4/5 | LoRa UART0 GP0/1\n");
    printf("--------------------------------------------------------------\n");
}

// -----------------------------------------------------------------------------
// --- MAIN LOOP ---
// -----------------------------------------------------------------------------
const uint32_t SENSOR_READ_INTERVAL_MS = 500;

int main() {
    init_peripherals();
    absolute_time_t last_sensor_time = get_absolute_time();

    while (1) {
        // Continuous non-blocking radar processing to catch data as it arrives
        read_and_process_radar();  

        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_sensor_time, now) >= (SENSOR_READ_INTERVAL_MS * 1000)) {

            // 1. Read Temperature
            uint16_t raw_ambient = mlx90614_read_reg(MLX90614_REGISTER_TA);
            uint16_t raw_object = mlx90614_read_reg(MLX90614_REGISTER_TOBJ1);
            float temp_ambient_c = convert_to_celsius(raw_ambient);
            float temp_object_c = convert_to_celsius(raw_object);

            // 2. Perform FFT/Frequency Analysis
            int mic_shriek_status = analyze_mic_spectrum();

            // 3. Transmit Consolidated Data via LoRa
            transmit_sensor_data(temp_ambient_c, temp_object_c, mic_shriek_status);

            last_sensor_time = now;
        }
    }
}
