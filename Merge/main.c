#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h" // For Microphone
#include "hardware/uart.h" 
#include "hardware/gpio.h"
#include "pico/time.h"    // Required for non-blocking timing

// --- MLX90614 I2C Configuration (Temp Sensor) ---
#define I2C_PORT i2c0
#define I2C_SDA_PIN 8               // MLX90614 SDA -> Pico GP4
#define I2C_SCL_PIN 9               // MLX90614 SCL -> Pico GP5
#define MLX90614_ADDRESS 0x5A       
#define MLX90614_REGISTER_TA 0x06   // Ambient Temp
#define MLX90614_REGISTER_TOBJ1 0x07 // Object Temp

// --- Analog Microphone ADC Configuration ---
#define ADC_PIN 26                  // Microphone Signal -> Pico GP26 (ADC Channel 0)
#define ADC_CHANNEL 0               // ADC Channel 0

// // --- LoRa UART Configuration (Pico #1 for Future Use) ---
// #define LORA_UART_ID uart1
// #define LORA_BAUD_RATE 9600          
// #define LORA_TX_PIN 8               // LoRa TX -> Pico GP8
// #define LORA_RX_PIN 9               // LoRa RX <- Pico GP9

// --- Sensor Read Interval ---
const uint32_t SENSOR_READ_INTERVAL_MS = 500; // Read and transmit every 500ms

// --- Function Prototypes ---
void init_peripherals();

// --- MLX90614 Functions (Temp) ---
uint16_t mlx90614_read_reg(uint8_t reg);
float convert_to_celsius(uint16_t raw_temp);
void mlx90614_init();
void read_and_print_temperatures();

// --- ADC Functions (Mic) ---
void adc_mic_init();
uint16_t read_microphone();

// --- MLX90614 Implementation ---
void mlx90614_init() {
    i2c_init(I2C_PORT, 100 * 1000); 
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

uint16_t mlx90614_read_reg(uint8_t reg) {
    uint8_t buffer[3]; 
    i2c_write_blocking(I2C_PORT, MLX90614_ADDRESS, &reg, 1, true); 
    i2c_read_blocking(I2C_PORT, MLX90614_ADDRESS, buffer, 3, false); 
    return (uint16_t)buffer[0] | (uint16_t)(buffer[1] << 8);
}

float convert_to_celsius(uint16_t raw_temp) {
    return ((float)raw_temp * 0.02f) - 273.15f;
}

void read_and_print_temperatures() {
    uint16_t raw_ambient = mlx90614_read_reg(MLX90614_REGISTER_TA);
    uint16_t raw_object = mlx90614_read_reg(MLX90614_REGISTER_TOBJ1);

    float temp_ambient_c = convert_to_celsius(raw_ambient);
    float temp_object_c = convert_to_celsius(raw_object);

    printf("[TEMP] Ambient: %.2f C | Object: %.2f C ", temp_ambient_c, temp_object_c);
}

// --- ADC Implementation (Mic) ---
void adc_mic_init() {
    adc_init();
    adc_gpio_init(ADC_PIN); 
    adc_select_input(ADC_CHANNEL);
}

uint16_t read_microphone() {
    return adc_read();
}

// --- Combined Initialization ---
void init_peripherals() {
    stdio_init_all();
    
    // Initialize both sensors
    mlx90614_init(); // Temp Sensor I2C (GP4/GP5)
    adc_mic_init();   // Mic ADC (GP26)
    
    // // Initialize UART for LoRa (Future use)
    // uart_init(LORA_UART_ID, LORA_BAUD_RATE);
    // gpio_set_function(LORA_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(LORA_RX_PIN, GPIO_FUNC_UART);
    
    printf("\n--- Pico #1: Temp Sensor and Microphone Initialized ---\n");
    printf("I2C (MLX90614) on GP4/GP5. ADC (Mic) on GP26. LoRa (UART1) on GP8/GP9.\n");
    printf("------------------------------------------------------\n");
}

int main() {
    init_peripherals();
    
    // Initialize a variable to track the last sensor read time
    absolute_time_t last_sensor_read_time = get_absolute_time();
    
    while (1) {
        // Get the current time on every loop iteration
        absolute_time_t current_time = get_absolute_time();
        
        // Calculate the difference between the current time and the last read time in microseconds
        int64_t time_diff_us = absolute_time_diff_us(last_sensor_read_time, current_time);
        
        // Check if the required interval has passed (e.g., 500ms = 500,000 us)
        if (time_diff_us >= (int64_t)SENSOR_READ_INTERVAL_MS * 1000) {
            
            // --- 1. SENSOR READ AND PRINT (Only runs when time is up) ---
            read_and_print_temperatures();
            
            uint16_t mic_value = read_microphone();
            printf("| [MIC] Raw ADC: %d\n", mic_value);
            
            // --- 2. TRANSMISSION (Future Task) ---
            // lora_transmit_data(temp_ambient_c, mic_value); 
            
            // Update the last read time to the current time to reset the timer
            last_sensor_read_time = current_time;
        }
        
        // --- NON-BLOCKING CHECK ---
        // This is where any real-time task, such as checking for incoming LoRa commands, 
        // would be placed. The processor constantly checks this without waiting.
        // e.g., check_lora_for_incoming_commands();
    }
    
    return 0;
}