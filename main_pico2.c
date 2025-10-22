//for pico 2 - Sensor Pico with LoRa Communication

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// -----------------------------------------------------------------------------
// --- LoRa UART Configuration ---
// -----------------------------------------------------------------------------
#define LORA_UART_ID uart0
#define LORA_BAUD_RATE 9600
#define LORA_TX_PIN 0  // GP0
#define LORA_RX_PIN 1  // GP1
#define M0_PIN 2
#define M1_PIN 3

// -----------------------------------------------------------------------------
// --- MLX90614 I2C Configuration (Temperature Sensor) ---
// -----------------------------------------------------------------------------
#define I2C_PORT i2c0
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define MLX90614_ADDRESS 0x5A
#define MLX90614_REGISTER_TA 0x06
#define MLX90614_REGISTER_TOBJ1 0x07

// -----------------------------------------------------------------------------
// --- Microphone ADC Configuration ---
// -----------------------------------------------------------------------------
#define ADC_PIN_P 26  // MIC_P
#define ADC_PIN_N 27  // MIC_N
#define ADC_CHANNEL_P 0
#define ADC_CHANNEL_N 1

// -----------------------------------------------------------------------------
// --- HLK-LD2450 mmWave Radar UART Configuration ---
// -----------------------------------------------------------------------------
#define RADAR_UART_ID uart1
#define RADAR_TX_PIN 4
#define RADAR_RX_PIN 5
#define RADAR_BAUD_RATE 115200

#define FRAME_HEADER1 0xFD
#define FRAME_HEADER2 0xFC

typedef struct {
    int16_t x;
    int16_t y;
    int16_t speed;
    float distance;
    float angle;
} Target;

// -----------------------------------------------------------------------------
// --- Function Prototypes ---
// -----------------------------------------------------------------------------
void init_peripherals();
void lora_init();
void mlx90614_init();
uint16_t mlx90614_read_reg(uint8_t reg);
float convert_to_celsius(uint16_t raw_temp);
void read_and_send_temperatures();
void adc_mic_init();
uint16_t read_microphone();
void parse_target(uint8_t *data, Target *t);
void radar_init();
void read_and_send_radar();
void check_lora_messages();

// -----------------------------------------------------------------------------
// --- LoRa Functions ---
// -----------------------------------------------------------------------------
void lora_init() {
    uart_init(LORA_UART_ID, LORA_BAUD_RATE);
    gpio_set_function(LORA_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LORA_RX_PIN, GPIO_FUNC_UART);
    
    // Set mode control pins (M0=0, M1=0 for transparent mode)
    gpio_init(M0_PIN);
    gpio_init(M1_PIN);
    gpio_set_dir(M0_PIN, GPIO_OUT);
    gpio_set_dir(M1_PIN, GPIO_OUT);
    gpio_put(M0_PIN, 0);
    gpio_put(M1_PIN, 0);
}

void check_lora_messages() {
    static char rx_buffer[256];
    static int rx_idx = 0;
    
    while (uart_is_readable(LORA_UART_ID)) {
        char c = uart_getc(LORA_UART_ID);
        
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buffer[rx_idx] = '\0';
                printf("\n>>> RECEIVED FROM LAPTOP: %s <<<\n\n", rx_buffer);
                rx_idx = 0;
            }
        } else {
            if (rx_idx < 255) {
                rx_buffer[rx_idx++] = c;
            }
        }
    }
}

void test_lora_transmission() {
    char test_msg[] = "TEST_MESSAGE_FROM_PICO2\n";
    printf(">>> SENDING TEST: %s", test_msg);
    uart_write_blocking(LORA_UART_ID, (uint8_t*)test_msg, strlen(test_msg));
    printf(">>> TEST SENT\n");
}

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
    i2c_write_blocking(I2C_PORT, MLX90614_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MLX90614_ADDRESS, buffer, 3, false);
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

float convert_to_celsius(uint16_t raw_temp) {
    return ((float)raw_temp * 0.02f) - 273.15f;
}

void read_and_send_temperatures() {
    uint16_t raw_ambient = mlx90614_read_reg(MLX90614_REGISTER_TA);
    uint16_t raw_object = mlx90614_read_reg(MLX90614_REGISTER_TOBJ1);
    float temp_ambient_c = convert_to_celsius(raw_ambient);
    float temp_object_c = convert_to_celsius(raw_object);

    char msg[128];
    snprintf(msg, sizeof(msg), "[TEMP] Ambient: %.2f C | Object: %.2f C\n", 
             temp_ambient_c, temp_object_c);
    
    printf("%s", msg);  // Print locally
    printf(">>> SENDING TO LORA: %s", msg);  // DEBUG: Show what we're sending
    uart_write_blocking(LORA_UART_ID, (uint8_t*)msg, strlen(msg));  // Send via LoRa
    printf(">>> SENT %d bytes\n", strlen(msg));  // DEBUG: Confirm bytes sent
}

// -----------------------------------------------------------------------------
// --- Microphone ADC Functions ---
// -----------------------------------------------------------------------------
void adc_mic_init() {
    adc_init();
    adc_gpio_init(ADC_PIN_P);
    adc_gpio_init(ADC_PIN_N);
    adc_select_input(ADC_CHANNEL_P);
}

uint16_t read_microphone() {
    // Read both channels
    adc_select_input(ADC_CHANNEL_P);
    uint16_t mic_p = adc_read();
    
    adc_select_input(ADC_CHANNEL_N);
    uint16_t mic_n = adc_read();
    
    // Calculate differential (P - N)
    int32_t differential = (int32_t)mic_p - (int32_t)mic_n;
    
    // Convert to unsigned (add offset to make positive)
    return (uint16_t)(differential + 2048);
}

// -----------------------------------------------------------------------------
// --- Radar (HLK-LD2450) Functions ---
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
    t->distance = sqrtf((float)(t->x * t->x + t->y * t->y));
    t->angle = atan2f((float)t->x, (float)t->y) * (180.0f / M_PI);
}

void read_and_send_radar() {
    static uint8_t buffer[64];
    static int index = 0;
    static bool in_frame = false;

    while (uart_is_readable(RADAR_UART_ID)) {
        uint8_t byte = uart_getc(RADAR_UART_ID);

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

        buffer[index++] = byte;

        if (index >= 47) {
            in_frame = false;
            index = 0;

            Target t1, t2, t3;
            parse_target(&buffer[8], &t1);
            parse_target(&buffer[16], &t2);
            parse_target(&buffer[24], &t3);

            char msg[256];
            snprintf(msg, sizeof(msg), 
                     "[RADAR] T1: %.1fmm %.1fdeg %dmm/s | T2: %.1fmm %.1fdeg %dmm/s | T3: %.1fmm %.1fdeg %dmm/s\n",
                     t1.distance, t1.angle, t1.speed,
                     t2.distance, t2.angle, t2.speed,
                     t3.distance, t3.angle, t3.speed);
            
            printf("%s", msg);  // Print locally
            uart_write_blocking(LORA_UART_ID, (uint8_t*)msg, strlen(msg));  // Send via LoRa
        }
    }
}

// -----------------------------------------------------------------------------
// --- Initialization ---
// -----------------------------------------------------------------------------
void init_peripherals() {
    stdio_init_all();
    lora_init();
    mlx90614_init();
    adc_mic_init();
    radar_init();

    printf("\n--- Pico 2: Sensor Pico with LoRa Initialized ---\n");
    printf("MLX90614 on I2C GP8/9 | Mic on ADC GP26 | Radar on UART1 GP4/5\n");
    printf("LoRa on UART0 GP0/1\n");
    printf("-----------------------------------------------------------\n");
}

// -----------------------------------------------------------------------------
// --- MAIN LOOP ---
// -----------------------------------------------------------------------------
const uint32_t SENSOR_READ_INTERVAL_MS = 1000;  // Read sensors every 1 second

int main() {
    init_peripherals();
    absolute_time_t last_sensor_time = get_absolute_time();

    while (1) {
        // Continuously check for incoming LoRa messages
        check_lora_messages();
        
        // Continuously read radar (non-blocking)
        read_and_send_radar();

        // Periodically read other sensors
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_sensor_time, now) >= (SENSOR_READ_INTERVAL_MS * 1000)) {
            read_and_send_temperatures();
            
            uint16_t mic_val = read_microphone();
            char msg[64];
            snprintf(msg, sizeof(msg), "[MIC] Raw ADC: %d\n", mic_val);
            printf("%s", msg);
            uart_write_blocking(LORA_UART_ID, (uint8_t*)msg, strlen(msg));
            
            printf("\n");  // Separator
            last_sensor_time = now;
        }
        
        sleep_ms(10);
    }
}