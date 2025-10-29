// for pico 2 - Sensor Pico with LoRa Communication

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "kissfft/kiss_fft.h"

// -----------------------------------------------------------------------------
// --- LoRa UART Configuration ---
// -----------------------------------------------------------------------------
#define LORA_UART_ID uart0
#define LORA_BAUD_RATE 9600
#define LORA_TX_PIN 0 // GP0
#define LORA_RX_PIN 1 // GP1
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
#define ADC_PIN_P 26 // MIC_P
#define ADC_PIN_N 27 // MIC_N
#define ADC_CHANNEL_P 0
#define ADC_CHANNEL_N 1
#define SAMPLE_COUNT 128
#define SAMPLE_RATE_HZ 8000
#define SHRIEK_FREQ_THRESHOLD 1500
#define SHRIEK_MAG_THRESHOLD 300

// -----------------------------------------------------------------------------
// --- HLK-LD2450 mmWave Radar UART Configuration ---
// -----------------------------------------------------------------------------
#define RADAR_UART_ID uart1
#define RADAR_TX_PIN 4
#define RADAR_RX_PIN 5
#define RADAR_BAUD_R operationTE 115200

#define FRAME_HEADER1 0xFD
#define FRAME_HEADER2 0xFC

typedef struct
{
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
int analyze_mic_fft();

// -----------------------------------------------------------------------------
// --- LoRa Functions ---
// -----------------------------------------------------------------------------
void lora_init()
{
    uart_init(LORA_UART_ID, LORA_BAUD_RATE);
    gpio_set_function(LORA_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LORA_RX_PIN, GPIO_FUNC_UART);

    gpio_init(M0_PIN);
    gpio_init(M1_PIN);
    gpio_set_dir(GPIO_OUT, M0_PIN);
    gpio_set_dir(GPIO_OUT, M1_PIN);
    gpio_put(M0_PIN, 0);
    gpio_put(M1_PIN, 0);
}

void check_lora_messages()
{
    static char rx_buffer[256];
    static int rx_idx = 0;

    while (uart_is_readable(LORA_UART_ID))
    {
        char c = uart_getc(LORA_UART_ID);

        if (c == '\n' || c == '\r')
        {
            if (rx_idx > 0)
            {
                rx_buffer[rx_idx] = '\0';
                printf("\n>>> RECEIVED FROM LAPTOP: %s <<<\n\n", rx_buffer);
                rx_idx = 0;
            }
        }
        else
        {
            if (rx_idx < 255)
            {
                rx_buffer[rx_idx++] = c;
            }
        }
    }
}

void test_lora_transmission()
{
    char test_msg[] = "TEST_MESSAGE_FROM_PICO2\n";
    printf(">>> SENDING TEST: %s", test_msg);
    uart_write_blocking(LORA_UART_ID, (uint8_t *)test_msg, strlen(test_msg));
    printf(">>> TEST SENT\n");
}

// -----------------------------------------------------------------------------
// --- MLX90614 Temperature Functions ---
// -----------------------------------------------------------------------------
void mlx90614_init()
{
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

uint16_t mlx90614_read_reg(uint8_t reg)
{
    uint8_t buffer[3];
    i2c_write_blocking(I2C_PORT, MLX90614_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MLX90614_ADDRESS, buffer, 3, false);
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

float convert_to_celsius(uint16_t raw_temp)
{
    return ((float)raw_temp * 0.02f) - 273.15f;
}

void read_and_send_temperatures()
{
    uint16_t raw_ambient = mlx90614_read_reg(MLX90614_REGISTER_TA);
    uint16_t raw_object = mlx90614_read_reg(MLX90614_REGISTER_TOBJ1);
    float temp_ambient_c = convert_to_celsius(raw_ambient);
    float temp_object_c = convert_to_celsius(raw_object);

    char msg[128];
    snprintf(msg, sizeof(msg), "[TEMP] Ambient: %.2f C | Object: %.2f C\n",
             temp_ambient_c, temp_object_c);

    printf("%s", msg);
    printf(">>> SENDING TO LORA: %s", msg);
    uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));
    printf(">>> SENT %d bytes\n", strlen(msg));
}

// -----------------------------------------------------------------------------
// --- Microphone ADC Functions ---
// -----------------------------------------------------------------------------
void adc_mic_init()
{
    adc_init();
    adc_gpio_init(ADC_PIN_P);
    adc_gpio_init(ADC_PIN_N);
    adc_select_input(ADC_CHANNEL_P);
}

uint16_t read_microphone()
{
    adc_select_input(ADC_CHANNEL_P);
    uint16_t mic_p = adc_read();

    adc_select_input(ADC_CHANNEL_N);
    uint16_t mic_n = adc_read();

    int32_t differential = (int32_t)mic_p - (int32_t)mic_n;

    return (uint16_t)(differential + 2048);
}

int analyze_mic_fft()
{
    kiss_fft_cpx in[SAMPLE_COUNT], out[SAMPLE_COUNT];
    uint16_t samples[SAMPLE_COUNT];
    uint64_t start_time = time_us_64();
    float sample_period_us = 1000000.0f / SAMPLE_RATE_HZ;

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        samples[i] = read_microphone();
        uint64_t target_us = start_time + (i + 1) * sample_period_us;
        while (time_us_64() < target_us)
            tight_loop_contents();
    }

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        in[i].r = (float)samples[i] - 2048.0f;
        in[i].i = 0.0f;
    }

    kiss_fft_cfg cfg = kiss_fft_alloc(SAMPLE_COUNT, 0, NULL, NULL);
    kiss_fft(cfg, in, out);
    free(cfg);

    float max_magnitude = 0.0f;
    int max_bin = 0;
    for (int i = 1; i < SAMPLE_COUNT / 2; i++)
    {
        float mag = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
        if (mag > max_magnitude)
        {
            max_magnitude = mag;
            max_bin = i;
        }
    }

    float dominant_freq = ((float)max_bin * SAMPLE_RATE_HZ) / SAMPLE_COUNT;
    printf("| [FFT] Peak Freq: %.1f Hz | Mag: %.2f", dominant_freq, max_magnitude);

    if ((dominant_freq > SHRIEK_FREQ_THRESHOLD) && (max_magnitude > SHRIEK_MAG_THRESHOLD))
    {
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// --- Radar (HLK-LD2450) Functions ---
// -----------------------------------------------------------------------------
void radar_init()
{
    uart_init(RADAR_UART_ID, 115200);
    gpio_set_function(RADAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RADAR_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(RADAR_UART_ID, false, false);

    sleep_ms(100);

    printf("[RADAR] UART initialized at %d baud on GP%d (TX) / GP%d (RX)\n",
           115200, RADAR_TX_PIN, RADAR_RX_PIN);
}

void parse_target(uint8_t *data, Target *t)
{
    t->x = (int16_t)(data[0] | (data[1] << 8));
    t->y = (int16_t)(data[2] | (data[3] << 8));
    t->speed = (int16_t)(data[4] | (data[5] << 8));
    t->distance = sqrtf((float)(t->x * t->x + t->y * t->y));
    t->angle = atan2f((float)t->y, (float)t->x) * (180.0f / M_PI);
}

void read_and_send_radar()
{
    static uint8_t buffer[64];
    static int index = 0;
    static bool in_frame = false;

    while (uart_is_readable(RADAR_UART_ID))
    {
        uint8_t byte = uart_getc(RADAR_UART_ID);

        if (!in_frame)
        {
            if (index == 0 && byte == FRAME_HEADER1)
            {
                buffer[index++] = byte;
            }
            else if (index == 1 && byte == FRAME_HEADER2)
            {
                buffer[index++] = byte;
                in_frame = true;
            }
            else
            {
                index = 0;
            }
            continue;
        }

        buffer[index++] = byte;

        bool should_parse = false;

        if (index >= 20)
        {
            should_parse = true;
        }
        else if (index >= 8 && byte == 0xFD && index > 8)
        {
            index--;
            should_parse = true;
        }

        if (should_parse)
        {
            printf("[RADAR-DEBUG] Frame len=%d: ", index);
            for (int i = 0; i < index && i < 24; i++)
            {
                printf("%02X ", buffer[i]);
            }
            printf("\n");

            int offset = 2;
            Target targets[3] = {0};
            int target_count = 0;

            if (offset < index && (buffer[offset] == 0x40 || buffer[offset] == 0xC0))
            {
                offset++;
            }

            for (int t = 0; t < 3 && offset + 6 <= index; t++)
            {
                int16_t x = (int16_t)(buffer[offset] | (buffer[offset + 1] << 8));
                int16_t y = (int16_t)(buffer[offset + 2] | (buffer[offset + 3] << 8));
                int16_t speed = (int16_t)(buffer[offset + 4] | (buffer[offset + 5] << 8));

                printf("[RADAR-DEBUG] T%d @offset%d: x=%04X(%d) y=%04X(%d) v=%04X(%d)\n",
                       t + 1, offset,
                       (uint16_t)x, x,
                       (uint16_t)y, y,
                       (uint16_t)speed, speed);

                if (abs(x) < 10000 && abs(y) < 10000 && abs(speed) < 5000)
                {
                    targets[t].x = x;
                    targets[t].y = y;
                    targets[t].speed = speed;
                    targets[t].distance = sqrtf((float)(x * x + y * y));
                    targets[t].angle = atan2f((float)y, (float)x) * (180.0f / M_PI);
                    target_count++;
                    offset += 6;
                }
                else
                {
                    printf("[RADAR-DEBUG] T%d rejected: x=%d y=%d v=%d AREA(out of range)\n",
                           t + 1, x, y, speed);
                    break;
                }
            }

            if (target_count > 0)
            {
                char msg[256];
                if (target_count == 1)
                {
                    snprintf(msg, sizeof(msg),
                             "[RADAR] T1: %.1fmm %.1fdeg %dmm/s\n",
                             targets[0].distance, targets[0].angle, targets[0].speed);
                }
                else if (target_count == 2)
                {
                    snprintf(msg, sizeof(msg),
                             "[RADAR] T1: %.1fmm %.1fdeg %dmm/s | T2: %.1fmm %.1fdeg %dmm/s\n",
                             targets[0].distance, targets[0].angle, targets[0].speed,
                             targets[1].distance, targets[1].angle, targets[1].speed);
                }
                else
                {
                    snprintf(msg, sizeof(msg),
                             "[RADAR] T1: %.1fmm %.1fdeg %dmm/s | T2: %.1fmm %.1fdeg %dmm/s | T3: %.1fmm %.1fdeg %dmm/s\n",
                             targets[0].distance, targets[0].angle, targets[0].speed,
                             targets[1].distance, targets[1].angle, targets[1].speed,
                             targets[2].distance, targets[2].angle, targets[2].speed);
                }
                printf("%s", msg);
                uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));
            }

            in_frame = false;

            if (byte == 0xFD)
            {
                buffer[0] = 0xFD;
                index = 1;
            }
            else
            {
                index = 0;
            }
            continue;
        }

        if (index >= sizeof(buffer))
        {
            in_frame = false;
            index = 0;
        }
    }
}

// -----------------------------------------------------------------------------
// --- Initialization ---
// -----------------------------------------------------------------------------
void init_peripherals()
{
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
const uint32_t SENSOR_READ_INTERVAL_MS = 1000;

int main()
{
    init_peripherals();
    absolute_time_t last_sensor_time = get_absolute_time();

    while (1)
    {
        check_lora_messages();

        read_and_send_radar();

        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_sensor_time, now) >= (SENSOR_READ_INTERVAL_MS * 1000))
        {
            read_and_send_temperatures();

            int shriek_detected = analyze_mic_fft();
            char msg[128];
            snprintf(msg, sizeof(msg), " | [MIC] Shriek: %d\n", shriek_detected);
            printf("%s", msg);
            uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));

            printf("\n");
            last_sensor_time = now;
        }

        sleep_ms(10);
    }
}