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
#define RADAR_BAUD_RATE 115200 // FIXED: was "operationTE 115200"

// Frame markers from mmwave_detect.c
#define MARKER_FE_F8 0xFEF8
#define MARKER_FD_F8 0xFDF8
#define MARKER_FE_FC 0xFEFC
#define MARKER_FD_FC 0xFDFC

// Detection thresholds (in cm)
#define BLIND_ZONE_CM 25         // Blind zone - ignore detections closer than this
#define MIN_DISTANCE_CM 20       // Lower this from 30 to 20
#define MAX_DISTANCE_CM 500      // Increase this from 150 to 500
#define MIN_DISTANCE_CHANGE_CM 2 // Minimum distance change to calculate speed
#define MAX_SPEED_CM_S 500       // Maximum realistic speed
#define FRAME_RATE_HZ 10         // Approximate radar frame rate

// Object classification thresholds
#define STATIONARY_SPEED_THRESHOLD 15.0f // Below this is considered stationary
#define HUMAN_TYPICAL_SPEED_MIN 10.0f    // Humans typically move at least this fast

// Object types
typedef enum
{
    OBJECT_UNKNOWN,
    OBJECT_HUMAN,
    OBJECT_STATIONARY, // Wall, furniture, etc.
    OBJECT_NOISE       // Likely false detection
} object_type_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t speed;
    float distance; // in mm (existing)
    float angle;
    // New fields from mmwave_detect:
    float speed_cm_s;     // Positive = moving away, Negative = moving closer
    float avg_speed_cm_s; // Averaged speed over recent frames
    object_type_t object_type;
    uint32_t frame_number;
    uint32_t timestamp_ms;
    float distance_stability; // How stable the distance is
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
// New radar functions:
object_type_t classify_object(Target *target, Target *prev_targets, int history_count);
const char *object_type_string(object_type_t type);

// Global radar byte counter for debugging
static uint32_t radar_total_bytes = 0;

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
    uart_init(RADAR_UART_ID, RADAR_BAUD_RATE); // FIXED
    gpio_set_function(RADAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RADAR_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(RADAR_UART_ID, false, false);

    sleep_ms(100);

    printf("[RADAR] UART initialized at %d baud on GP%d (TX) / GP%d (RX)\n",
           RADAR_BAUD_RATE, RADAR_TX_PIN, RADAR_RX_PIN);
}

void parse_target(uint8_t *data, Target *t)
{
    t->x = (int16_t)(data[0] | (data[1] << 8));
    t->y = (int16_t)(data[2] | (data[3] << 8));
    t->speed = (int16_t)(data[4] | (data[5] << 8));
    t->distance = sqrtf((float)(t->x * t->x + t->y * t->y));
    t->angle = atan2f((float)t->y, (float)t->x) * (180.0f / M_PI);
}

// Get object type as string
const char *object_type_string(object_type_t type)
{
    switch (type)
    {
    case OBJECT_HUMAN:
        return "Human";
    case OBJECT_STATIONARY:
        return "Stationary";
    case OBJECT_NOISE:
        return "Noise";
    default:
        return "Unknown";
    }
}

// Classify object based on motion characteristics
object_type_t classify_object(Target *target, Target *prev_targets, int history_count)
{
    float distance_cm = target->distance / 10.0f; // Convert mm to cm

    // Very close detections are likely noise
    if (distance_cm < BLIND_ZONE_CM)
    {
        return OBJECT_NOISE;
    }

    float abs_speed = fabsf(target->avg_speed_cm_s);
    float abs_instant_speed = fabsf(target->speed_cm_s);

    // Use instant speed if averaged speed isn't available yet
    if (abs_speed < 1.0f && abs_instant_speed > 1.0f)
    {
        abs_speed = abs_instant_speed;
    }

    // Stationary objects: very low speed AND stable distance
    if (abs_speed < STATIONARY_SPEED_THRESHOLD)
    {
        if (target->distance_stability < 10.0f || history_count < 2)
        {
            return OBJECT_STATIONARY;
        }
        if (abs_speed > 2.0f)
        {
            return OBJECT_HUMAN;
        }
        return OBJECT_STATIONARY;
    }

    // High speed: likely human (or noise if too high)
    if (abs_speed >= HUMAN_TYPICAL_SPEED_MIN)
    {
        if (abs_speed >= MAX_SPEED_CM_S)
        {
            return OBJECT_NOISE;
        }
        return OBJECT_HUMAN;
    }

    // Between thresholds - use distance stability as tiebreaker
    if (history_count >= 2)
    {
        if (target->distance_stability < 5.0f)
        {
            return OBJECT_STATIONARY;
        }
        else
        {
            return OBJECT_HUMAN;
        }
    }

    return OBJECT_HUMAN;
}

void read_and_send_radar()
{
    static uint8_t buf[64];
    static int idx = 0;
    static bool in_frame = false;
    static uint32_t frame_count = 0;
    static uint32_t detection_count = 0;

    // Track previous detections
    static Target target_history[5] = {0};
    static int history_index = 0;
    static bool has_history = false;
    static uint32_t start_time_ms = 0;

    while (uart_is_readable(RADAR_UART_ID))
    {
        uint8_t ch = uart_getc(RADAR_UART_ID);
        radar_total_bytes++;

        // Initialize start time on first radar data
        if (start_time_ms == 0)
        {
            start_time_ms = to_ms_since_boot(get_absolute_time());
        }

        // Detect start of frame - try FD FC first
        if (!in_frame)
        {
            if (idx == 0 && ch == 0xFD)
            {
                buf[idx++] = ch;
            }
            else if (idx == 1 && ch == 0xFC)
            {
                buf[idx++] = ch;
                in_frame = true;
            }
            else
            {
                idx = 0;
            }
            continue;
        }

        // Store data
        buf[idx++] = ch;

        // Expect 47-byte frames
        if (idx >= 47)
        {
            in_frame = false;
            idx = 0;
            frame_count++;

            // DEBUG: Print what we got
            if (frame_count % 100 == 0)
            { // Every 100 frames
                printf("[DEBUG] Got 47-byte frame %lu: ", frame_count);
                for (int i = 0; i < 20; i++)
                {
                    printf("%02X ", buf[i]);
                }
                printf("\n");
            }

            Target targets[3] = {0};

            // Parse targets (offsets from MMWAVE/main.c)
            for (int i = 0; i < 3; i++)
            {
                int offset = 8 + i * 8; // offsets 8, 16, 24
                targets[i].x = (int16_t)(buf[offset] | (buf[offset + 1] << 8));
                targets[i].y = (int16_t)(buf[offset + 2] | (buf[offset + 3] << 8));
                targets[i].speed = (int16_t)(buf[offset + 4] | (buf[offset + 5] << 8));
                targets[i].distance = sqrtf((float)(targets[i].x * targets[i].x + targets[i].y * targets[i].y));
                targets[i].angle = atan2f((float)targets[i].y, (float)targets[i].x) * (180.0f / M_PI);

                // Filter valid targets
                float dist_cm = targets[i].distance / 10.0f;
                if (dist_cm >= MIN_DISTANCE_CM && dist_cm <= MAX_DISTANCE_CM &&
                    abs(targets[i].x) < 10000 && abs(targets[i].y) < 10000)
                {
                    targets[i].frame_number = frame_count;
                    targets[i].timestamp_ms = to_ms_since_boot(get_absolute_time()) - start_time_ms;

                    // Calculate speed from previous detection (compute speed_cm_s from frame speed)
                    if (has_history)
                    {
                        Target *last = &target_history[(history_index - 1 + 5) % 5];
                        float dist_change_cm = (targets[i].distance - last->distance) / 10.0f;
                        uint32_t time_diff_ms = targets[i].timestamp_ms - last->timestamp_ms;

                        if (time_diff_ms > 0)
                        {
                            targets[i].speed_cm_s = (dist_change_cm / (time_diff_ms / 1000.0f));
                            if (fabsf(targets[i].speed_cm_s) > MAX_SPEED_CM_S)
                            {
                                targets[i].speed_cm_s = 0.0f;
                            }
                        }

                        // Calculate average speed
                        float speed_sum = targets[i].speed_cm_s;
                        int count = 1;
                        for (int j = 1; j < 3 && j < history_index; j++)
                        {
                            int hist_idx = (history_index - 1 - j + 5) % 5;
                            if (target_history[hist_idx].distance > 0)
                            {
                                speed_sum += target_history[hist_idx].speed_cm_s;
                                count++;
                            }
                        }
                        targets[i].avg_speed_cm_s = speed_sum / count;

                        // Calculate distance stability
                        float dist_sum = targets[i].distance;
                        float dist_sum_sq = targets[i].distance * targets[i].distance;
                        for (int j = 1; j < 3 && j < history_index; j++)
                        {
                            int hist_idx = (history_index - 1 - j + 5) % 5;
                            if (target_history[hist_idx].distance > 0)
                            {
                                dist_sum += target_history[hist_idx].distance;
                                dist_sum_sq += target_history[hist_idx].distance * target_history[hist_idx].distance;
                                count++;
                            }
                        }
                        float mean = dist_sum / count;
                        float variance = (dist_sum_sq / count) - (mean * mean);
                        targets[i].distance_stability = sqrtf(variance);
                    }

                    // Classify
                    targets[i].object_type = classify_object(&targets[i], target_history, history_index);

                    if (targets[i].object_type != OBJECT_NOISE)
                    {
                        // Update history
                        target_history[history_index] = targets[i];
                        history_index = (history_index + 1) % 5;
                        if (!has_history)
                            has_history = true;

                        detection_count++;

                        // Print full formatted output like mmwave_detect
                        printf("[Detection #%lu] %s\n", detection_count, object_type_string(targets[i].object_type));
                        printf("  Distance: %.1f cm (%.2f m)\n", dist_cm, dist_cm / 100.0f);

                        if (targets[i].x != 0 || targets[i].y != 0)
                        {
                            printf("  Position: X=%d, Y=%d | Angle: %.1f°\n",
                                   targets[i].x, targets[i].y, targets[i].angle);
                        }

                        if (fabsf(targets[i].speed_cm_s) > 1.0f)
                        {
                            const char *direction = targets[i].speed_cm_s > 0 ? "away" : "closer";
                            printf("  Speed: %.1f cm/s (%.2f m/s) - moving %s\n",
                                   fabsf(targets[i].speed_cm_s),
                                   fabsf(targets[i].speed_cm_s) / 100.0f,
                                   direction);
                        }
                        else
                        {
                            printf("  Speed: < 1 cm/s - stationary\n");
                        }

                        printf("  Frame #%lu | Time: %.1f s\n",
                               targets[i].frame_number, targets[i].timestamp_ms / 1000.0f);
                        printf("\n");

                        // Also send to LoRa (simplified)
                        char msg[256];
                        if (targets[i].object_type == OBJECT_HUMAN)
                        {
                            snprintf(msg, sizeof(msg),
                                     "[RADAR-HUMAN] %.1fcm %.1fdeg %.1fcm/s\n",
                                     dist_cm, targets[i].angle, fabsf(targets[i].speed_cm_s));
                        }
                        else
                        {
                            snprintf(msg, sizeof(msg),
                                     "[RADAR] %.1fcm %.1fdeg %s\n",
                                     dist_cm, targets[i].angle, object_type_string(targets[i].object_type));
                        }
                        uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));
                    }
                }
            }
        }

        // Reset on overflow
        if (idx >= sizeof(buf))
        {
            in_frame = false;
            idx = 0;
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

    // Debug: Check if radar UART is initialized
    sleep_ms(500);
    if (uart_is_enabled(RADAR_UART_ID))
    {
        printf("[RADAR-DEBUG] UART1 is enabled\n");
    }
    else
    {
        printf("[RADAR-DEBUG] WARNING: UART1 is NOT enabled!\n");
    }
}

// -----------------------------------------------------------------------------
// --- MAIN LOOP ---
// -----------------------------------------------------------------------------
const uint32_t SENSOR_READ_INTERVAL_MS = 1000;

int main()
{
    init_peripherals();
    absolute_time_t last_sensor_time = get_absolute_time();
    absolute_time_t last_debug_time = get_absolute_time();

    while (1)
    {
        check_lora_messages();

        read_and_send_radar();

        absolute_time_t now = get_absolute_time();

        // Print radar status every 5 seconds
        if (absolute_time_diff_us(last_debug_time, now) >= 5000000)
        {
            printf("[RADAR-STATUS] Bytes received from radar: %lu\n", radar_total_bytes);
            last_debug_time = now;
        }

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