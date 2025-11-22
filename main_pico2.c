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
#define LORA_TX_PIN 1 // GP1
#define LORA_RX_PIN 0 // GP0
#define M0_PIN 2
#define M1_PIN 3

// -----------------------------------------------------------------------------
// --- MLX90614 I2C Configuration (Temperature Sensor) ---
// -----------------------------------------------------------------------------
#define I2C_PORT i2c0
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define TEMP_ADDRESS 0x5A
#define TEMP_REGISTER_TA 0x06
#define TEMP_REGISTER_TOBJ1 0x07

// -----------------------------------------------------------------------------
// --- Microphone ADC Configuration ---
// -----------------------------------------------------------------------------
#define ADC_PIN_P 26 // MIC_P
#define ADC_CHANNEL_P 0
#define SAMPLE_COUNT 128
#define SAMPLE_RATE_HZ 8000
#define SHRIEK_FREQ_THRESHOLD 1500
#define SHRIEK_MAG_THRESHOLD 500
#define LOUD_MAG_THRESHOLD 700

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
void temp_init();
uint16_t temp_read_reg(uint8_t reg);
float convert_to_celsius(uint16_t raw_temp);
float read_and_send_temperatures();
void adc_mic_init(void);
uint16_t read_microphone(void);
void parse_target(uint8_t *data, Target *t);
void radar_init();
void read_and_send_radar();
void check_lora_messages();
const char* analyze_mic_fft(void);
// New radar functions:
object_type_t classify_object(Target *target, Target *prev_targets, int history_count);
const char *object_type_string(object_type_t type);

// Global radar byte counter for debugging
static uint32_t radar_total_bytes = 0;
static bool lora_busy = false;

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
    gpio_set_dir(M0_PIN, GPIO_OUT);
    gpio_set_dir(M1_PIN, GPIO_OUT);
    gpio_put(M0_PIN, 0); // Transparent mode
    gpio_put(M1_PIN, 0); // Transparent mode
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
// --- Temperature Sensor Functions ---
// -----------------------------------------------------------------------------

// Initialize I2C and pins for temperature sensor
void temperature_sensor_init(void)
{
    i2c_init(I2C_PORT, 100 * 1000);  // 100 kHz I2C
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

// Read a raw 16-bit value from a sensor register
uint16_t temperature_sensor_read_register(uint8_t reg)
{
    uint8_t buffer[3];
    i2c_write_blocking(I2C_PORT, TEMP_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, TEMP_ADDRESS, buffer, 3, false);
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

// Convert raw temperature reading to Celsius
float temperature_raw_to_celsius(uint16_t raw_temperature)
{
    return ((float)raw_temperature * 0.02f) - 273.15f;
}

// Read object (human) temperature and return it
float read_and_send_temperatures(void)
{
    uint16_t raw_object = temperature_sensor_read_register(TEMP_REGISTER_TOBJ1);
    float temp_object_c = temperature_raw_to_celsius(raw_object);
    return temp_object_c;  // Return temperature for main loop logic
}


// -----------------------------------------------------------------------------
// --- Microphone ADC Functions ---
// -----------------------------------------------------------------------------
void adc_mic_init(void)
{
    adc_init();
    adc_gpio_init(ADC_PIN_P);
    adc_select_input(ADC_CHANNEL_P);
}

uint16_t read_microphone(void)
{
    adc_select_input(ADC_CHANNEL_P);
    return adc_read();
}

// Analyze microphone FFT and detect shrieks or loud sounds
const char* analyze_mic_fft(void)
{
    kiss_fft_cpx in[SAMPLE_COUNT], out[SAMPLE_COUNT];
    uint16_t samples[SAMPLE_COUNT];
    uint64_t start_time = time_us_64();
    float sample_period_us = 1000000.0f / SAMPLE_RATE_HZ;

    // 1. Collect samples
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        samples[i] = read_microphone();
        uint64_t target_us = start_time + (i + 1) * sample_period_us;
        while (time_us_64() < target_us)
            tight_loop_contents();
    }

    // 2. Calculate and remove DC offset (Mean)
    float sum = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        sum += (float)samples[i];
    }
    float mean_offset = sum / SAMPLE_COUNT;

    // Prepare input for FFT
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        // Subtract the calculated mean to center the signal around zero
        in[i].r = (float)samples[i] - mean_offset; 
        in[i].i = 0.0f;
    }

    // 3. FFT calculation
    static kiss_fft_cfg cfg = NULL;
    if (!cfg)
        cfg = kiss_fft_alloc(SAMPLE_COUNT, 0, NULL, NULL);

    kiss_fft(cfg, in, out);

    // 4. Find peak magnitude and frequency
    float max_magnitude = 0.0f;
    int max_bin = 0;
    
    // *** FIX: Start search from i=2 to skip DC (i=0) and Mains Hum (i=1 at 62.5Hz) ***
    for (int i = 2; i < SAMPLE_COUNT / 2; i++)
    {
        float mag = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
        if (mag > max_magnitude)
        {
            max_magnitude = mag;
            max_bin = i;
        }
    }

    float dominant_freq = ((float)max_bin * SAMPLE_RATE_HZ) / SAMPLE_COUNT;
    printf("[FFT] Peak Freq: %.1f Hz | Mag: %.2f\n", dominant_freq, max_magnitude);

    // 5. Detection logic
    // Note: The magnitude thresholds might need calibration now that 62.5Hz is filtered out.
    int shriek_detected = (dominant_freq > SHRIEK_FREQ_THRESHOLD) && (max_magnitude > SHRIEK_MAG_THRESHOLD);
    int loud_detected   = (dominant_freq <= SHRIEK_FREQ_THRESHOLD) && (max_magnitude > LOUD_MAG_THRESHOLD);

    if (shriek_detected)
        return "Distress sound detected!";
    else if (loud_detected)
        return "Loud noise detected! Possible trouble";
    else
        return "No sound detected";
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

// Revised function for clearer human/stationary classification
object_type_t classify_object(Target *target, Target *prev_targets, int history_count)
{
    float distance_cm = target->distance / 10.0f; // Convert mm to cm

    // 1. BLIND ZONE & MAX SPEED CHECK (Noise Filter)
    if (distance_cm < BLIND_ZONE_CM || fabsf(target->avg_speed_cm_s) >= MAX_SPEED_CM_S)
    {
        return OBJECT_NOISE;
    }

    float abs_avg_speed = fabsf(target->avg_speed_cm_s);
    float abs_instant_speed = fabsf(target->speed_cm_s);
    
    // Use instant speed if averaged speed isn't available or is near zero
    if (abs_avg_speed < 1.0f && abs_instant_speed > 1.0f)
    {
        abs_avg_speed = abs_instant_speed;
    }

    // 2. HUMAN/MOVING CHECK
    // If the object is moving faster than the typical human minimum speed, it's human.
    if (abs_avg_speed >= HUMAN_TYPICAL_SPEED_MIN) // 10.0 cm/s
    {
        return OBJECT_HUMAN;
    }

    // 3. STATIONARY/SLOW CHECK
    // If speed is below the typical human minimum, rely on distance stability.
    if (abs_avg_speed < HUMAN_TYPICAL_SPEED_MIN)
    {
        // For accurate stationary classification, we need a history.
        if (history_count < 2) 
        {
             // Not enough history, but since it's slow, assume stationary initially
             return OBJECT_STATIONARY; 
        }

        // If distance variance is low (e.g., < 1.5 cm) AND speed is slow, it's stationary (wall, furniture).
        if (target->distance_stability < 15.0f) // 15.0 mm variance = 1.5 cm
        {
            return OBJECT_STATIONARY;
        }
        else 
        {
            // If the object is slow (under 10 cm/s) but shows high distance variance/instability 
            // (e.g., breathing, small shifts), classify it as human for navigation safety.
            return OBJECT_HUMAN;
        }
    }

    return OBJECT_UNKNOWN; // Should rarely be reached with the above logic
}

void read_and_send_radar()
{
    static uint8_t buf[64];
    static int idx = 0;
    static bool in_frame = false;
    static uint32_t frame_count = 0;
    static uint32_t detection_count = 0;

    // Track previous detections (History structure remains the same)
    static Target target_history[5] = {0};
    static int history_index = 0;
    static bool has_history = false;
    static uint32_t start_time_ms = 0;

    while (uart_is_readable(RADAR_UART_ID))
    {
        uint8_t ch = uart_getc(RADAR_UART_ID);
        radar_total_bytes++;
        
        // Initialize start time on first radar data
        if (start_time_ms == 0) start_time_ms = to_ms_since_boot(get_absolute_time());

        // Detect start of frame (FD FC marker)
        if (!in_frame)
        {
            if (idx == 0 && ch == 0xFD) { buf[idx++] = ch; }
            else if (idx == 1 && ch == 0xFC) { buf[idx++] = ch; in_frame = true; }
            else { idx = 0; }
            continue;
        }
        
        // Store data
        buf[idx++] = ch;

        // Process a complete frame (47 bytes expected)
        if (idx >= 47)
        {
            in_frame = false;
            idx = 0;
            frame_count++;

            Target targets[3] = {0};

            // 1. PARSE, CALCULATE, CLASSIFY, AND REPORT ALL 3 TARGETS
            for (int i = 0; i < 3; i++)
            {
                int offset = 8 + i * 8; // offsets 8, 16, 24
                
                // Parse raw data
                targets[i].x = (int16_t)(buf[offset] | (buf[offset + 1] << 8));
                targets[i].y = (int16_t)(buf[offset + 2] | (buf[offset + 3] << 8));
                targets[i].speed = (int16_t)(buf[offset + 4] | (buf[offset + 5] << 8));
                targets[i].distance = sqrtf((float)(targets[i].x * targets[i].x + targets[i].y * targets[i].y));
                targets[i].angle = atan2f((float)targets[i].y, (float)targets[i].x) * (180.0f / M_PI);

                // Apply Filters and Initialization
                float dist_cm = targets[i].distance / 10.0f;
                
                // Check if target is valid and within the required range (20cm to 500cm)
                if (dist_cm < MIN_DISTANCE_CM || dist_cm > MAX_DISTANCE_CM ||
                    abs(targets[i].x) >= 10000 || abs(targets[i].y) >= 10000)
                {
                    continue; // Skip invalid or filtered targets
                }
                
                targets[i].frame_number = frame_count;
                targets[i].timestamp_ms = to_ms_since_boot(get_absolute_time()) - start_time_ms;

                // 2. Perform Speed/Stability Calculation (Crucial for classification)
                if (has_history)
                {
                    // Calculate instant speed (change in distance over time)
                    Target *last = &target_history[(history_index - 1 + 5) % 5];
                    float dist_change_cm = (targets[i].distance - last->distance) / 10.0f;
                    uint32_t time_diff_ms = targets[i].timestamp_ms - last->timestamp_ms;

                    if (time_diff_ms > 0)
                    {
                        targets[i].speed_cm_s = (dist_change_cm / (time_diff_ms / 1000.0f));
                        if (fabsf(targets[i].speed_cm_s) > MAX_SPEED_CM_S) targets[i].speed_cm_s = 0.0f;
                    }
                    
                    // Calculate average speed and distance stability
                    float speed_sum = targets[i].speed_cm_s;
                    float dist_sum = targets[i].distance;
                    float dist_sum_sq = targets[i].distance * targets[i].distance;
                    int count = 1;

                    for (int j = 1; j < 3 && j < history_index; j++)
                    {
                        int hist_idx = (history_index - 1 - j + 5) % 5;
                        if (target_history[hist_idx].distance > 0)
                        {
                            speed_sum += target_history[hist_idx].speed_cm_s;
                            dist_sum += target_history[hist_idx].distance;
                            dist_sum_sq += target_history[hist_idx].distance * target_history[hist_idx].distance;
                            count++;
                        }
                    }
                    targets[i].avg_speed_cm_s = speed_sum / count;

                    float mean = dist_sum / count;
                    float variance = (dist_sum_sq / count) - (mean * mean);
                    targets[i].distance_stability = sqrtf(variance);
                }

                // 3. Classify
                targets[i].object_type = classify_object(&targets[i], target_history, history_index);

                // 4. REPORT AND SEND IF CLASSIFICATION IS VALID (Non-Noise)
                if (targets[i].object_type != OBJECT_NOISE)
                {
                    // Update history buffer with the current valid target
                    target_history[history_index] = targets[i];
                    history_index = (history_index + 1) % 5;
                    if (!has_history) has_history = true;

                    detection_count++;
                    
                    // Print full output
                    printf("[Detection #%lu] %s (Target %d)\n", detection_count, object_type_string(targets[i].object_type), i + 1);
                    printf("  Distance: %.1f cm (%.2f m)\n", dist_cm, dist_cm / 100.0f);
                    printf("  Position: X=%d, Y=%d | Angle: %.1f°\n", targets[i].x, targets[i].y, targets[i].angle);
                    if (fabsf(targets[i].speed_cm_s) > 1.0f) {
                        const char *direction = targets[i].speed_cm_s > 0 ? "away" : "closer";
                        printf("  Speed: %.1f cm/s - moving %s\n", fabsf(targets[i].speed_cm_s), direction);
                    } else {
                        printf("  Speed: < 1 cm/s - stationary\n");
                    }
                    printf("\n");

                    // Send to LoRa using mapping format: [RADAR,TYPE,DIST_CM,ANGLE_DEG,SPEED_CM_S]\n
                    char msg[256];
                    snprintf(msg, sizeof(msg),
                             "[RADAR,%s,%.1f,%.1f,%.1f]\n",
                             object_type_string(targets[i].object_type),
                             dist_cm, targets[i].angle, fabsf(targets[i].speed_cm_s));
                             
                    // Ensure LoRa is not busy
                    uint32_t wait_count = 0;
                    while (lora_busy && wait_count < 50) {
                        sleep_ms(10);
                        wait_count++;
                    }
                    
                    if (!lora_busy) {
                        lora_busy = true;
                        uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));
                        sleep_ms(100); 
                        lora_busy = false;
                    }
                }
            } // End of 3-target loop
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
    temperature_sensor_init();
    adc_mic_init();
    radar_init();
}

// -----------------------------------------------------------------------------
// --- MAIN LOOP ---
// -----------------------------------------------------------------------------
const uint32_t SENSOR_READ_INTERVAL_MS = 1000;

int main(void)
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

        // Sensor read interval
        if (absolute_time_diff_us(last_sensor_time, now) >= (SENSOR_READ_INTERVAL_MS * 1000))
        {
            float temp_c = read_and_send_temperatures();

            // Get microphone status string
            const char* mic_status = analyze_mic_fft();

            // Prepare message
            char msg[128];
            snprintf(msg, sizeof(msg), "[TEMP] Object: %.1fC | [MIC] Status: %s\n",
                     temp_c, mic_status);

            // Print and send via LoRa
            printf("%s", msg);
            uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));

            last_sensor_time = now;
        }

        sleep_ms(10);
    }
}
