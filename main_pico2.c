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
#include "MMWAVE/mmwave_detect.h"
#include "Spider/spider_main.h"

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
// --- mmWave Radar Configuration (now handled by external module) ---
// -----------------------------------------------------------------------------
// mmWave radar functionality is now provided by MMWAVE/mmwave_detect.h
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
void check_lora_messages();
const char *analyze_mic_fft(void);
void process_mmwave_data(void);
bool request_temp_send = false;
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
                // Detect spider command (SC)
                if (strncmp(rx_buffer, "SC", 2) == 0)
                {
                    int command = atoi(&rx_buffer[2]);
                    printf("Received Spider Command: %d\n", command);
                }
                // Detect human found → request to send temp
                if (strcmp(rx_buffer, "found human") == 0)
                {
                    request_temp_send = true;
                }
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

// -----------------------------------------------------------------------------
// --- Temperature Sensor Functions ---
// -----------------------------------------------------------------------------
// Initialize I2C and pins for temperature sensor
void temperature_sensor_init(void)
{
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz I2C
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
    return temp_object_c; // Return temperature for main loop logic
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
const char *analyze_mic_fft(void)
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
    int loud_detected = (dominant_freq <= SHRIEK_FREQ_THRESHOLD) && (max_magnitude > LOUD_MAG_THRESHOLD);
    if (shriek_detected)
        return "Distress sound detected!";
    else if (loud_detected)
        return "Loud noise detected! Possible trouble";
    else
        return "No sound detected";
}
// Process mmWave data and send detections via LoRa
void process_mmwave_data(void)
{
    static mmwave_target_info_t last_reported_target = {0};
    static uint32_t last_report_time = 0;
    // Process incoming mmWave data
    mmwave_process_data();
    // Check for new detections
    mmwave_target_info_t target;
    if (mmwave_get_latest_detection(&target))
    {
        last_reported_target = target;
        last_report_time = to_ms_since_boot(get_absolute_time());
        // Print detection info
        printf("[mmWave Detection] %s\n", mmwave_object_type_string(target.object_type));
        printf("  Distance: %.1f cm (%.2f m)\n", target.distance_cm, target.distance_cm / 100.0f);
        if (target.x != 0 || target.y != 0)
        {
            printf("  Position: X=%d, Y=%d | Angle: %.1f°\n", target.x, target.y, target.angle_deg);
        }
        if (fabsf(target.speed_cm_s) > 1.0f)
        {
            const char *direction = target.speed_cm_s > 0 ? "away" : "closer";
            printf("  Speed: %.1f cm/s (%.2f m/s) - moving %s\n",
                   fabsf(target.speed_cm_s), fabsf(target.speed_cm_s) / 100.0f, direction);
        }
        else
        {
            printf("  Speed: < 1 cm/s - stationary\n");
        }
        printf("  Frame #%lu | Time: %.1f s\n", target.frame_number, target.timestamp_ms / 1000.0f);
        printf("  Detection Count: %lu | Signal: %.1f | Stability: %.1f cm\n",
               target.detection_count, target.signal_strength, target.distance_stability);
        printf("  Avg Speed: %.2f cm/s | Instant Speed: %.2f cm/s\n\n",
               target.avg_speed_cm_s, target.speed_cm_s);
        // Send to LoRa
        char msg[256];
        snprintf(msg, sizeof(msg), "[RADAR,%s,%.1f,%.1f,%.1f]\n",
                 mmwave_object_type_string(target.object_type),
                 target.distance_cm, target.angle_deg, fabsf(target.speed_cm_s));
        // Ensure LoRa is not busy
        uint32_t wait_count = 0;
        while (lora_busy && wait_count < 50)
        {
            sleep_ms(10);
            wait_count++;
        }
        if (!lora_busy)
        {
            lora_busy = true;
            uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));
            sleep_ms(100);
            lora_busy = false;
        }
    }
    // Re-report stationary objects every 10 seconds for continuous tracking
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (last_reported_target.distance_cm > 0 &&
        (last_reported_target.object_type == MMWAVE_OBJECT_WALL ||
         last_reported_target.object_type == MMWAVE_OBJECT_FURNITURE ||
         last_reported_target.object_type == MMWAVE_OBJECT_SMALL_ITEM) &&
        (current_time - last_report_time) > 10000) // 10 seconds
    {
        printf("[mmWave Re-Detection] %s (Continuous)\n",
               mmwave_object_type_string(last_reported_target.object_type));
        printf("  Distance: %.1f cm (%.2f m) - Still present\n",
               last_reported_target.distance_cm, last_reported_target.distance_cm / 100.0f);
        // Send periodic update via LoRa
        char msg[256];
        snprintf(msg, sizeof(msg), "[RADAR,%s,%.1f,%.1f,%.1f]\n",
                 mmwave_object_type_string(last_reported_target.object_type),
                 last_reported_target.distance_cm, last_reported_target.angle_deg,
                 fabsf(last_reported_target.speed_cm_s));
        if (!lora_busy)
        {
            lora_busy = true;
            uart_write_blocking(LORA_UART_ID, (uint8_t *)msg, strlen(msg));
            sleep_ms(50);
            lora_busy = false;
        }
        last_report_time = current_time;
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
    mmwave_init();
}
// -----------------------------------------------------------------------------
// --- MAIN LOOP ---
// -----------------------------------------------------------------------------
const uint32_t SENSOR_READ_INTERVAL_MS = 1000;
int main(void)
{
    init_peripherals();
    initialize_spider();

    absolute_time_t last_sensor_time = get_absolute_time();
    while (1)
    {
        check_lora_messages();
        process_mmwave_data();
        absolute_time_t now = get_absolute_time();
        // Sensor read interval
        if (absolute_time_diff_us(last_sensor_time, now) >= (SENSOR_READ_INTERVAL_MS * 1000))
        {
            last_sensor_time = now;
            // ---- TEMPERATURE READ ----
            float temp_c = read_and_send_temperatures();
            printf("[TEMP] Object: %.1fC\n", temp_c);
            // ---- MICROPHONE LOGIC ----
            const char *mic_status = analyze_mic_fft();
            // Always print locally
            printf("[MIC] %s\n", mic_status);
            // Only send if it's a real alert
            if (strcmp(mic_status, "Distress sound detected!") == 0 ||
                strcmp(mic_status, "Loud noise detected! Possible trouble") == 0)
            {
                char mic_msg[128];
                snprintf(mic_msg, sizeof(mic_msg), "[MIC ALERT] %s\n", mic_status);
                uart_write_blocking(LORA_UART_ID, (uint8_t *)mic_msg, strlen(mic_msg));
            }
            // ---- TEMPERATURE SEND LOGIC ----
            
            if (request_temp_send)
            {
                char temp_msg[128];
                snprintf(temp_msg, sizeof(temp_msg),
                         "human confirmed, temperature %.1f degrees\n", temp_c);
                uart_write_blocking(LORA_UART_ID,
                                    (uint8_t *)temp_msg, strlen(temp_msg));
                request_temp_send = false;   // reset request
            }
        }
        sleep_ms(10);
    }
}