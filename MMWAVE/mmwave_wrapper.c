#include "mmwave_detect.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

// UART configuration - same as in mmwave_detect.c
#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define BAUD_RATE 115200

#define BUF_LEN 512

// Frame markers
#define MARKER_FE_F8 0xFEF8
#define MARKER_FD_F8 0xFDF8
#define MARKER_FE_FC 0xFEFC
#define MARKER_FD_FC 0xFDFC

// Detection thresholds (in cm)
#define BLIND_ZONE_CM 25
#define MIN_DISTANCE_CM 30
#define MAX_DISTANCE_CM 150
#define MIN_DISTANCE_CHANGE_CM 2
#define MAX_SPEED_CM_S 500
#define FRAME_RATE_HZ 10

// Object classification thresholds
#define STATIONARY_SPEED_THRESHOLD 15.0f
#define HUMAN_TYPICAL_SPEED_MIN 10.0f

// Global variables for mmWave processing
static uint8_t buf[BUF_LEN];
static uint16_t idx = 0;
static uint32_t frame_count = 0;
static uint32_t detection_count = 0;
static mmwave_target_info_t target_history[5] = {0};
static int history_index = 0;
static bool has_history = false;
static uint32_t global_detection_count = 0;
static uint16_t last_marker = 0;
static uint16_t frame_start_idx = 0;
static bool in_frame = false;
static uint32_t start_time_ms = 0;
static mmwave_target_info_t latest_target = {0};
static bool new_detection_available = false;
static uint32_t total_bytes_received = 0;
static uint32_t last_detection_time = 0;
// Runtime-configurable max range (0 = disabled, falls back to MAX_DISTANCE_CM)
static uint16_t max_range_cm = 0;

// Calculate angle from X/Y coordinates (in degrees)
static float calculate_angle(int16_t x, int16_t y)
{
    if (x == 0 && y == 0)
        return 0.0f;
    float angle_rad = atan2f((float)y, (float)x);
    float angle_deg = angle_rad * 180.0f / M_PI;
    return angle_deg;
}

// Enhanced object classification with wall/small object differentiation
static mmwave_object_type_t classify_object(mmwave_target_info_t *target, mmwave_target_info_t *prev_targets, int history_count)
{
    // Very close detections are likely noise
    if (target->distance_cm < BLIND_ZONE_CM)
    {
        return MMWAVE_OBJECT_NOISE;
    }

    float abs_speed = fabsf(target->avg_speed_cm_s);
    float abs_instant_speed = fabsf(target->speed_cm_s);

    // Use instant speed if averaged speed isn't available yet
    if (abs_speed < 0.5f && abs_instant_speed > 0.5f)
    {
        abs_speed = abs_instant_speed;
    }

    // MOVING OBJECTS - Clear human movement
    if (abs_speed >= HUMAN_TYPICAL_SPEED_MIN && abs_speed < MAX_SPEED_CM_S)
    {
        return MMWAVE_OBJECT_HUMAN;
    }

    // Filter unrealistic speeds as noise
    if (abs_speed >= MAX_SPEED_CM_S)
    {
        return MMWAVE_OBJECT_NOISE;
    }

    // SLOW MOVEMENT - Could be human moving slowly
    if (abs_speed > 3.0f && abs_speed < HUMAN_TYPICAL_SPEED_MIN)
    {
        // Slightly moving - likely human
        return MMWAVE_OBJECT_HUMAN;
    }

    // STATIONARY OR VERY SLOW - Need more analysis
    // Wait for sufficient history before classifying stationary objects
    if (history_count < 2)
    {
        return MMWAVE_OBJECT_UNKNOWN; // Need more data
    }

    // Calculate position and distance stability
    float x_sum = target->x;
    float y_sum = target->y;
    float x_sum_sq = target->x * target->x;
    float y_sum_sq = target->y * target->y;
    float dist_sum = target->distance_cm;
    float dist_sum_sq = target->distance_cm * target->distance_cm;
    int count = 1;

    // Analyze recent history (up to 4 previous frames)
    int history_depth = (history_count < 4) ? history_count : 4;
    for (int i = 1; i < history_depth; i++)
    {
        int hist_idx = (history_index - i + 5) % 5;
        if (target_history[hist_idx].distance_cm > 0)
        {
            x_sum += target_history[hist_idx].x;
            y_sum += target_history[hist_idx].y;
            x_sum_sq += target_history[hist_idx].x * target_history[hist_idx].x;
            y_sum_sq += target_history[hist_idx].y * target_history[hist_idx].y;
            dist_sum += target_history[hist_idx].distance_cm;
            dist_sum_sq += target_history[hist_idx].distance_cm * target_history[hist_idx].distance_cm;
            count++;
        }
    }

    // Calculate means and standard deviations
    float x_mean = x_sum / count;
    float y_mean = y_sum / count;
    float dist_mean = dist_sum / count;

    float x_variance = (x_sum_sq / count) - (x_mean * x_mean);
    float y_variance = (y_sum_sq / count) - (y_mean * y_mean);
    float dist_variance = (dist_sum_sq / count) - (dist_mean * dist_mean);

    target->position_stability = sqrtf(fmaxf(0.0f, x_variance + y_variance));
    target->distance_stability = sqrtf(fmaxf(0.0f, dist_variance));

    // Classification based on stability and distance
    bool very_stable = (target->distance_stability < 3.0f && target->position_stability < 5.0f);
    bool moderately_stable = (target->distance_stability < 8.0f && target->position_stability < 15.0f);
    bool very_stationary = (abs_speed < 2.0f);
    bool mostly_stationary = (abs_speed < 5.0f);

    // WALL: Far away, very stable, and stationary
    if (target->distance_cm > 100.0f)
    {
        if (very_stable && very_stationary)
        {
            return MMWAVE_OBJECT_WALL;
        }
        else if (moderately_stable && mostly_stationary)
        {
            return MMWAVE_OBJECT_WALL;
        }
        // Far but unstable - could be human walking away
        else if (abs_speed > 1.0f)
        {
            return MMWAVE_OBJECT_HUMAN;
        }
    }

    // FURNITURE: Medium distance, stable
    if (target->distance_cm > 50.0f && target->distance_cm <= 100.0f)
    {
        if (very_stable && very_stationary)
        {
            return MMWAVE_OBJECT_FURNITURE;
        }
        else if (moderately_stable && mostly_stationary)
        {
            return MMWAVE_OBJECT_FURNITURE;
        }
        // Medium distance but moving - human
        else if (abs_speed > 1.0f)
        {
            return MMWAVE_OBJECT_HUMAN;
        }
    }

    // SMALL OBJECTS: Close range, stable
    if (target->distance_cm <= 50.0f)
    {
        if (very_stable && very_stationary)
        {
            return MMWAVE_OBJECT_SMALL_ITEM;
        }
        else if (moderately_stable && mostly_stationary)
        {
            return MMWAVE_OBJECT_SMALL_ITEM;
        }
        // Close and moving - likely human
        else if (abs_speed > 1.0f)
        {
            return MMWAVE_OBJECT_HUMAN;
        }
    }

    // Default to stationary if we can't classify confidently
    return MMWAVE_OBJECT_STATIONARY;
}

// Initialize mmWave radar
void mmwave_init(void)
{
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    start_time_ms = to_ms_since_boot(get_absolute_time());

    printf("[MMWAVE] Initialized on UART1, pins GP%d/GP%d at %d baud\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
}

// Process incoming mmWave data (non-blocking)
bool mmwave_process_data(void)
{
    bool data_processed = false;

    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);
        total_bytes_received++;
        data_processed = true;

        last_marker = (last_marker << 8) | ch;
        last_marker &= 0xFFFF;

        buf[idx++] = ch;

        // Check for frame markers
        if (last_marker == MARKER_FE_F8 || last_marker == MARKER_FD_F8 ||
            last_marker == MARKER_FE_FC || last_marker == MARKER_FD_FC)
        {
            // Process previous frame if we have one
            if (in_frame && idx > frame_start_idx + 2)
            {
                uint16_t frame_len = (idx - 2) - frame_start_idx;
                uint16_t frame_end = idx - 2;

                if (frame_len >= 6)
                {
                    frame_count++;
                    bool has_detection = false;
                    mmwave_target_info_t target = {0};
                    target.frame_number = frame_count;
                    target.timestamp_ms = to_ms_since_boot(get_absolute_time()) - start_time_ms;

                    // Method 1: Look for distance measurements
                    for (uint16_t i = frame_start_idx + 2; i <= frame_end - 4; i++)
                    {
                        uint16_t pattern = buf[i + 2] | (buf[i + 3] << 8);

                        if (pattern == 0x6810 || pattern == 0x6813 || pattern == 0x6C10 ||
                            pattern == 0x9013 || pattern == 0x1410)
                        {
                            uint16_t dist_raw = buf[i] | (buf[i + 1] << 8);
                            float dist_cm = dist_raw / 10.0f;

                            if (dist_cm >= MIN_DISTANCE_CM && dist_cm <= MAX_DISTANCE_CM)
                            {
                                target.distance_cm = dist_cm;
                                has_detection = true;

                                // Try to extract X/Y coordinates
                                for (uint16_t j = frame_start_idx; j <= frame_end - 5; j++)
                                {
                                    if (buf[j] == 0x04 && buf[j + 3] == 0x20 && buf[j + 4] == 0x08)
                                    {
                                        int8_t x_byte = (int8_t)buf[j + 1];
                                        int8_t y_byte = (int8_t)buf[j + 2];

                                        if (abs(x_byte) < 200 && abs(y_byte) < 200)
                                        {
                                            target.x = x_byte;
                                            target.y = y_byte;
                                            break;
                                        }
                                    }
                                }
                                break;
                            }
                        }
                    }

                    // Method 2: Alternative pattern matching
                    if (!has_detection && frame_len >= 8)
                    {
                        for (uint16_t i = frame_start_idx; i <= frame_end - 8; i++)
                        {
                            if (buf[i] == 0x04 &&
                                (buf[i + 1] == 0x62 || buf[i + 1] == 0x42 ||
                                 buf[i + 1] == 0xE0 || buf[i + 1] == 0xE2) &&
                                buf[i + 2] == 0x20 && buf[i + 3] == 0x00 && buf[i + 4] == 0x08)
                            {
                                if (i >= frame_start_idx + 2)
                                {
                                    uint16_t dist_raw = buf[i - 2] | (buf[i - 1] << 8);
                                    float dist_cm = dist_raw / 10.0f;

                                    if (dist_cm >= MIN_DISTANCE_CM && dist_cm <= MAX_DISTANCE_CM)
                                    {
                                        target.distance_cm = dist_cm;
                                        has_detection = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    // Process detection
                    if (has_detection && target.distance_cm > 0)
                    {
                        // Calculate angle from X/Y if available
                        if (target.x != 0 || target.y != 0)
                        {
                            target.angle_deg = calculate_angle(target.x, target.y);
                        }

                        // Calculate speed and stability metrics
                        if (has_history)
                        {
                            mmwave_target_info_t *last = &target_history[(history_index - 1 + 5) % 5];

                            // Speed calculation
                            float dist_change = target.distance_cm - last->distance_cm;
                            uint32_t time_diff_ms = target.timestamp_ms - last->timestamp_ms;

                            if (time_diff_ms >= 50 && time_diff_ms < 2000)
                            {
                                float instant_speed = (dist_change / (time_diff_ms / 1000.0f));

                                // Filter noise and unrealistic speeds
                                if (fabsf(dist_change) < 0.5f || fabsf(instant_speed) > MAX_SPEED_CM_S)
                                {
                                    target.speed_cm_s = 0.0f;
                                }
                                else
                                {
                                    target.speed_cm_s = instant_speed;
                                }
                            }
                            else
                            {
                                target.speed_cm_s = 0.0f;
                            }

                            // Calculate averaged speed
                            float speed_sum = 0.0f;
                            int speed_count = 0;

                            if (fabsf(target.speed_cm_s) < MAX_SPEED_CM_S)
                            {
                                speed_sum += target.speed_cm_s;
                                speed_count++;
                            }

                            for (int i = 1; i < 5 && i <= history_index; i++)
                            {
                                int idx = (history_index - i + 5) % 5;
                                if (target_history[idx].distance_cm > 0 &&
                                    fabsf(target_history[idx].speed_cm_s) < MAX_SPEED_CM_S)
                                {
                                    speed_sum += target_history[idx].speed_cm_s;
                                    speed_count++;
                                }
                            }

                            target.avg_speed_cm_s = (speed_count > 0) ? (speed_sum / speed_count) : 0.0f;

                            // Calculate distance stability (standard deviation)
                            float dist_sum = target.distance_cm;
                            float dist_sum_sq = target.distance_cm * target.distance_cm;
                            int stab_count = 1;

                            for (int i = 1; i < 4 && i <= history_index; i++)
                            {
                                int idx = (history_index - i + 5) % 5;
                                if (target_history[idx].distance_cm > 0)
                                {
                                    dist_sum += target_history[idx].distance_cm;
                                    dist_sum_sq += target_history[idx].distance_cm *
                                                   target_history[idx].distance_cm;
                                    stab_count++;
                                }
                            }

                            float dist_mean = dist_sum / stab_count;
                            float dist_variance = (dist_sum_sq / stab_count) - (dist_mean * dist_mean);
                            target.distance_stability = sqrtf(fmaxf(0.0f, dist_variance));
                        }
                        else
                        {
                            // No history yet
                            target.speed_cm_s = 0.0f;
                            target.avg_speed_cm_s = 0.0f;
                            target.distance_stability = 0.0f;
                        }

                        // Classify the object
                        target.object_type = classify_object(&target, target_history, history_index);

                        // Update detection persistence (how many times we've seen this object)
                        target.detection_count = 1;
                        if (has_history)
                        {
                            // Check if this matches a recent detection (same object)
                            for (int i = 1; i <= 3 && i <= history_index; i++)
                            {
                                int idx = (history_index - i + 5) % 5;
                                mmwave_target_info_t *prev = &target_history[idx];

                                if (prev->distance_cm <= 0)
                                    continue;

                                float dist_diff = fabsf(target.distance_cm - prev->distance_cm);
                                float pos_diff = sqrtf(powf(target.x - prev->x, 2) +
                                                       powf(target.y - prev->y, 2));

                                // Same object if within thresholds
                                if (dist_diff < 15.0f && pos_diff < 25.0f)
                                {
                                    target.detection_count = prev->detection_count + 1;
                                    break;
                                }
                            }
                        }

                        // Map all non-human types to STATIONARY
                        if (target.object_type == MMWAVE_OBJECT_NOISE ||
                            target.object_type == MMWAVE_OBJECT_WALL ||
                            target.object_type == MMWAVE_OBJECT_FURNITURE ||
                            target.object_type == MMWAVE_OBJECT_SMALL_ITEM ||
                            target.object_type == MMWAVE_OBJECT_UNKNOWN)
                        {
                            target.object_type = MMWAVE_OBJECT_STATIONARY;
                        }
                        // Apply runtime range limit if set
                        if (max_range_cm > 0 && target.distance_cm > max_range_cm)
                        {
                            target.object_type = MMWAVE_OBJECT_STATIONARY;
                        }
                        // Report detections
                        if (true)
                        {
                            // Estimate signal strength (inverse of distance)
                            target.signal_strength = 1000.0f / (target.distance_cm + 10.0f);

                            global_detection_count++;

                            // Update history
                            target_history[history_index] = target;
                            history_index = (history_index + 1) % 5;
                            if (!has_history)
                                has_history = true;

                            // Store latest detection
                            latest_target = target;
                            new_detection_available = true;
                            detection_count++;
                            last_detection_time = to_ms_since_boot(get_absolute_time());
                        }
                    }
                }
            }

            // Start new frame
            frame_start_idx = idx - 2;
            in_frame = true;
        }

        // Reset buffer if getting full
        if (idx >= BUF_LEN - 10)
        {
            if (in_frame && frame_start_idx > BUF_LEN - 100)
            {
                uint16_t shift = BUF_LEN - 100;
                memmove(buf, buf + shift, 100);
                idx = 100;
                frame_start_idx = 100 - (BUF_LEN - frame_start_idx);
                last_marker = (buf[idx - 2] << 8) | buf[idx - 1];
            }
            else if (!in_frame)
            {
                memmove(buf, buf + BUF_LEN - 10, 10);
                idx = 10;
                last_marker = (buf[8] << 8) | buf[9];
            }
        }
    }

    return data_processed;
}

// Get the latest detection (if available)
bool mmwave_get_latest_detection(mmwave_target_info_t *target)
{
    if (!new_detection_available || !target)
        return false;

    *target = latest_target;
    new_detection_available = false;
    return true;
}

// Get object type as string
const char *mmwave_object_type_string(mmwave_object_type_t type)
{
    switch (type)
    {
    case MMWAVE_OBJECT_HUMAN:
        return "Human";
    case MMWAVE_OBJECT_STATIONARY:
        return "Stationary";
    case MMWAVE_OBJECT_WALL:
        return "Wall";
    case MMWAVE_OBJECT_SMALL_ITEM:
        return "Small Object";
    case MMWAVE_OBJECT_FURNITURE:
        return "Furniture";
    case MMWAVE_OBJECT_NOISE:
        return "Noise";
    default:
        return "Unknown";
    }
}

// Get total bytes received (for debugging)
uint32_t mmwave_get_total_bytes(void)
{
    return total_bytes_received;
}

// Get frame count (for debugging)
uint32_t mmwave_get_frame_count(void)
{
    return frame_count;
}

// Set maximum detection range at runtime
void mmwave_set_max_range_cm(uint16_t max_cm)
{
    max_range_cm = max_cm;
}