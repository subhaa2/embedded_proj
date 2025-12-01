#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

// UART configuration
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
#define BLIND_ZONE_CM 25         // Blind zone - ignore detections closer than this (typical 20-50cm for mmWave)
#define MIN_DISTANCE_CM 30       // Minimum valid detection distance (above blind zone)
#define MAX_DISTANCE_CM 150      // Maximum detection distance
#define MIN_DISTANCE_CHANGE_CM 2 // Minimum distance change to calculate speed
#define MAX_SPEED_CM_S 500       // Maximum realistic speed (filter noise)
#define FRAME_RATE_HZ 10         // Approximate radar frame rate (for speed calculation)

// Object classification thresholds
#define STATIONARY_SPEED_THRESHOLD 15.0f // cm/s - below this is considered stationary (increased from 10.0f)
#define HUMAN_TYPICAL_SPEED_MIN 10.0f    // cm/s - humans typically move at least this fast (lowered from 20.0f)

// Object types
typedef enum
{
    OBJECT_UNKNOWN,
    OBJECT_HUMAN,
    OBJECT_STATIONARY, // Wall, furniture, etc.
    OBJECT_NOISE       // Likely false detection
} object_type_t;

// Structure to store target information
typedef struct
{
    float distance_cm;
    int16_t x;
    int16_t y;
    float angle_deg;
    float speed_cm_s;     // Positive = moving away, Negative = moving closer
    float avg_speed_cm_s; // Averaged speed over recent frames
    object_type_t object_type;
    uint32_t frame_number;
    uint32_t timestamp_ms;
    float distance_stability; // How stable the distance is (for object classification)
} target_info_t;

// Calculate angle from X/Y coordinates (in degrees)
float calculate_angle(int16_t x, int16_t y)
{
    if (x == 0 && y == 0)
        return 0.0f;
    float angle_rad = atan2f((float)y, (float)x);
    float angle_deg = angle_rad * 180.0f / M_PI;
    return angle_deg;
}

// Classify object based on motion characteristics
object_type_t classify_object(target_info_t *target, target_info_t *prev_targets, int history_count)
{
    // Very close detections are likely noise
    if (target->distance_cm < BLIND_ZONE_CM)
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
        // If distance is stable, it's stationary
        if (target->distance_stability < 10.0f || history_count < 2)
        {
            // For objects with very low speed variation, classify as stationary
            return OBJECT_STATIONARY;
        }
        // Otherwise might be slow-moving human
        if (abs_speed > 2.0f)
        {
            // Some movement detected but slow - could be human moving slowly
            return OBJECT_HUMAN;
        }
        return OBJECT_STATIONARY;
    }

    // High speed: likely human (or noise if too high)
    if (abs_speed >= HUMAN_TYPICAL_SPEED_MIN)
    {
        if (abs_speed >= MAX_SPEED_CM_S)
        {
            // Unrealistically high speed - likely noise
            return OBJECT_NOISE;
        }
        // Reasonable speed range for human movement
        return OBJECT_HUMAN;
    }

    // Between thresholds - use distance stability as tiebreaker
    if (history_count >= 2)
    {
        if (target->distance_stability < 5.0f)
        {
            return OBJECT_STATIONARY; // Stable = stationary
        }
        else
        {
            return OBJECT_HUMAN; // Variable = moving object (likely human)
        }
    }

    // Default to human if we're not sure (better to detect than miss)
    return OBJECT_HUMAN;
}

// Get object type as string
const char *object_type_string(object_type_t type)
{
    switch (type)
    {
    case OBJECT_HUMAN:
        return "Human";
    case OBJECT_STATIONARY:
        return "Stationary (Wall/Furniture)";
    case OBJECT_NOISE:
        return "Noise/False Detection";
    default:
        return "Unknown";
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    printf("=== mmWave Radar Human Detection ===\n");
    printf("Detection range: %.0f - %.0f cm (blind zone: %.0f cm)\n",
           MIN_DISTANCE_CM, MAX_DISTANCE_CM, BLIND_ZONE_CM);
    printf("Frame rate: ~%d Hz (estimated)\n", FRAME_RATE_HZ);
    printf("Monitoring for targets...\n\n");

    uint8_t buf[BUF_LEN];
    uint16_t idx = 0;
    uint32_t frame_count = 0;
    uint32_t detection_count = 0;

    // Track previous detections for speed calculation and filtering
    target_info_t target_history[5] = {0}; // Keep last 5 detections
    int history_index = 0;
    bool has_history = false;

    uint16_t last_marker = 0;
    uint16_t frame_start_idx = 0;
    bool in_frame = false;

    uint32_t start_time_ms = to_ms_since_boot(get_absolute_time());

    while (true)
    {
        if (uart_is_readable(UART_ID))
        {
            uint8_t ch = uart_getc(UART_ID);

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
                        target_info_t target = {0};
                        target.frame_number = frame_count;
                        target.timestamp_ms = to_ms_since_boot(get_absolute_time()) - start_time_ms;

                        // Method 1: Look for distance measurements with patterns like "XX XX 10 68"
                        for (uint16_t i = frame_start_idx + 2; i <= frame_end - 4; i++)
                        {
                            uint16_t pattern = buf[i + 2] | (buf[i + 3] << 8);

                            if (pattern == 0x6810 || pattern == 0x6813 || pattern == 0x6C10 ||
                                pattern == 0x9013 || pattern == 0x1410)
                            {

                                uint16_t dist_raw = buf[i] | (buf[i + 1] << 8);
                                float dist_cm = dist_raw / 10.0f;

                                // Apply distance thresholds (ignore blind zone)
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
                            // Calculate angle
                            if (target.x != 0 || target.y != 0)
                            {
                                target.angle_deg = calculate_angle(target.x, target.y);
                            }

                            // Calculate speed from previous detection
                            if (has_history)
                            {
                                target_info_t *last = &target_history[(history_index - 1 + 5) % 5];

                                float dist_change = target.distance_cm - last->distance_cm;
                                uint32_t time_diff_ms = target.timestamp_ms - last->timestamp_ms;

                                if (time_diff_ms > 0)
                                {
                                    // Calculate speed: positive = moving away, negative = moving closer
                                    target.speed_cm_s = (dist_change / (time_diff_ms / 1000.0f));

                                    // Filter unrealistic speeds
                                    if (fabsf(target.speed_cm_s) > MAX_SPEED_CM_S)
                                    {
                                        target.speed_cm_s = 0.0f; // Likely noise
                                    }
                                }

                                // Calculate average speed over recent frames
                                float speed_sum = target.speed_cm_s;
                                int count = 1;
                                for (int i = 1; i < 3 && i < history_index; i++)
                                {
                                    int idx = (history_index - 1 - i + 5) % 5;
                                    if (target_history[idx].distance_cm > 0)
                                    {
                                        speed_sum += target_history[idx].speed_cm_s;
                                        count++;
                                    }
                                }
                                target.avg_speed_cm_s = speed_sum / count;

                                // Calculate distance stability (standard deviation estimate)
                                float dist_sum = target.distance_cm;
                                float dist_sum_sq = target.distance_cm * target.distance_cm;
                                for (int i = 1; i < 3 && i < history_index; i++)
                                {
                                    int idx = (history_index - 1 - i + 5) % 5;
                                    if (target_history[idx].distance_cm > 0)
                                    {
                                        dist_sum += target_history[idx].distance_cm;
                                        dist_sum_sq += target_history[idx].distance_cm *
                                                       target_history[idx].distance_cm;
                                        count++;
                                    }
                                }
                                float mean = dist_sum / count;
                                float variance = (dist_sum_sq / count) - (mean * mean);
                                target.distance_stability = sqrtf(variance);
                            }

                            // Classify object
                            target.object_type = classify_object(&target, target_history, history_index);

                            // Only report valid detections (filter noise)
                            if (target.object_type != OBJECT_NOISE)
                            {
                                // Update history
                                target_history[history_index] = target;
                                history_index = (history_index + 1) % 5;
                                if (!has_history)
                                    has_history = true;

                                // Report detection
                                detection_count++;
                                printf("[Detection #%lu] %s\n", detection_count,
                                       object_type_string(target.object_type));
                                printf("  Distance: %.1f cm (%.2f m)\n",
                                       target.distance_cm, target.distance_cm / 100.0f);

                                if (target.x != 0 || target.y != 0)
                                {
                                    printf("  Position: X=%d, Y=%d | Angle: %.1f°\n",
                                           target.x, target.y, target.angle_deg);
                                }

                                if (fabsf(target.speed_cm_s) > 1.0f)
                                {
                                    const char *direction = target.speed_cm_s > 0 ? "away" : "closer";
                                    printf("  Speed: %.1f cm/s (%.2f m/s) - moving %s\n",
                                           fabsf(target.speed_cm_s),
                                           fabsf(target.speed_cm_s) / 100.0f,
                                           direction);
                                }
                                else
                                {
                                    printf("  Speed: < 1 cm/s - stationary\n");
                                }

                                printf("  Frame #%lu | Time: %.1f s\n",
                                       target.frame_number, target.timestamp_ms / 1000.0f);
                                printf("\n");
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
        else
        {
            sleep_us(100);
        }
    }
}
