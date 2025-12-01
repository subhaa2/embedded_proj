#ifndef MMWAVE_DETECT_H
#define MMWAVE_DETECT_H

#include <stdint.h>
#include <stdbool.h>

// Object types
typedef enum
{
    MMWAVE_OBJECT_UNKNOWN,
    MMWAVE_OBJECT_HUMAN,
    MMWAVE_OBJECT_WALL,       // Large flat surface (wall, door)
    MMWAVE_OBJECT_SMALL_ITEM, // Small objects (bottle, cup, etc.)
    MMWAVE_OBJECT_FURNITURE,  // Medium objects (chair, table, etc.)
    MMWAVE_OBJECT_NOISE       // Likely false detection
} mmwave_object_type_t;

// Structure to store target information
typedef struct
{
    float distance_cm;
    int16_t x;
    int16_t y;
    float angle_deg;
    float speed_cm_s;     // Positive = moving away, Negative = moving closer
    float avg_speed_cm_s; // Averaged speed over recent frames
    mmwave_object_type_t object_type;
    uint32_t frame_number;
    uint32_t timestamp_ms;
    float distance_stability; // How stable the distance is (for object classification)
    float position_stability; // How stable the X/Y position is
    uint32_t detection_count; // How many times this object has been detected
    float signal_strength;    // Relative signal strength (for size estimation)
} mmwave_target_info_t;

// Function prototypes
void mmwave_init(void);
bool mmwave_process_data(void);
bool mmwave_get_latest_detection(mmwave_target_info_t *target);
const char *mmwave_object_type_string(mmwave_object_type_t type);
uint32_t mmwave_get_total_bytes(void);
uint32_t mmwave_get_frame_count(void);

#endif // MMWAVE_DETECT_H
