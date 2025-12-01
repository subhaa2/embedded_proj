
// sensors.c (Reads and parses data from serial monitor / stdin)
#include "defs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
static int read_cycle_count = 0;
// Global variable definition (assuming it was missing in the original sensors.c)
extern SensorData current_sim_data;
void Sensors_init(const char* filename) {
    // No file needed - reading from stdin (serial monitor)
    read_cycle_count = 0;
    printf("[SENSORS] Initialized - reading from serial monitor input\n");
}
// sensors.c (Add to the end of the file)
/**
 * @brief Calculates the shortest angular distance and direction between the current
 * heading and the target heading.
 * * @param current_heading The robot's current orientation in degrees (0.0 to 359.9).
 * @param target_heading The desired orientation in degrees (0.0 to 359.9).
 * @return float The angle difference. Positive means turn LEFT, Negative means turn RIGHT.
 */
float Sensors_get_angle_to_turn(float current_heading, float target_heading) {
    // 1. Calculate the initial difference
    float diff = target_heading - current_heading;
    // 2. Normalize the difference to be between -360.0 and 360.0
    while (diff >= 360.0f) {
        diff -= 360.0f;
    }
    while (diff < -360.0f) {
        diff += 360.0f;
    }
    // 3. Find the shortest path (difference should be between -180.0 and 180.0)
    // If diff is > 180 (e.g., 270), turning -90 (right) is shorter.
    if (diff > 180.0f) {
        diff -= 360.0f;
    } 
    // If diff is < -180 (e.g., -270), turning +90 (left) is shorter.
    else if (diff <= -180.0f) {
        diff += 360.0f;
    }
    // Result interpretation:
    // Positive 'diff': Turn LEFT (e.g., +90 deg)
    // Negative 'diff': Turn RIGHT (e.g., -90 deg)
    // Zero 'diff': No turn needed (aligned)
    return diff;
}
/**
 * @brief Read the next sensor data entry from serial monitor input (stdin).
 */
bool Sensors_get_next_data() {
    char line[512];
    float dist;
    
    // Reset status before reading to avoid stale data propagation
    current_sim_data.status = UNKNOWN_STATUS; 
    current_sim_data.distance_m = 0.0f;
    current_sim_data.temperature_c = 0.0f;
    
    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        
        // 1. --- CLEAR PATH DETECTION ---
        if (strstr(line, "[CLEAR_PATH]")) {
            current_sim_data.status = CLEAR_PATH;
            current_sim_data.distance_m = 0.0f;
            read_cycle_count++;
            return true;
        }
        
        // 1b. --- EMPTY SPACE DETECTION (during scans) ---
        if (strstr(line, "[EMPTY_SPACE_DETECTED]")) {
            current_sim_data.status = CLEAR_PATH;
            current_sim_data.distance_m = 0.0f;
            read_cycle_count++;
            return true;
        }
        
        // 1d. --- HUMAN CONFIRMED ---
        if (strstr(line, "Human Confirmed")) {
            current_sim_data.status = HUMAN_CONFIRMED;
            // Look for temperature in format: "temperature XX degrees"
            float temp;
            if (sscanf(line, "%*[^t]temperature %f degrees", &temp) == 1) {
                current_sim_data.temperature_c = temp;
            }
            read_cycle_count++;
            return true;
        }
        
        // 2. --- RADAR DETECTION (New Format) ---
        // Format: [RADAR,Type,Distance_cm,param3,param4,param5,param6,param7]
        if (strstr(line, "[RADAR")) {
            char type[64];
            int distance_cm;
            
            // Parse the CSV format: [RADAR,Type,Distance,...]
            if (sscanf(line, "[RADAR,%63[^,],%d", type, &distance_cm) == 2) {
                // Convert distance from cm to meters
                current_sim_data.distance_m = distance_cm / 100.0f;
                
                // Map type to status
                if (strstr(type, "Stationary")) {
                    current_sim_data.status = WALL;
                } else if (strstr(type, "Small Object")) {
                    current_sim_data.status = STRAFE_OBJECT;
                } else if (strstr(type, "Goal")) {
                    current_sim_data.status = GOAL_OBJECT;
                } else if (strstr(type, "Human")) {
                    current_sim_data.status = POSSIBLE_HUMAN;
                } else {
                    current_sim_data.status = UNKNOWN_STATUS;
                }
                
                read_cycle_count++;
                return true;
            }
        }
    }
    
    // If we reach here, stdin has no more data
    return false;
}

/**
 * @brief Parse sensor data from a string (for embedded LoRa integration)
 * @param input CSV format string: [RADAR,Type,Distance_cm,...] or special messages
 * @return SensorData structure with parsed values
 */
SensorData Sensors_parse_radar_string(const char* input) {
    SensorData data;
    data.status = UNKNOWN_STATUS;
    data.distance_m = 0.0f;
    data.temperature_c = 0.0f;
    
    if (input == NULL || strlen(input) == 0) {
        return data;
    }
    
    // Make a copy to work with
    char line[512];
    strncpy(line, input, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    
    // 1. --- CLEAR PATH DETECTION ---
    if (strstr(line, "[CLEAR_PATH]")) {
        data.status = CLEAR_PATH;
        data.distance_m = 0.0f;
        return data;
    }
    
    // 1b. --- EMPTY SPACE DETECTION (during scans) ---
    if (strstr(line, "[EMPTY_SPACE_DETECTED]")) {
        data.status = CLEAR_PATH;
        data.distance_m = 0.0f;
        return data;
    }
    
    // 1d. --- HUMAN CONFIRMED ---
    if (strstr(line, "Human Confirmed")) {
        data.status = HUMAN_CONFIRMED;
        // Look for temperature in format: "temperature XX degrees"
        float temp;
        if (sscanf(line, "%*[^t]temperature %f degrees", &temp) == 1) {
            data.temperature_c = temp;
        }
        return data;
    }
    
    // 2. --- RADAR DETECTION (CSV Format) ---
    // Format: [RADAR,Type,Distance_cm,param3,param4,param5,param6,param7]
    if (strstr(line, "[RADAR")) {
        char type[64];
        int distance_cm;
        
        // Parse the CSV format: [RADAR,Type,Distance,...]
        if (sscanf(line, "[RADAR,%63[^,],%d", type, &distance_cm) == 2) {
            // Convert distance from cm to meters
            data.distance_m = distance_cm / 100.0f;
            
            // Map type to status
            if (strstr(type, "Stationary")) {
                data.status = WALL;
            } else if (strstr(type, "Small Object")) {
                data.status = STRAFE_OBJECT;
            } else if (strstr(type, "Goal")) {
                data.status = GOAL_OBJECT;
            } else if (strstr(type, "Human")) {
                data.status = POSSIBLE_HUMAN;
            } else {
                data.status = UNKNOWN_STATUS;
            }
            
            return data;
        }
    }
    
    return data;
}
