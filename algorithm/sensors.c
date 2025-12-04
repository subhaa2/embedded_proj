// sensors.c - Sensor data parsing from stdin for simulation

#include "defs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static int read_cycle_count = 0;
extern SensorData current_sim_data;

// Initialize sensor reading system
void Sensors_init(const char* filename) {
    read_cycle_count = 0;
    printf("[SENSORS] Initialized - reading from serial monitor input\n");
}

// Calculate shortest angular distance between current and target heading
// Returns positive for LEFT turn, negative for RIGHT turn
float Sensors_get_angle_to_turn(float current_heading, float target_heading) {
    float diff = target_heading - current_heading;
    
    // Normalize to -360 to 360 range
    while (diff >= 360.0f) {
        diff -= 360.0f;
    }
    while (diff < -360.0f) {
        diff += 360.0f;
    }
    
    // Find shortest path (-180 to 180 range)
    if (diff > 180.0f) {
        diff -= 360.0f;
    } 
    else if (diff <= -180.0f) {
        diff += 360.0f;
    }
    
    return diff;
}

// Read next sensor data entry from stdin
bool Sensors_get_next_data() {
    char line[512];
    float dist;
    
    // Reset status to avoid stale data
    current_sim_data.status = UNKNOWN_STATUS; 
    current_sim_data.distance_m = 0.0f;
    current_sim_data.temperature_c = 0.0f;
    
    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        
        // CLEAR PATH DETECTION
        if (strstr(line, "[CLEAR_PATH]")) {
            current_sim_data.status = CLEAR_PATH;
            current_sim_data.distance_m = 0.0f;
            read_cycle_count++;
            return true;
        }
        
        // EMPTY SPACE DETECTION (during scans)
        if (strstr(line, "[EMPTY_SPACE_DETECTED]")) {
            current_sim_data.status = CLEAR_PATH;
            current_sim_data.distance_m = 0.0f;
            read_cycle_count++;
            return true;
        }
        
        // HUMAN CONFIRMED
        if (strstr(line, "Human Confirmed")) {
            current_sim_data.status = HUMAN_CONFIRMED;
            // Parse temperature from format: "temperature XX degrees"
            float temp;
            if (sscanf(line, "%*[^t]temperature %f degrees", &temp) == 1) {
                current_sim_data.temperature_c = temp;
            }
            read_cycle_count++;
            return true;
        }
        
        // RADAR DETECTION
        if (strstr(line, "[RADAR")) {
            char type[64];
            int distance_cm;
            
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
    
    return false;
}
