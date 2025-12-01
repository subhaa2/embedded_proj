
// sensors.c (Reads and parses data from the log file)
#include "defs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
static FILE *data_file = NULL;
static int read_cycle_count = 0;
// Global variable definition (assuming it was missing in the original sensors.c)
extern SensorData current_sim_data;
void Sensors_init(const char* filename) {
    data_file = fopen(filename, "r");
    if (data_file == NULL) {
        perror("Error opening sensor data file");
        exit(1);
    }
    read_cycle_count = 0;
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
 * @brief Read the next sensor data entry from the log file (mmWave event-driven format).
 */
bool Sensors_get_next_data() {
    char line[512];
    float dist;
    
    // Reset status before reading to avoid stale data propagation
    current_sim_data.status = UNKNOWN_STATUS; 
    current_sim_data.distance_m = 0.0f;
    current_sim_data.temperature_c = 0.0f;
    while (fgets(line, sizeof(line), data_file) != NULL) {
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
        
        // 2. --- OBJECT DETECTION (Start of Block) ---
        if (strstr(line, "[mmWave Detection]")) { 
            
            // Map the string type to EnvironmentStatus based on user's requirements
            if (strstr(line, "Stationary")) {
                current_sim_data.status = WALL; // WALL now also represents FORK/JUNCTION
            } else if (strstr(line, "Small Object")) {
                current_sim_data.status = STRAFE_OBJECT;
            } else if (strstr(line, "Goal")) {
                current_sim_data.status = GOAL_OBJECT;
            } else if (strstr(line, "Human")) {
                current_sim_data.status = POSSIBLE_HUMAN;
                current_sim_data.distance_m = 0.0f;
                read_cycle_count++;
            return true;
            } else {
                current_sim_data.status = UNKNOWN_STATUS; 
            }
            // Look ahead for the Distance line in the next line(s)
            while (fgets(line, sizeof(line), data_file) != NULL) {
                if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
                    continue;
                }
                
                // Format: Distance: XX.X cm (Y.YY m)
                int dist_match = sscanf(line, " Distance: %*f cm (%f m)", &dist);
                if (dist_match == 1) { 
                    current_sim_data.distance_m = dist;
                    read_cycle_count++;
                    return true;
                }
            }
            
            break; // EOF reached while looking for Distance
        }
    }
}
