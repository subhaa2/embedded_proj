
// control.c - simulation of actuation and logging
#define _USE_MATH_DEFINES
#include "defs.h"
#include <stdio.h>
#include <math.h>
#include "main.h"

// --- Spider Command Definitions ---
#define SPIDER_90DEGREE -1
#define SPIDER_STANDBY 0
#define SPIDER_MOVE_FORWARD 1
#define SPIDER_MOVE_BACKWARD 2
#define SPIDER_MOVE_LEFT 3
#define SPIDER_MOVE_RIGHT 4
#define SPIDER_ROTATE_LEFT 5
#define SPIDER_ROTATE_RIGHT 6
// Custom gait for longer-distance stationary inputs
#define SPIDER_CUSTOM_WALK 7
#define SPIDER_TEMPERATURE_WALK 8

// --- Movement Constants ---
#define MOVE_DISTANCE 0.5f  // Distance moved for WALK and STRAFE (in meters)
#define TURN_ANGLE 90.0f    // Default turn angle for LEFT/RIGHT (in degrees)
#define BACKTRACK_DISTANCE -0.5f // Distance for backtracking (moves opposite to current heading)
// Global variables (extern declarations in defs.h)
extern Pose current_pose;
extern SensorData current_sim_data;
extern int scan_steps; 
// Assumed utility functions
static float deg_to_rad(float deg) {
    return deg * M_PI / 180.0f;
}
static float normalize_heading(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

/**
 * @brief Logs the current simulated data from the mmWave sensor and pose.
 * * NOTE: The old FFT logging is removed.
 */
void Control_log_data() {
    const char* status_str;
    switch (current_sim_data.status) {
        case CLEAR_PATH: 
            // Check if distance is 0 - might indicate empty space detection during scan
            if (current_sim_data.distance_m == 0.0f) {
                status_str = "EMPTY_SPACE_DETECTED";
            } else {
                status_str = "CLEAR_PATH";
            }
            break;
        case WALL: status_str = "WALL (Stationary)"; break;
        case STRAFE_OBJECT: status_str = "STRAFE_OBJECT (Small Obj)"; break;
        case STATIONARY: status_str = "STATIONARY"; break;
        
        case GOAL_OBJECT: status_str = "GOAL_OBJECT (Goal)"; break;
        case FORK: status_str = "FORK (Internal)"; break;
        case POSSIBLE_HUMAN: status_str = "POSSIBLE_HUMAN"; break;
        case HUMAN_CONFIRMED: status_str = "HUMAN_CONFIRMED"; break;
        default: status_str = "UNKNOWN"; break;
    }
    printf("--- CYCLE DATA --- Status: %s | Dist: %.2f m | Pose (X:%.1f, Y:%.1f, H:%.1f)\n", 
           status_str, 
           current_sim_data.distance_m,
           current_pose.x, current_pose.y, current_pose.orientation_deg);
}
// --- Control System Function ---
/**
 * @brief Simulates the execution of a single robot command.
 * * @param command The RobotCommand to execute.
 * @return The command that was executed.
 */
RobotCommand Control_execute_action(RobotCommand command) {
    float move_x = 0.0f;
    float move_y = 0.0f;
    float move_h = 0.0f;
    switch (command) {
        case COMMAND_CUSTOM_WALK:
            // Custom walk used for stationary detections >= 66 cm
            printf("[CONTROL] Executing CUSTOM_WALK (Forward %.1f m - custom gait)\n", MOVE_DISTANCE);
            SendSpiderCommand(SPIDER_CUSTOM_WALK);
            move_x = MOVE_DISTANCE * cosf(deg_to_rad(current_pose.orientation_deg));
            move_y = MOVE_DISTANCE * sinf(deg_to_rad(current_pose.orientation_deg));
            break;
        
        case COMMAND_MOVE_WALK:
            printf("[CONTROL] Executing WALK (Forward %.1f m)\n", MOVE_DISTANCE);
            SendSpiderCommand(SPIDER_MOVE_FORWARD);
            move_x = MOVE_DISTANCE * cosf(deg_to_rad(current_pose.orientation_deg));
            move_y = MOVE_DISTANCE * sinf(deg_to_rad(current_pose.orientation_deg));
            break;
        case COMMAND_MOVE_BACKTRACK:
            // Backtrack moves the robot backwards along its current heading
            printf("[CONTROL] Executing BACKTRACK (Reverse %.1f m)\n", -BACKTRACK_DISTANCE);
            SendSpiderCommand(SPIDER_ROTATE_LEFT);
            SendSpiderCommand(SPIDER_ROTATE_LEFT);
            SendSpiderCommand(SPIDER_MOVE_FORWARD);
            move_x = BACKTRACK_DISTANCE * cosf(deg_to_rad(current_pose.orientation_deg));
            move_y = BACKTRACK_DISTANCE * sinf(deg_to_rad(current_pose.orientation_deg));
            break;
        case COMMAND_MOVE_TURN_RIGHT:
            printf("[CONTROL] Executing TURN_RIGHT (Rotating -%.1f degrees)\n", TURN_ANGLE);
            SendSpiderCommand(SPIDER_ROTATE_RIGHT);
            move_h = -TURN_ANGLE;
            break;
        case COMMAND_MOVE_TURN_LEFT:
            printf("[CONTROL] Executing TURN_LEFT (Rotating +%.1f degrees)\n", TURN_ANGLE);
            SendSpiderCommand(SPIDER_ROTATE_LEFT);
            move_h = TURN_ANGLE;
            break;
            
        case COMMAND_MOVE_STRAFE:
            // Strafe is a movement perpendicular to the current heading (e.g., to the right/positive turn)
            printf("[CONTROL] Executing STRAFE (Right Sideways %.1f m)\n", MOVE_DISTANCE);
            SendSpiderCommand(SPIDER_MOVE_RIGHT);
            // Strafe 90 degrees to the right of current orientation
            move_x = MOVE_DISTANCE * cosf(deg_to_rad(current_pose.orientation_deg + 90.0f));
            move_y = MOVE_DISTANCE * sinf(deg_to_rad(current_pose.orientation_deg + 90.0f));
            break;
            
        case COMMAND_MOVE_SCAN:
            // A scan step involves a turn and incrementing the step counter
            printf("[CONTROL] Executing SCAN (rotating +%.1f degrees)\n", TURN_ANGLE);
            SendSpiderCommand(SPIDER_ROTATE_RIGHT);
    
            move_h = TURN_ANGLE;
            scan_steps++;
            break;
            
        case COMMAND_TEMPERATURE_WALK:
            // Temperature walk - moves forward while performing temperature scan
            printf("[CONTROL] Executing TEMPERATURE_WALK (Forward %.1f m with temperature scan)\n", MOVE_DISTANCE);
            SendSpiderCommand(SPIDER_TEMPERATURE_WALK);
            SendHumanFoundCommand();
            move_x = MOVE_DISTANCE * cosf(deg_to_rad(current_pose.orientation_deg));
            move_y = MOVE_DISTANCE * sinf(deg_to_rad(current_pose.orientation_deg));
            break;
            
        case COMMAND_MOVE_HALT:
            printf(">>>> [COMMAND] HALT (GOAL REACHED/END)\n");
            SendSpiderCommand(SPIDER_STANDBY);
            return command;
            
        // Handle Critical Action Commands
        case COMMAND_CRIT_LOCKDOWN:
            printf(">>>> [COMMAND] LOCKDOWN_DOORS (Critical Threat Detected)\n");
            SendSpiderCommand(SPIDER_STANDBY);
            return command;
        
        case COMMAND_CRIT_NO_OP:
        case COMMAND_CRIT_STANDBY:
            printf("[CONTROL] STANDBY: Waiting for next decision.\n");
            SendSpiderCommand(SPIDER_STANDBY);
            return command;

        case COMMAND_STOP_PROGRAM:
            printf("[CONTROL] STOPPING PROGRAM...\n");
            SendSpiderCommand(SPIDER_STANDBY);
            return command;
            
        default:
            printf("[CONTROL] UNKNOWN COMMAND: %d\n", command);
            return command;
    }
    // Apply pose updates if movement was executed
    current_pose.x += move_x;
    current_pose.y += move_y;
    current_pose.orientation_deg = normalize_heading(current_pose.orientation_deg + move_h);
    
    // Log the movement for the current cycle
    if (move_x != 0.0f || move_y != 0.0f || move_h != 0.0f) {
        printf("[CONTROL] Pose Updated: (X:%.1f, Y:%.1f, H:%.1f)\n", current_pose.x, current_pose.y, current_pose.orientation_deg);
    }
    return command;
}
