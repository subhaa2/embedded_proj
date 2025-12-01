
// main.c (Simulation entry point)
#include <stdio.h>    // Standard I/O library (for printf)
#include <stdlib.h>   // Standard utility library (for exit)
#include "defs.h"     // Project-specific definitions, structs, and prototypes
// Global State Variables (Definitions corresponding to extern declarations in defs.h)
SensorData current_sim_data;            // Global structure to hold the most recent sensor readings
Pose current_pose = {0.0f, 0.0f, 0.0f}; // Robot's current simulated position (X, Y) and orientation (Heading)
int current_fork_id = 0;                // ID of the fork/junction the robot is currently at or tracking
// DFS variables defined in algorithm.c, but extern here:
extern int next_available_fork_id; 
extern Fork map_graph[MAX_FORKS];
extern int current_path_number;     
extern bool is_scanning;            
extern int scan_steps;
extern bool is_turning_to_backtrack;
extern float target_backtrack_heading;             
extern StackElement dfs_stack[STACK_SIZE];
extern int stack_ptr;

// Function to convert a command enum to a readable string for logging
const char* command_to_string(RobotCommand command) {
    switch(command) {
        case COMMAND_MOVE_WALK: return "MOVE_WALK";
        case COMMAND_MOVE_TURN_LEFT: return "MOVE_TURN_LEFT";
        case COMMAND_MOVE_TURN_RIGHT: return "MOVE_TURN_RIGHT";
        case COMMAND_MOVE_BACKTRACK: return "MOVE_BACKTRACK";
        case COMMAND_MOVE_STRAFE: return "MOVE_STRAFE";
        
        case COMMAND_MOVE_HALT: return "HALT";
        case COMMAND_MOVE_SCAN: return "MOVE_SCAN";
        case COMMAND_CUSTOM_WALK: return "CUSTOM_WALK";
        case COMMAND_TEMPERATURE_WALK: return "TEMPERATURE_WALK";
        case COMMAND_CRIT_STANDBY: return "STANDBY";
        case COMMAND_CRIT_NO_OP: return "NO_OP";
        case COMMAND_CRIT_LOCKDOWN: return "LOCKDOWN";
        default: return "UNKNOWN_CMD";
    }
}
/**
 * @brief The main simulation loop.
 */
int main() {
    printf("--- SPIDER DFS SIMULATION START ---\n");
    
    // 1. Initialize Systems
    // Initialize the sensor system with the final mmWave data file
    Sensors_init("sensor_data.txt"); 
    DFS_init_map();
    
    RobotCommand next_command = COMMAND_CRIT_STANDBY; // Start in standby to initiate the first sensor read
    RobotCommand executed_command = COMMAND_CRIT_STANDBY;
    
    int loop_counter = 0;
    
    // The Main Loop: Sense -> Think -> Act
    while (executed_command != COMMAND_MOVE_HALT && loop_counter < MAX_SIM_STEPS) {
        loop_counter++;
        printf("\n--- CYCLE %d ---\n", loop_counter);
        // Step 1: SENSE - Read the next sensor data point
        if (!Sensors_get_next_data()) {
            printf("[MAIN] End of sensor data reached. Exiting loop.\n");
            break;
        }
        // Log the newly read data
        Control_log_data();
        
        // Determine the environment status from the parsed sensor data
        EnvironmentStatus env_status = current_sim_data.status;
        int target_path_index = -1; 
        
        // Step 2: THINK - Determine next command
        // Special case: If we're currently scanning or turning to backtrack, always consult the algorithm
        // (don't trigger implicit walk during these operations)
        if (is_scanning || is_turning_to_backtrack) {
            next_command = DFS_decision_maker(current_fork_id, env_status, &target_path_index);
        }
        // If the sensor reports a clear path and we're NOT scanning or turning, the robot implicitly walks
        else if (env_status == CLEAR_PATH) {
            next_command = COMMAND_MOVE_WALK;
            printf("[MAIN] Implicit WALK triggered by CLEAR_PATH.\n");
        } 
        // Otherwise, consult the algorithm for decision
        else {
            next_command = DFS_decision_maker(current_fork_id, env_status, &target_path_index);
        }
        
        // Step 3: ACT - Execute the determined command
        executed_command = Control_execute_action(next_command);
        printf("  - EXECUTION: %s\n", command_to_string(executed_command));
        
        // If the algorithm returned a non-movement command (e.g., STANDBY), we loop immediately
        // to get the next decision (which might be the start of a SCAN sequence).
        if (executed_command == COMMAND_CRIT_STANDBY || executed_command == COMMAND_CRIT_NO_OP) {
            // Do not increment loop_counter here to avoid confusing the log cycles, 
            // but the loop check will ensure we don't exceed MAX_SIM_STEPS.
            // For a robust simulation, we increment the main loop counter for every sensor read.
        }
    }
    // Simulation End Summary
    printf("\n--- SIMULATION END ---\n");
    if (executed_command == COMMAND_MOVE_HALT) {
        printf("SUCCESS: GOAL REACHED in %d cycles.\n", loop_counter);
    } else if (loop_counter >= MAX_SIM_STEPS) {
        printf("!!! SIMULATION TIMED OUT after %d cycles.\n", loop_counter);
    } else {
        printf("FINISHED after %d cycles due to end of sensor data.\n", loop_counter);
    }
    return 0;
}
