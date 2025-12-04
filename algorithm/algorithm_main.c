
// main.c (Simulation entry point)
#include <stdio.h>    // Standard I/O library (for printf)
#include <stdlib.h>   // Standard utility library (for exit)
#include "defs.h"     // Project-specific definitions, structs, and prototypes
#include "main.h"

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

#define SPIDER_90DEGREE -1
#define SPIDER_STANDBY 0
#define SPIDER_MOVE_FORWARD 1
#define SPIDER_MOVE_BACKWARD 2
#define SPIDER_MOVE_LEFT 3
#define SPIDER_MOVE_RIGHT 4
#define SPIDER_ROTATE_LEFT 5
#define SPIDER_ROTATE_RIGHT 6
#define SPIDER_CUSTOM_WALK 7
#define SPIDER_TEMPERATURE_WALK 8
#define SPIDER_BACKTRACK 9

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
        case COMMAND_STOP_PROGRAM: return "STOP_PROGRAM";
        default: return "UNKNOWN_CMD";
    }
}
/**
 * @brief The main simulation loop.
 */

int algo_init(){
    DFS_init_map();
}

// Map RobotCommand to Spider Command
int map_algorithm_to_spider(RobotCommand cmd) {
    switch(cmd) {
        case COMMAND_MOVE_WALK:
            return SPIDER_MOVE_FORWARD;
        case COMMAND_MOVE_TURN_LEFT:
            return SPIDER_ROTATE_LEFT;
        case COMMAND_MOVE_TURN_RIGHT:
            return SPIDER_ROTATE_RIGHT;
        case COMMAND_MOVE_BACKTRACK:
            return SPIDER_BACKTRACK;
        case COMMAND_MOVE_STRAFE:
            return SPIDER_MOVE_RIGHT;
        case COMMAND_MOVE_HALT:
            SendHumanFoundCommand();
            return SPIDER_STANDBY;
        case COMMAND_TEMPERATURE_WALK:
            return SPIDER_TEMPERATURE_WALK;
        case COMMAND_CUSTOM_WALK:
            return SPIDER_CUSTOM_WALK;
        case COMMAND_CRIT_STANDBY:
        case COMMAND_MOVE_SCAN:
        default:
            return SPIDER_STANDBY;
    }
}

// Process sensor data through algorithm and send appropriate command
void ProcessSensorDataAndSendCommand(const char* radar_data) {
    // Parse the sensor data
    SensorData sensor_data = Sensors_parse_radar_string(radar_data);
    
    if (sensor_data.status == UNKNOWN_STATUS) {
        printf("[ALGORITHM] Unknown sensor status, skipping\n");
        return;
    }
    
    // Update global sensor data
    current_sim_data = sensor_data;
    
    // Get environment status
    EnvironmentStatus env_status = sensor_data.status;
    int target_path_index = -1;
    RobotCommand next_command;
    
    // Determine next command (similar logic to algorithm_main.c)
    if (is_scanning || is_turning_to_backtrack) {
        next_command = DFS_decision_maker(current_fork_id, env_status, &target_path_index);
    }
    else if (env_status == CLEAR_PATH) {
        next_command = COMMAND_MOVE_WALK;
        printf("[ALGORITHM] Implicit WALK triggered by CLEAR_PATH\n");
    }
    else {
        next_command = DFS_decision_maker(current_fork_id, env_status, &target_path_index);
    }
    
    Control_execute_action(next_command);
    // Map to spider command
    int spider_cmd = map_algorithm_to_spider(next_command);
    
    // Send the command
    //SendSpiderCommand(spider_cmd);
    
    printf("[ALGORITHM] Command: %d → Spider: SC%d\n", next_command, spider_cmd);
}

void algo_execute_spider_command(){
    
    RobotCommand next_command = COMMAND_CRIT_STANDBY; // Start in standby to initiate the first sensor read
    RobotCommand executed_command = COMMAND_CRIT_STANDBY;

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
    

    if (next_command == COMMAND_STOP_PROGRAM){
        printf("Demo End\n");
        isCompleted = true;
    }

    // Step 3: ACT - Execute the determined command
    executed_command = Control_execute_action(next_command);
    printf("  - EXECUTION: %s\n", command_to_string(executed_command));
}
