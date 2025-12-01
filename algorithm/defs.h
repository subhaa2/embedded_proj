
#ifndef DEFS_H
#define DEFS_H
#include <stdbool.h>
// --- Constants ---
#define MAX_PATHS 3
#define MAX_FORKS 50
#define STACK_SIZE 20
#define MAX_SIM_STEPS 2000
// --- OBSOLETE FFT Thresholds (Removed/Commented out) ---
// #define WALL_THRESHOLD 2500.0
// #define GOAL_MAG_VALUE 3000.0
// #define FORK_THRESHOLD 1500.0
// #define STRAFE_FFT_VALUE 1200.0
// #define CLIMB_FFT_VALUE 1100.0
// --- Unified Command Enumeration ---
typedef enum {
    // Movement/DFS Actions
    COMMAND_MOVE_WALK,        
    COMMAND_MOVE_TURN_LEFT,   
    COMMAND_MOVE_TURN_RIGHT,  
    COMMAND_MOVE_BACKTRACK,   
    COMMAND_MOVE_STRAFE,      
    
    COMMAND_MOVE_HALT,        
    COMMAND_MOVE_SCAN,
    COMMAND_CUSTOM_WALK,      
    COMMAND_TEMPERATURE_WALK,        
    // Critical Action Commands
    COMMAND_CRIT_STANDBY,     // Wait for next sensor cycle/decision
    COMMAND_CRIT_NO_OP,       // No operation, usually for error handling
    COMMAND_CRIT_LOCKDOWN     // Emergency stop (e.g., Critical Threat)
} RobotCommand;
// --- Sensor Status Enumeration (Based on mmWave Detection Type) ---
// This is the logical mapping derived by the system from the raw sensor input.
typedef enum {
    UNKNOWN_STATUS,     // Default or unrecognized state
    CLEAR_PATH,         // No object detected in front of the robot
    WALL,               // Maps to 'Furniture' - treated as a Dead End or a Fork depending on state
    STRAFE_OBJECT,      // Maps to 'Small Object'
    STATIONARY,         // Unified stationary obstacle classification
    
    FORK,               // Logically represents a Fork/Junction (used in algorithm.c, derived from WALL/Furniture)
    GOAL_OBJECT,         // Maps to 'Goal'
    EMPTY_SPACE_DETECTED,
    POSSIBLE_HUMAN,     // Possible human detected - need temperature verification
    HUMAN_CONFIRMED     // Human confirmed with temperature reading
} EnvironmentStatus;
// --- Sensor Data Structure (Updated for mmWave) ---
typedef struct {
    EnvironmentStatus status;   // The logical status determined by sensor parsing (e.g., WALL, STRAFE_OBJECT)
    float distance_m;           // Distance to the detected object (in meters)
    float temperature_c;        // Temperature reading for human confirmation (in Celsius)
    // float fft_peak_frequency; // OLD: Removed
    // float fft_mag;            // OLD: Removed
} SensorData;
// --- Robot Pose Structure ---
typedef struct {
    float x;
    float y;
    float orientation_deg;
} Pose;
// --- Path Status Enumeration ---
typedef enum {
    PATH_UNEXPLORED,
    PATH_EXPLORING,
    PATH_DEAD_END,
    PATH_SUCCESS
} PathStatus;
// --- Path Structure ---
typedef struct {
    int fork_id;                // ID of the fork this path leads to (or -1 if dead end)
    PathStatus status;          // Status of this path (e.g., UNEXPLORED, DEAD_END)
    float target_heading_deg;   // Heading required to turn onto this path from the fork's entry heading
} Path;
// --- Fork Structure ---
typedef struct {
    int id;                         // Unique ID for the fork (0 for Start)
    Path paths[MAX_PATHS];          // Paths detected during 360° scan at this fork
    float fork_entry_heading_deg;   // Heading when robot first entered this fork
    int num_paths_detected;         // How many paths were detected during scan (1-MAX_PATHS)
    bool has_been_scanned;          // Whether a 360° scan has been performed at this fork
} Fork;
// --- Stack Element Structure (For DFS Backtracking) ---
typedef struct {
    int fork_id;
    int path_index;
    Pose entry_pose;
} StackElement;

// --- Global variables (extern) ---
extern Pose current_pose;
extern int current_fork_id;
extern int next_available_fork_id; 
extern SensorData current_sim_data;
extern Fork map_graph[MAX_FORKS];
extern int current_path_number;     
extern bool is_scanning;            
extern int scan_steps;
extern bool is_turning_to_backtrack;
extern float target_backtrack_heading;             
extern StackElement dfs_stack[STACK_SIZE];
extern int stack_ptr;

// --- Function Prototypes ---
// algorithm.c 
void DFS_init_map();
RobotCommand DFS_decision_maker(int fork_id, EnvironmentStatus environment_status, int *target_path_index);
StackElement stack_pop(); 
void stack_push(StackElement element);
// control.c
void Control_log_data(void);
RobotCommand Control_execute_action(RobotCommand command);
// sensors.c
void Sensors_init(const char* filename);
bool Sensors_get_next_data();
float Sensors_get_angle_to_turn(float current_heading, float path_target_heading);

#endif // DEFS_H
