
// algorithm.c - Depth-first exploration decision logic
#include "defs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
// --- Global Variables Definition (as per defs.h) ---
Fork map_graph[MAX_FORKS];
int next_available_fork_id = 1; 
int current_path_number = 1; 
bool is_scanning = false;
float scan_start_heading = 0.0f; 
int scan_steps = 0;
bool is_turning_to_backtrack = false;
float target_backtrack_heading = 0.0f;
StackElement dfs_stack[STACK_SIZE];
int stack_ptr = 0;
         
// --- Utility Functions ---
static float normalize_heading(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}
static float angle_difference(float current_deg, float target_deg) {
    float diff = target_deg - current_deg;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}
extern float Sensors_get_angle_to_turn(float current_heading, float path_target_heading);

// --- Fork/Map Management Functions ---
void DFS_init_map() {
    printf("[DFS_INIT] Initializing map graph and DFS stack.\n");
    memset(map_graph, 0, sizeof(map_graph));
    map_graph[0].id = 0; // Start Fork
    map_graph[0].fork_entry_heading_deg = 0.0f; // Starting heading
    current_fork_id = 0;
}
static void Fork_update_path_status(Fork *fork, int path_index, PathStatus status) {
    if (fork && path_index >= 0 && path_index < MAX_PATHS) {
        fork->paths[path_index].status = status;
        printf("[MAP_UPDATE] Fork %d, Path %d status set to %s.\n", fork->id, path_index, 
                (status == PATH_DEAD_END) ? "DEAD_END" : (status == PATH_SUCCESS ? "SUCCESS" : "EXPLORING"));
    }
}
static int Fork_find_unexplored_path(Fork *fork) {
    if (!fork) return -1;
    for (int i = 0; i < fork->num_paths_detected; ++i) {
        if (fork->paths[i].status == PATH_UNEXPLORED) {
            return i;
        }
    }
    return -1;
}
static bool Fork_scan_paths(Fork *fork) {
    int MAX_SCAN_STEPS = 4;
    
    if (scan_steps == 0) {
        scan_start_heading = current_pose.orientation_deg;
        for (int i = 1; i < MAX_PATHS; i++) {
             memset(&fork->paths[i], 0, sizeof(Path));
        }
        printf("\n>>>> [SCAN START] Scanning at Fork %d. Entry Heading: %.1f deg.\n", fork->id, fork->fork_entry_heading_deg);
        return false;
    }
    
    float back_path_heading = normalize_heading(fork->fork_entry_heading_deg + 180.0f);
    float wall_path_heading = fork->paths[0].target_heading_deg;
    
    if (scan_steps >= 1 && scan_steps <= MAX_SCAN_STEPS) { 
        float relative_angle = scan_steps * 90.0f;
        float absolute_heading = normalize_heading(scan_start_heading + relative_angle);
        
        const char* direction_name = (relative_angle == 90) ? "Right" : 
                                     (relative_angle == 180 ? "Back" : 
                                     (relative_angle == 270 ? "Left" : "Front"));
        
        float detected_heading = absolute_heading;
        
        // Check 1: Ignore the back path (180 deg relative) - regardless of WALL or EMPTY_SPACE
        if (fabs(angle_difference(detected_heading, back_path_heading)) < 5.0f) {
            if (current_sim_data.status == WALL) {
                printf("[SCAN IGNORE] Direction at %.1f deg (%.1f deg relative: %s) - Back path (%.1f deg) has WALL, ignored.\n", 
                       absolute_heading, relative_angle, direction_name, back_path_heading);
            } else {
                printf("[SCAN IGNORE] Direction at %.1f deg (%.1f deg relative: %s) - Back path (%.1f deg) ignored.\n", 
                       absolute_heading, relative_angle, direction_name, back_path_heading);
            }
        }
        // Check 2: Ignore the wall path (Path 0) if EMPTY_SPACE detected there
        else if (current_sim_data.status != WALL && fork->num_paths_detected > 0 && fabs(angle_difference(detected_heading, wall_path_heading)) < 5.0f) {
            printf("[SCAN IGNORE] Direction at %.1f deg (%.1f deg relative: %s) - Wall path (%.1f deg) already marked DEAD_END.\n", 
                   absolute_heading, relative_angle, direction_name, wall_path_heading);
        }
        // Handle WALL detections (that aren't the back path)
        else if (current_sim_data.status == WALL) {
            printf("[SCAN BLOCKED] Direction at %.1f deg (%.1f deg relative: %s) - STATIONARY detected, path blocked.\n", 
                    absolute_heading, relative_angle, direction_name);
        } 
        // Handle EMPTY_SPACE detections - new unexplored path
        else {
            if (fork->num_paths_detected < MAX_PATHS) {
                int new_path_index = fork->num_paths_detected; 
                
                fork->paths[new_path_index].target_heading_deg = absolute_heading;
                fork->paths[new_path_index].status = PATH_UNEXPLORED;
                fork->num_paths_detected++;
                
                printf("[SCAN DETECT] Path %d found at %.1f deg (%.1f deg relative: %s) - Empty space detected, path OPEN.\n", 
                        new_path_index, absolute_heading, relative_angle, direction_name);
            }
        }
    }
    if (scan_steps >= MAX_SCAN_STEPS) { 
        printf("<<<< [SCAN COMPLETE] Fork %d Scan Finished.\n", fork->id);
        printf("<<<<   Total Paths Available: %d\n", fork->num_paths_detected);
        for (int i = 0; i < fork->num_paths_detected; i++) {
            printf("<<<<   - Path %d: Heading %.1f deg (Status: %s)\n", 
                    i, fork->paths[i].target_heading_deg, 
                    (fork->paths[i].status == PATH_DEAD_END) ? "DEAD_END" : "UNEXPLORED");
        }
        
        scan_steps = 0;
        fork->has_been_scanned = true;
        is_scanning = false;
        return true;
    }
    return false;
}
// --- Main Decision Logic ---
RobotCommand DFS_decision_maker(int fork_id, EnvironmentStatus environment_status, int *target_path_index) {
    Fork *current_fork = &map_graph[fork_id];
    *target_path_index = -1; 
    
    // --- STEP 1: Handle Sensor/Environment Events ---
    switch (environment_status) {
        
        case STRAFE_OBJECT:
            printf("[DFS_DECISION] STRAFE_OBJECT (Small Object) detected at %.2f m. Action: STRAFE.\n", current_sim_data.distance_m);
            return COMMAND_MOVE_STRAFE;
        
        case POSSIBLE_HUMAN:
            printf("[DFS_DECISION] POSSIBLE_HUMAN detected. Action: TEMPERATURE_WALK.\n");
            return COMMAND_TEMPERATURE_WALK;
        case HUMAN_CONFIRMED:
            printf("[GOAL REACHED] Human confirmed, temperature %.1f degrees\n", current_sim_data.temperature_c);
            Fork_update_path_status(current_fork, current_path_number, PATH_SUCCESS);
            return COMMAND_MOVE_HALT;
        case GOAL_OBJECT:
            printf("[DFS_DECISION] GOAL_OBJECT detected at %.2f m. Action: HALT.\n", current_sim_data.distance_m);
            Fork_update_path_status(current_fork, current_path_number, PATH_SUCCESS);
            return COMMAND_MOVE_HALT;
        case WALL:
            printf("[DFS_DECISION] WALL (Stationary) detected at %.2f m.\n", current_sim_data.distance_m);
            
            // If Stationary is detected far (>= 66cm), perform a custom walk first, then scan next
            if (!is_scanning && current_sim_data.distance_m >= 0.66f) {
                return COMMAND_CUSTOM_WALK;
            }
            
            if (is_scanning) {
                break;
            }
            
            // When a wall is detected while exploring a path, treat it as a potential new fork
            // The robot should scan to determine if there are alternative paths
            // Only after scanning and finding no paths should it be marked as a dead end
            
            // Case: NEW FORK (or potential dead end - scan will determine)
            StackElement new_fork_state = {
                .fork_id = current_fork_id,
                .path_index = current_path_number,
                .entry_pose = current_pose
            };
            stack_push(new_fork_state);
            current_fork_id = next_available_fork_id++;
            current_fork = &map_graph[current_fork_id];
            current_fork->id = current_fork_id;
            current_fork->has_been_scanned = false;
            current_fork->fork_entry_heading_deg = current_pose.orientation_deg;

            // FIX: Explicitly mark the path in front as DEAD_END (Path 0)
            int wall_path_index = 0; 
            current_fork->paths[wall_path_index].target_heading_deg = current_pose.orientation_deg;
            Fork_update_path_status(current_fork, wall_path_index, PATH_DEAD_END);
            current_fork->num_paths_detected = 1; 
            
            is_scanning = true;
            scan_steps = 0; 
            printf("[DFS_DECISION] WALL detected, assuming new FORK %d. Initiating SCAN sequence.\n", current_fork_id);
            break;
        default:
            // Default behavior: continue walking if not in a special state
            if (is_scanning) {
                break; 
            }
            
            // Check if the current path was just marked as a DEAD END
            if (current_fork->paths[current_path_number].status == PATH_DEAD_END) {
                 printf("[DFS_DECISION] Path is DEAD_END. Continuing BACKTRACK.\n");
                 return COMMAND_MOVE_BACKTRACK;
            }
            
            // Default action: WALK forward
            printf("[DFS_DECISION] No specific obstacle detected. Action: WALK.\n");
            return COMMAND_MOVE_WALK;
    }
    // --- STEP 2: Handle DFS Logic ---
    
    // A. Scanning Sequence 
    if (is_scanning) {
        if (Fork_scan_paths(current_fork)) {
            printf("[DFS_DECISION] Scan complete. Proceeding to path selection.\n");
        } else {
            // Before each scan step, return STANDBY to get sensor reading
            if (scan_steps == 0) {
                printf("[DFS_DECISION] Starting scan - STANDBY for initial sensor reading.\n");
                return COMMAND_CRIT_STANDBY;
            }
            return COMMAND_MOVE_SCAN; 
        }
    }
    
    // B. Path Selection/Backtracking
    if (current_fork->has_been_scanned) {
        int path_to_explore = Fork_find_unexplored_path(current_fork);
        
        if (path_to_explore != -1) {
            *target_path_index = path_to_explore;
            current_path_number = path_to_explore;
            Fork_update_path_status(current_fork, current_path_number, PATH_EXPLORING);
            Path *path_data = &current_fork->paths[path_to_explore];
            float angle_needed = Sensors_get_angle_to_turn(current_pose.orientation_deg, path_data->target_heading_deg);
            
            printf("[DFS_DECISION] Exploring Path %d (Heading: %.1f). Angle needed: %.1f. Action: TURN.\n", 
                   path_to_explore, path_data->target_heading_deg, angle_needed);
            if (fabs(angle_needed) > 1.0f) {
                // Positive angle -> TURN LEFT; Negative angle -> TURN RIGHT
                return (angle_needed > 0) ? COMMAND_MOVE_TURN_LEFT : COMMAND_MOVE_TURN_RIGHT;
            } else {
                return COMMAND_CRIT_STANDBY;
            }
        } else if (fork_id != 0) {
            // All paths explored at this fork - it's a dead end
            // Mark the path that led here as DEAD_END in the parent fork
            StackElement previous_state = stack_pop();
            Fork *parent_fork = &map_graph[previous_state.fork_id];
            Fork_update_path_status(parent_fork, previous_state.path_index, PATH_DEAD_END);
            
            current_fork_id = previous_state.fork_id;
            current_path_number = previous_state.path_index; 
            current_pose = previous_state.entry_pose; 
            
            printf("[DFS_DECISION] Fork %d is a DEAD END (no unexplored paths). Path %d marked as DEAD_END.\n", 
                   fork_id, previous_state.path_index);
            printf("[DFS_DECISION] BACKTRACKING to Fork %d. Action: BACKTRACK (includes 180 turn).\n", 
                   current_fork_id);
            return COMMAND_MOVE_BACKTRACK;
        } else {
            printf("[DFS_FINAL] Map fully explored from root. HALTING.\n");
            return COMMAND_MOVE_HALT;
        }
    }
    return COMMAND_CRIT_STANDBY;
}
// --- Stack operations implementation (Not fully shown, but assumed working) ---
void stack_push(StackElement element) {
    if (stack_ptr < STACK_SIZE) {
        dfs_stack[stack_ptr++] = element;
        printf("[STACK] Pushed Fork ID %d, Path Index %d. Stack size: %d\n", element.fork_id, element.path_index, stack_ptr);
    } else {
        fprintf(stderr, "[STACK_ERROR] Stack overflow! Cannot push element.\n");
    }
}
StackElement stack_pop() {
    if (stack_ptr > 0) {
        printf("[STACK] Popped element. Stack size: %d\n", stack_ptr - 1);
        return dfs_stack[--stack_ptr];
    } else {
        fprintf(stderr, "[STACK_ERROR] Stack underflow! Cannot pop element.\n");    
        return (StackElement){.fork_id = -1, .path_index = -1};
    }
}
