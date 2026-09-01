/*
    VESC Internal Encoder Driver
    
    This driver uses the VESC's internal position feedback (pid_pos_now) 
    as the encoder source, providing closed-loop control without external encoders.
    
    Features:
    - Uses VESC position data received via CAN
    - Converts revolutions to degrees with gear ratio compensation
    - Calculates velocity from position changes
    - Validates data based on CAN message age
    - Supports calibration via VESC position offset update
*/

#include "encoder_interface.h"
#include "board_config.h"
#include "esp_log.h"
#include <math.h>

// Include CAN communication for calibration
extern void comm_can_update_pid_pos_offset(int id, float angle_now, bool store);

static const char* TAG = "VESC_ENC";

// External variables from main.c
extern float vesc_current_position;  // Position in revolutions from VESC
extern bool vesc_position_valid;     // True if VESC data is recent and valid
extern uint32_t current_time;        // Current system time in ms

// Internal state
static float last_position_rev = 0.0f;
static float current_velocity_deg_s = 0.0f;
static uint32_t last_update_time = 0;
static uint32_t error_count = 0;
static bool initialized = false;

// Convert VESC position (revolutions) to joint angle (degrees)
static float vesc_position_to_joint_degrees(float vesc_revolutions) {
    // Convert VESC revolutions to joint degrees using gear ratio
    // VESC position offset is handled by VESC itself via comm_can_update_pid_pos_offset
    float joint_degrees = vesc_revolutions * 360.0f / GEAR_RATIO;
    return joint_degrees;
}

// Initialize VESC encoder
static bool vesc_encoder_init(void) {
    ESP_LOGI(TAG, "Initializing VESC internal encoder");
    ESP_LOGI(TAG, "Gear ratio: %.1f:1", GEAR_RATIO);
    ESP_LOGI(TAG, "Using VESC position feedback via CAN");
    
    last_position_rev = 0.0f;
    current_velocity_deg_s = 0.0f;
    last_update_time = 0;
    error_count = 0;
    initialized = true;
    
    ESP_LOGI(TAG, "VESC encoder initialized successfully");
    return true;
}

// Update encoder data (called periodically)
static bool vesc_encoder_update(void) {
    if (!initialized) return false;
    
    if (vesc_position_valid) {
        // Calculate velocity from position change
        if (last_update_time > 0) {
            uint32_t dt_ms = current_time - last_update_time;
            if (dt_ms > 0 && dt_ms < 1000) {  // Reasonable time delta
                float position_change_rev = vesc_current_position - last_position_rev;
                float position_change_deg = position_change_rev * 360.0f / GEAR_RATIO;
                current_velocity_deg_s = (position_change_deg * 1000.0f) / dt_ms;
            }
        }
        
        last_position_rev = vesc_current_position;
        last_update_time = current_time;
        return true;
    } else {
        // VESC data is invalid/stale
        if (current_time - last_update_time > 500) {  // No valid data for 500ms
            current_velocity_deg_s = 0.0f;
            error_count++;
        }
        return false;
    }
}

// Get current angle in radians
static float vesc_encoder_get_angle_rad(void) {
    if (!vesc_position_valid) return 0.0f;
    
    float joint_degrees = vesc_position_to_joint_degrees(vesc_current_position);
    return joint_degrees * M_PI / 180.0f;
}

// Get current angle in degrees
static float vesc_encoder_get_angle_deg(void) {
    if (!vesc_position_valid) return 0.0f;
    
    return vesc_position_to_joint_degrees(vesc_current_position);
}

// Get velocity in rad/s
static float vesc_encoder_get_velocity_rad_s(void) {
    return current_velocity_deg_s * M_PI / 180.0f;
}

// Get velocity in deg/s
static float vesc_encoder_get_velocity_deg_s(void) {
    return current_velocity_deg_s;
}

// Check if encoder data is valid
static bool vesc_encoder_is_valid(void) {
    return initialized && vesc_position_valid;
}

// Get error count
static uint32_t vesc_encoder_get_error_count(void) {
    return error_count;
}

// Get encoder type name
static const char* vesc_encoder_get_type_name(void) {
    return "VESC Internal";
}

// Reset errors
static void vesc_encoder_reset_errors(void) {
    error_count = 0;
}

// Set zero position (calibration)
static bool vesc_encoder_set_zero_position(float target_angle_deg) {
    if (!initialized || !vesc_position_valid) {
        ESP_LOGW(TAG, "Cannot calibrate: encoder not initialized or VESC data invalid");
        return false;
    }
    
    // Convert target angle from joint degrees to VESC revolutions
    // This tells the VESC "your current position should be considered as this many revolutions"
    float target_vesc_revolutions = target_angle_deg * GEAR_RATIO / 360.0f;
    
    // Send calibration command to VESC
    // This updates the VESC's internal position offset so that the current position
    // reads as target_vesc_revolutions instead of the current raw encoder value
    // NOTE: Wait for safe CAN slot to ensure command is received
    extern void wait_for_safe_can_slot(void);  // Import from main.c
    wait_for_safe_can_slot();
    comm_can_update_pid_pos_offset(CAN_VESC_ID, target_vesc_revolutions, true);
    
    ESP_LOGI(TAG, "Sent calibration command to VESC ID %d", CAN_VESC_ID);
    ESP_LOGI(TAG, "Target joint angle: %.1f° → VESC revolutions: %.6f", 
             target_angle_deg, target_vesc_revolutions);
    ESP_LOGI(TAG, "VESC will update its position offset (stored permanently)");
    
    return true;
}

// VESC encoder interface
const encoder_interface_t vesc_encoder_interface = {
    .init = vesc_encoder_init,
    .update = vesc_encoder_update,
    .get_angle_rad = vesc_encoder_get_angle_rad,
    .get_angle_deg = vesc_encoder_get_angle_deg,
    .get_velocity_rad_s = vesc_encoder_get_velocity_rad_s,
    .get_velocity_deg_s = vesc_encoder_get_velocity_deg_s,
    .is_valid = vesc_encoder_is_valid,
    .get_error_count = vesc_encoder_get_error_count,
    .reset_errors = vesc_encoder_reset_errors,
    .set_zero_position = vesc_encoder_set_zero_position,
    .get_type_name = vesc_encoder_get_type_name
};
