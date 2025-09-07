/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_flash.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "main.h"
#include "conf_general.h"
#include "debug_config.h"
#include "comm/crsf_receiver.h"
#include "comm/crsf_config.h"
#include "comm/crsf_utils.h"
#include "comm/comm_can.h"
#include "drivers/encoder_interface.h"
#include "datatypes.h"
#include "board_config.h"
#include "utils.h"

// Main task and debug configuration
static const char *TAG = "VESC_Express";

// ESP-NOW telemetry configuration
#if ESP_NOW_TELEMETRY_ENABLE
#include "comm/comm_espnow.h"
#endif

// Global backup variable (required by comm_can.c)
volatile backup_data backup = {
    .controller_id_init_flag = VAR_INIT_CODE,
    .controller_id = CAN_ESP32_ID,  // ESP32's own CAN ID
    .can_baud_rate_init_flag = VAR_INIT_CODE,
    .can_baud_rate = CAN_BAUD_500K,
    .config_init_flag = VAR_INIT_CODE,
    .config = {
        .controller_id = CAN_ESP32_ID,  // ESP32's own CAN ID
        .can_baud_rate = CAN_BAUD_500K,
        .can_status_rate_hz = 50,  // 50Hz status rate
        .wifi_mode = WIFI_MODE_DISABLED,
        .ble_mode = BLE_MODE_DISABLED
    }
};

// Global CRSF data
uint16_t channels[16] = {0}; // CRSF channel data (1000-2000 scaled)

// Global VESC status tracking - updated in main loop, used by control callback
float vesc_current_position = 0.0f;
bool vesc_position_valid = false;

// VESC position tracking for fallback control
float vesc_tracked_position_revs = 0.0f;  // Continuously tracked VESC position in revolutions
float vesc_tracked_position_degrees = 0.0f;  // Converted to joint degrees
bool vesc_tracking_initialized = false;
uint32_t last_vesc_position_update = 0;

// Channel 6 calibration control
static bool last_channel6_state = false;  // Previous state of channel 6 (high/low)
static uint32_t last_calibration_time = 0;

// VESC configuration update tracking
static uint32_t last_vesc_config_update = 0;
static bool vesc_config_sent = false;

// Forward declarations
void wait_for_safe_can_slot(void);

#if DEBUG_ESPNOW_TEST
// ESP-NOW test function - sends random telemetry data for testing
void espnow_test_send_random_data(void) {
    static uint32_t last_test_send = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Send test data every 1 second
    if (current_time - last_test_send > 1000) {
        // Generate random test data (but keep board name from config)
        float random_encoder = -180.0f + ((float)esp_random() / UINT32_MAX) * 360.0f;  // Random -180 to +180 degrees
        float random_crsf_target = -90.0f + ((float)esp_random() / UINT32_MAX) * 180.0f;  // Random -90 to +90 degrees  
        float random_vesc_target = -5.0f + ((float)esp_random() / UINT32_MAX) * 10.0f;  // Random -5 to +5 revolutions
        
        // Update telemetry with random test data (board name still from board_config.h)
        telemetry_espnow_set_payload_data(BOARD_NAME, random_encoder, random_crsf_target, random_vesc_target);
        
        ESP_LOGI(TAG, "[ESPNOW_TEST] Sent random data: %s, Enc:%.1f°, CRSF:%.1f°, VESC:%.3frev", 
                BOARD_NAME, random_encoder, random_crsf_target, random_vesc_target);
        
        last_test_send = current_time;
    }
}
#endif

// VESC configuration update function - sends velocity/acceleration parameters periodically
void update_vesc_motion_parameters(uint32_t current_time) {
    // Send configuration parameters every 5 seconds, or immediately if never sent
    const uint32_t CONFIG_UPDATE_INTERVAL_MS = 5000;  // 5 seconds
    
    if (!vesc_config_sent || (current_time - last_vesc_config_update > CONFIG_UPDATE_INTERVAL_MS)) {
        ESP_LOGI(TAG, "Updating VESC motion parameters");
        
        // Send velocity and acceleration limits (wait for safe CAN slots between commands)
        wait_for_safe_can_slot();
        comm_can_set_max_sp_vel(CAN_VESC_ID, MAX_VEL);
        ESP_LOGI(TAG, "Set max velocity: %.1f", MAX_VEL);
        
        wait_for_safe_can_slot();
        comm_can_set_max_sp_accel(CAN_VESC_ID, MAX_ACCEL);
        ESP_LOGI(TAG, "Set max acceleration: %.1f", MAX_ACCEL);
        
        wait_for_safe_can_slot();
        comm_can_set_max_sp_decel(CAN_VESC_ID, MAX_DECEL);
        ESP_LOGI(TAG, "Set max deceleration: %.1f", MAX_DECEL);
        
        last_vesc_config_update = current_time;
        vesc_config_sent = true;
    }
}

// CAN bus timing coordination
static uint32_t can_command_offset_ms = 5;  // Offset commands by 5ms to avoid 50Hz status collisions

// Debug macros (main.c specific)
#if DEBUG_POSITION_CONTROL
#define DEBUG_POS(fmt, ...) ESP_LOGI(TAG, "[POS] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_POS(fmt, ...)
#endif

#if DEBUG_ENCODER_DATA
#define DEBUG_ENC(fmt, ...) ESP_LOGI(TAG, "[ENC] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_ENC(fmt, ...)
#endif

#if DEBUG_VESC_STATUS
#define DEBUG_VESC(fmt, ...) ESP_LOGI(TAG, "[VESC] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_VESC(fmt, ...)
#endif

#if DEBUG_CAN_COMMANDS
#define DEBUG_CAN_CMD(fmt, ...) ESP_LOGI(TAG, "[CAN_CMD] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_CAN_CMD(fmt, ...)
#endif

// CAN timing coordination - wait for safe slot to send commands
// This prevents collisions with VESC's 50Hz status messages
void wait_for_safe_can_slot(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Calculate time within the 20ms status message cycle (50Hz = 20ms period)
    uint32_t cycle_time = current_time % 20;
    
    // If we're too close to when status messages are sent (0-5ms or 15-20ms), wait
    if (cycle_time < can_command_offset_ms || cycle_time > (20 - can_command_offset_ms)) {
        uint32_t wait_time = can_command_offset_ms - cycle_time;
        if (wait_time > 10) wait_time = can_command_offset_ms; // Handle wrap-around
        
        vTaskDelay(pdMS_TO_TICKS(wait_time));
    }
}

// VESC position tracking for fallback control when encoder is invalid
bool is_vesc_tracking_valid(uint32_t current_time) {
    return vesc_tracking_initialized && (current_time - last_vesc_position_update < 500);
}

float get_vesc_fallback_position_degrees(void) {
    return vesc_tracked_position_degrees;
}

void update_vesc_position_tracking(float new_position_revs, uint32_t current_time) {
    if (!vesc_tracking_initialized) {
        // Initialize tracking with first valid position
        vesc_tracked_position_revs = new_position_revs;
        vesc_tracking_initialized = true;
    } else {
        // Update tracked position
        vesc_tracked_position_revs = new_position_revs;
    }
    
    // Convert to joint degrees (accounting for gear ratio)
    vesc_tracked_position_degrees = (vesc_tracked_position_revs * 360.0f) / GEAR_RATIO;
    last_vesc_position_update = current_time;
}

// CRSF control task - handles CRSF data processing and motor control coordination
// In event-driven mode, this mainly handles non-critical tasks and coordination
void crsf_control_task(void *pvParameters) {
    #if DEBUG_CRSF_CHANNELS
    uint32_t last_print = 0;
    #endif
    uint32_t last_encoder_print = 0;
    
    // Static variables for rate-limited debug messages  
    uint32_t last_vesc_debug = 0;  // Rate limit VESC debug messages
    
    // Failsafe state tracking
    static bool was_connected = false;
    static bool failsafe_command_sent = false;
    static uint32_t last_failsafe_command = 0;

    ESP_LOGI(TAG, "CRSF control task started");

    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // VESC sends status messages automatically - we just check if we have recent data
        can_status_msg_4 *vesc_status = comm_can_get_status_msg_4_id(CAN_VESC_ID);
        
        // Process VESC Status 4 (position) - this now mainly updates global state
        // The actual control logic is triggered by the CAN callback
        if (vesc_status) {
            uint32_t age_ms = current_time - (vesc_status->rx_time * portTICK_PERIOD_MS);
            if (age_ms < 300) {
                // VESC data is fresh (less than 300ms old)
                vesc_current_position = vesc_status->pid_pos_now;
                vesc_position_valid = true;
                
                // Update position tracking for fallback
                update_vesc_position_tracking(vesc_current_position, current_time);
                
                // Rate limit VESC debug messages to every 2 seconds
                if (current_time - last_vesc_debug > 2000) {
                    DEBUG_VESC("VESC ID %d: Pos=%.3f rev", CAN_VESC_ID, vesc_current_position);
                    last_vesc_debug = current_time;
                }
            } else {
                // VESC data is stale (older than 300ms)
                vesc_position_valid = false;
                // Rate limit VESC debug messages to every 2 seconds
                if (current_time - last_vesc_debug > 2000) {
                    DEBUG_VESC("VESC ID %d data STALE: %lums", CAN_VESC_ID, age_ms);
                    last_vesc_debug = current_time;
                }
            }
        } else {
            // No VESC status message received at all
            vesc_position_valid = false;
            // Rate limit VESC debug messages to every 2 seconds
            if (current_time - last_vesc_debug > 2000) {
                DEBUG_VESC("VESC ID %d missing from CAN", CAN_VESC_ID);
                last_vesc_debug = current_time;
            }
        }
        
        // Debug encoder status periodically
        if (current_time - last_encoder_print > 500) { // Every 500ms
            encoder_update();  // Update encoder data before reading
            if (encoder_is_valid()) {
                #if DEBUG_ENCODER_DATA
                float angle = encoder_get_angle_deg();
                float velocity = encoder_get_velocity_deg_s();
                DEBUG_ENC("Angle: %.2f°, Velocity: %.1f°/s, Errors: %lu", 
                         angle, velocity, encoder_get_error_count());
                #else
                DEBUG_ENC("Valid encoder - Errors: %lu", encoder_get_error_count());
                #endif
            } else {
                DEBUG_ENC("INVALID - errors: %lu", encoder_get_error_count());
            }
            
            last_encoder_print = current_time;
        }
        
        // Check for new CRSF data
        if (crsf_has_new_data()) {
            // Reset failsafe command state when connection is restored
            if (!was_connected) {
                // Connection restored - reset failsafe state for next time
                failsafe_command_sent = false;
                was_connected = true;
            }
            
            // Get all channel values (scaled to 1000-2000)
            crsf_get_all_channels_scaled(channels);
            
            // Print channel data for debugging
            #if DEBUG_CRSF_CHANNELS
            if (current_time - last_print > CRSF_DEBUG_PRINT_RATE_MS) {
                uint32_t last_update = crsf_get_last_update_time();
                uint32_t age_ms = current_time - last_update;
                ESP_LOGI(TAG, "[CRSF] Ch1:%d Ch2:%d Ch3:%d Ch4:%d Ch5:%d Ch6:%d Age:%lums", 
                         channels[0], channels[1], channels[2], channels[3], 
                         channels[4], channels[5], age_ms);
                last_print = current_time;
            }
            #endif
            
            // Channel 6 calibration - sets encoder zero position to REST_ANGLE when activated
            bool channel6_high = (channels[CRSF_CHANNEL_AUX2] > 1500);  // Channel 6 = AUX2
            if (channel6_high && !last_channel6_state) {
                // Channel 6 transitioned from low to high - trigger calibration
                // Rate limit calibration to prevent accidental repeated triggers
                if (current_time - last_calibration_time > 2000) {  // Rate limit to once per 2 seconds
                    ESP_LOGI(TAG, "[CALIBRATION] Channel 6 triggered - calibrating encoder to %.1f degrees", REST_ANGLE);
                    
                    #ifdef ENCODER_TYPE_VESC_INTERNAL
                    // For VESC internal encoder, use the VESC's native position offset command
                    // This tells the VESC to adjust its position reference point
                    wait_for_safe_can_slot();
                    comm_can_update_pid_pos_offset(CAN_VESC_ID, REST_ANGLE, true);
                    ESP_LOGI(TAG, "[CALIBRATION] VESC position offset updated to %.1f degrees", REST_ANGLE);
                    #else
                    // For external encoders, calibrate the encoder directly
                    if (encoder_set_zero_position(REST_ANGLE)) {
                        ESP_LOGI(TAG, "[CALIBRATION] Encoder successfully calibrated to %.1f degrees", REST_ANGLE);
                    } else {
                        ESP_LOGW(TAG, "[CALIBRATION] Encoder calibration failed");
                    }
                    #endif
                    
                    last_calibration_time = current_time;
                } else {
                    ESP_LOGW(TAG, "[CALIBRATION] Ignoring calibration request (rate limited)");
                }
            }
            last_channel6_state = channel6_high;
            
        } else {
            // No connection - apply safety behavior
            if (was_connected) {
                // Just lost connection - reset failsafe state to send command immediately
                failsafe_command_sent = false;
                was_connected = false;
            }
                
            // Update ESP-NOW telemetry even during failsafe (with default CRSF target)
            #if ESP_NOW_TELEMETRY_ENABLE && !DEBUG_ESPNOW_TEST
            encoder_update();  // Update encoder data before reading
            float encoder_degrees = encoder_get_angle_deg();
            float crsf_target_degrees = (MIN_ANGLE + MAX_ANGLE) / 2.0f; // Center position during failsafe
            float vesc_target_revolutions = 0.0f; // No target during failsafe
            telemetry_espnow_set_payload_data(BOARD_NAME, encoder_degrees, crsf_target_degrees, vesc_target_revolutions);
            #endif
            
            // Apply failsafe brake/current command only periodically, not every loop
            // Send failsafe command immediately when entering failsafe, then every 1 second as keepalive
            if (!failsafe_command_sent || (current_time - last_failsafe_command > 1000)) {
                if (CRSF_FAILSAFE_ENABLE_BRAKE) {
                    wait_for_safe_can_slot();
                    DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT_BRAKE) to VESC_ID=%d, value=%.3f [FAILSAFE]", CAN_PACKET_SET_CURRENT_BRAKE, CAN_VESC_ID, CRSF_FAILSAFE_BRAKE_CURRENT);
                    comm_can_set_current_brake(CAN_VESC_ID, CRSF_FAILSAFE_BRAKE_CURRENT);
                } else {
                    wait_for_safe_can_slot();
                    DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [FAILSAFE]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, CRSF_FAILSAFE_CURRENT);
                    comm_can_set_current(CAN_VESC_ID, CRSF_FAILSAFE_CURRENT);
                }
                last_failsafe_command = current_time;
                failsafe_command_sent = true;
            }
        }
        
        // Periodic CAN status request to ensure VESC keeps sending STATUS_4 messages
        // This is critical when VESC isn't actively running motor control
        static uint32_t last_status_request = 0;
        if (current_time - last_status_request > 5000) { // Send request every 5 seconds (like backup)
            wait_for_safe_can_slot();
            
            // Method 1: CAN ping to verify communication
            HW_TYPE hw_type;
            bool ping_success = comm_can_ping(CAN_VESC_ID, &hw_type);
            
            // Method 2: Send COMM_GET_VALUES request to trigger status response
            // This is the standard VESC command to request all current values
            uint8_t get_values_cmd = 4; // COMM_GET_VALUES
            comm_can_send_buffer(CAN_VESC_ID, &get_values_cmd, 1, 0);
            
            // Method 3: Send a minimal duty cycle command as backup
            comm_can_set_duty(CAN_VESC_ID, 0.0f);  // 0% duty = no motor movement
            
            last_status_request = current_time;
            
            // Debug: Rate limit request debug messages to every 2 seconds
            static uint32_t last_request_debug = 0;
            if (current_time - last_request_debug > 2000) {
                DEBUG_CAN_CMD("Periodic CAN requests to VESC_ID=%d (ping: %s, GET_VALUES sent, duty: 0%%)", 
                             CAN_VESC_ID, ping_success ? "OK" : "FAIL");
                last_request_debug = current_time;
            }
        }
        
        // ESP-NOW test function - sends random data when DEBUG_ESPNOW_TEST is enabled
        #if DEBUG_ESPNOW_TEST
        #if ESP_NOW_TELEMETRY_ENABLE
        espnow_test_send_random_data();
        #endif
        #endif
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
    }
}

// Control function called from CAN Status 4 callback - executes position control logic
// This ensures control calculations are synchronized with fresh VESC position data
// NOTE: This function only runs when STATUS_4 messages are received from the VESC.
// The crsf_control_task includes periodic ping to ensure STATUS_4 messages continue
// even when the VESC isn't actively running motor control.
void main_process_control_logic(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    static uint32_t last_can_cmd_debug = 0;
    static uint32_t last_position_debug = 0;
    
    // Only proceed if we have CRSF connection and valid VESC data
    if (!crsf_is_connected()) {
        return; // No CRSF connection - nothing to do
    }
    
    // Check if armed using utility function
    if (crsf_is_armed()) {
        // Armed mode - send actual motor commands
        if (vesc_position_valid) {
            // Convert CRSF channel to target angle (using board-configured control channel)
            // CRSF channels are normalized -1.0 to +1.0, convert to 0.0 to 1.0, then to board-specific angle range
            float normalized_input = (crsf_channel_to_normalized(CONTROL_CHANNEL) + 1.0f) / 2.0f; // Convert -1..+1 to 0..1
            float angle_range = MAX_ANGLE - MIN_ANGLE;
            float crsf_target_degrees = MIN_ANGLE + (normalized_input * angle_range);
            
            // Get current position feedback - use encoder if valid, otherwise VESC fallback
            float current_position_degrees;
            // Update encoder data first
            encoder_update();
            
            bool using_encoder_feedback = encoder_is_valid();
            
            if (using_encoder_feedback) {
                // Use encoder feedback for closed-loop control
                current_position_degrees = encoder_get_angle_deg();
            } else if (is_vesc_tracking_valid(current_time)) {
                // Use VESC position tracking as fallback
                current_position_degrees = get_vesc_fallback_position_degrees();
                // Rate limited warning about using fallback
                static uint32_t last_fallback_warning = 0;
                if (current_time - last_fallback_warning > 5000) {
                    ESP_LOGW(TAG, "[FALLBACK] Using VESC position feedback (encoder invalid)");
                    last_fallback_warning = current_time;
                }
            } else {
                // No valid position feedback available - skip control
                DEBUG_POS("No valid position feedback available (encoder invalid, VESC tracking stale)");
                return;
            }
            
            // Calculate position error (CRSF target - current position)
            float position_error = crsf_target_degrees - current_position_degrees;
            
            // Apply gear ratio compensation
            float gear_compensated_error = position_error * GEAR_RATIO;
            
            // Calculate new target position for VESC (in revolutions)
            // vesc_current_position is already in revolutions, so convert degrees error to revolutions
            float vesc_target_position_revolutions = vesc_current_position - (gear_compensated_error / 360.0f);
            
            // Send position command to VESC (wait for safe CAN slot to avoid collisions)
            wait_for_safe_can_slot();
            
            // Rate limit CAN command debug to every 2 seconds
            if (current_time - last_can_cmd_debug > 2000) {
                DEBUG_CAN_CMD("CMD_ID=%d (SET_POS_FLOATINGPOINT) to VESC_ID=%d, value=%.6f", CAN_PACKET_SET_POS_FLOATINGPOINT, CAN_VESC_ID, vesc_target_position_revolutions);
                last_can_cmd_debug = current_time;
            }
            comm_can_set_pos_floatingpoint(CAN_VESC_ID, vesc_target_position_revolutions);
            
            // Update VESC motion parameters periodically (after position control)
            update_vesc_motion_parameters(current_time);
            
            // Update ESP-NOW telemetry data with current system values
            #if ESP_NOW_TELEMETRY_ENABLE && !DEBUG_ESPNOW_TEST
            telemetry_espnow_set_payload_data(BOARD_NAME, current_position_degrees, crsf_target_degrees, vesc_target_position_revolutions);
            #endif
            
            // Rate limit position control debug messages to every 1 second
            if (current_time - last_position_debug > 1000) {
                #if DEBUG_POSITION_CONTROL
                const char* feedback_source = using_encoder_feedback ? "Encoder" : "VESC";
                DEBUG_POS("CRSF: %.1f°, %s: %.1f°, Error: %.1f°, VESC Target: %.6f rev, Armed=YES", 
                        crsf_target_degrees, feedback_source, current_position_degrees, position_error,
                        vesc_target_position_revolutions);
                #endif
                last_position_debug = current_time;
            }
        } else {
            // No valid VESC position - SAFETY: stop motor (wait for safe CAN slot)
            ESP_LOGW(TAG, "[SAFETY] No valid VESC position data - stopping motor for safety");
            wait_for_safe_can_slot();
            DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [NO_POSITION_STOP]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, 0.0f);
            comm_can_set_current(CAN_VESC_ID, 0.0f);
            // Rate limit safety debug messages to every 1 second
            if (current_time - last_position_debug > 1000) {
                DEBUG_POS("Motor stopped - no valid position feedback (Armed=YES but unsafe)");
                last_position_debug = current_time;
            }
        }
    } else {
        // Disarmed mode - show what position control would do for debugging
        if (vesc_position_valid) {
            // Convert CRSF channel to target angle (using board-configured control channel)
            float normalized_input = (crsf_channel_to_normalized(CONTROL_CHANNEL) + 1.0f) / 2.0f; // Convert -1..+1 to 0..1
            float angle_range = MAX_ANGLE - MIN_ANGLE;
            float crsf_target_degrees = MIN_ANGLE + (normalized_input * angle_range);
            
            // Update encoder data first
            encoder_update();
            
            // Get current position feedback - use encoder if valid, otherwise VESC fallback  
            float current_position_degrees;
            bool using_encoder_feedback = encoder_is_valid();
            
            if (using_encoder_feedback) {
                current_position_degrees = encoder_get_angle_deg();
            } else if (is_vesc_tracking_valid(current_time)) {
                current_position_degrees = get_vesc_fallback_position_degrees();
            } else {
                // Use invalid marker for debug display when no position feedback available
                current_position_degrees = -999.0f;
            }
            
            // Calculate position error (CRSF target - current position)
            float position_error = crsf_target_degrees - current_position_degrees;
            // Apply gear ratio compensation
            float gear_compensated_error = position_error * GEAR_RATIO;
            // Calculate new target position for VESC (in revolutions)
            // vesc_current_position is already in revolutions, so convert degrees error to revolutions
            float vesc_target_position_revolutions = vesc_current_position - (gear_compensated_error / 360.0f);
            
            // Update ESP-NOW telemetry data with current system values (even when disarmed)
            #if ESP_NOW_TELEMETRY_ENABLE && !DEBUG_ESPNOW_TEST
            telemetry_espnow_set_payload_data(BOARD_NAME, current_position_degrees, crsf_target_degrees, vesc_target_position_revolutions);
            #endif
            
            // Rate limit position control debug messages to every 1 second
            if (current_time - last_position_debug > 1000) {
                #if DEBUG_POSITION_CONTROL
                const char* feedback_source = using_encoder_feedback ? "Encoder" : (current_position_degrees == -999.0f ? "NONE" : "VESC");
                DEBUG_POS("CRSF: %.1f°, %s: %.1f°, Error: %.1f°, VESC Target: %.6f rev, Armed=NO", 
                        crsf_target_degrees, feedback_source, current_position_degrees, position_error,
                        vesc_target_position_revolutions);
                #endif
                last_position_debug = current_time;
            }
        }
        
        // Disarmed - send zero current
        // Rate limit CAN command debug to every 2 seconds
        if (current_time - last_can_cmd_debug > 2000) {
            DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [DISARMED]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, 0.0f);
            last_can_cmd_debug = current_time;
        }
        comm_can_set_current(CAN_VESC_ID, 0.0f);
    }
}

// Hardware initialization and CRC functions (standard VESC implementation)
uint32_t main_calc_hw_crc(void) {
    uint32_t crc = 0;
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    return crc;
}

void main_store_backup_data(void) {
    // Backup data is statically initialized, no need to store
}

bool main_init_done(void) {
    return true;
}

void main_wait_until_init_done(void) {
    // Wait for initialization to complete
}

// ESP-NOW initialization task (runs once then deletes itself)
#if ESP_NOW_TELEMETRY_ENABLE
void espnow_init_task(void *pvParameters) {
    ESP_LOGI(TAG, "ESP-NOW initialization task starting...");
    
    // Initialize ESP-NOW with retry mechanism
    int retry_count = 0;
    const int max_retries = 3;
    
    while (retry_count < max_retries) {
        esp_err_t ret = telemetry_espnow_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ESP-NOW initialized successfully on attempt %d", retry_count + 1);
            break;
        } else {
            retry_count++;
            ESP_LOGW(TAG, "ESP-NOW init failed (attempt %d/%d): %s", retry_count, max_retries, esp_err_to_name(ret));
            if (retry_count < max_retries) {
                vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second before retry
            }
        }
    }
    
    if (retry_count >= max_retries) {
        ESP_LOGE(TAG, "ESP-NOW initialization failed after %d attempts - telemetry disabled", max_retries);
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialization task completed - deleting self");
    vTaskDelete(NULL);  // Delete this task
}
#endif

void app_main(void) {
    ESP_LOGI(TAG, "VESC Express starting...");
    ESP_LOGI(TAG, "Board: %s, VESC ID: %d", BOARD_NAME, CAN_VESC_ID);
    
    // Initialize NVS for configuration storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS Flash initialized");
    ESP_LOGI(TAG, "Backup config initialized - ESP32 CAN ID: %d, Target VESC ID: %d", 
             CAN_ESP32_ID, CAN_VESC_ID);

    // Initialize WiFi/networking if needed by ESP-NOW
    #if ESP_NOW_TELEMETRY_ENABLE
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Set WiFi channel for ESP-NOW (both devices must use same channel)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    // Wait for WiFi interface to be fully ready
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "WiFi initialized for ESP-NOW (Channel 1, Station Mode)");
    #endif

    // Initialize CAN interface
    comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
    ESP_LOGI(TAG, "CAN interface initialized on TX:%d, RX:%d", CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
    
    // Debug initialization countdown - allows time for hardware to stabilize before encoder init
    #if DEBUG_INIT_COUNTDOWN
    ESP_LOGI(TAG, "=== DEBUG INITIALIZATION COUNTDOWN ENABLED ===");
    ESP_LOGI(TAG, "Starting 30-second hardware stabilization countdown before encoder initialization...");
    for (int i = 30; i > 0; i--) {
        if (i % 5 == 0 || i <= 10) {  // Print every 5 seconds, or every second for last 10
            ESP_LOGI(TAG, "Countdown: %d seconds remaining...", i);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second
    }
    ESP_LOGI(TAG, "=== COUNTDOWN COMPLETE - INITIALIZING ENCODER ===");
    #endif

    // Initialize encoder system based on board configuration
    ESP_LOGI(TAG, "Initializing encoder system...");
    
    #if ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC
    ESP_LOGI(TAG, "Encoder config - Type: SPI_MAGNETIC (AS504x series)");
    ESP_LOGI(TAG, "SPI pins - CS: GPIO%d, MISO: GPIO%d, MOSI: GPIO%d, CLK: GPIO%d", 
             ENCODER_SPI_CS_PIN, ENCODER_SPI_MISO_PIN, ENCODER_SPI_MOSI_PIN, ENCODER_SPI_CLK_PIN);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "SPI magnetic encoder system initialized successfully");
        vTaskDelay(pdMS_TO_TICKS(200));
        encoder_update();  // Update encoder data before reading
        DEBUG_ENC("SPI Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO", encoder_get_angle_deg());
        
        if (encoder_is_valid()) {
            ESP_LOGI(TAG, "SPI magnetic encoder ready - %s", encoder_get_type_name());
        } else {
            ESP_LOGW(TAG, "SPI magnetic encoder initialized but data not yet valid");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize SPI magnetic encoder system");
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_PWM_MAGNETIC
    ESP_LOGI(TAG, "Encoder config - Type: PWM_MAGNETIC");
    ESP_LOGI(TAG, "PWM pin: GPIO%d, Range: %d-%d us", 
             ENCODER_PWM_PIN, ENCODER_PWM_MIN_US, ENCODER_PWM_MAX_US);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "PWM magnetic encoder system initialized successfully");
        vTaskDelay(pdMS_TO_TICKS(100));
        encoder_update();  // Update encoder data before reading
        DEBUG_ENC("PWM Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO", encoder_get_angle_deg());
    } else {
        ESP_LOGE(TAG, "Failed to initialize PWM magnetic encoder system");
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_QUADRATURE
    ESP_LOGI(TAG, "Encoder config - Type: QUADRATURE, PPR: %d", ENCODER_PPR);
    ESP_LOGI(TAG, "Quadrature pins - A: GPIO%d, B: GPIO%d", ENCODER_A_PIN, ENCODER_B_PIN);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "Quadrature encoder system initialized successfully");
        vTaskDelay(pdMS_TO_TICKS(100));
        encoder_update();  // Update encoder data before reading
        DEBUG_ENC("Quadrature Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO", encoder_get_angle_deg());
    } else {
        ESP_LOGE(TAG, "Failed to initialize quadrature encoder system");
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_VESC_INTERNAL
    ESP_LOGI(TAG, "VESC internal encoder configured - using CAN position feedback");
    ESP_LOGI(TAG, "Gear ratio: %.1f:1 for position conversion", GEAR_RATIO);
    
    #elif ENCODER_TYPE == ENCODER_TYPE_NONE
    ESP_LOGW(TAG, "No encoder configured - position feedback disabled");
    ESP_LOGW(TAG, "System will operate in current control mode only");
    
    #else
    ESP_LOGE(TAG, "Unknown encoder type: %d - check board_config.h", ENCODER_TYPE);
    return;
    #endif

    ESP_LOGI(TAG, "Encoder initialization phase completed");

    // Initialize CRSF receiver
    crsf_init(CRSF_UART_NUM, CRSF_TX_PIN, CRSF_RX_PIN, CRSF_BAUDRATE);
    ESP_LOGI(TAG, "CRSF receiver initialized");
    
    // Create CRSF control task
    xTaskCreate(crsf_control_task, "crsf_control", CRSF_TASK_STACK_SIZE, NULL, CRSF_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "CRSF control task created");
    
    // Create ESP-NOW initialization task with large stack (runs once then deletes itself)
    #if ESP_NOW_TELEMETRY_ENABLE
    ESP_LOGI(TAG, "ESP-NOW telemetry ENABLED - Creating initialization task...");
    xTaskCreate(espnow_init_task, "espnow_init", 8192, NULL, 3, NULL);
    #else
    ESP_LOGI(TAG, "ESP-NOW telemetry DISABLED in board configuration");
    #endif
    
    ESP_LOGI(TAG, "Initialization complete - System ready");
    ESP_LOGI(TAG, "Hybrid control: Periodic ping + Event-driven VESC Status 4 position control");
    ESP_LOGI(TAG, "Angle Range: %.1f° to %.1f°", MIN_ANGLE, MAX_ANGLE);
}
