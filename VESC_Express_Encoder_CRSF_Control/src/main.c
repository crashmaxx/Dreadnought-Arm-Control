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
   
#include "comm/comm_can.h"
#include "comm/crsf_receiver.h"
#include "comm/crsf_utils.h"
#include "comm/crsf_config.h"
#include "comm/comm_espnow.h"
#include "drivers/encoder_interface.h"
#include "board_config.h"
#include "debug_config.h"  // Debug configuration and macros
#include "conf_general.h"
#include "hw.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

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

#if DEBUG_CRSF_CHANNELS
#define DEBUG_CRSF(fmt, ...) ESP_LOGI(TAG, "[CRSF] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_CRSF(fmt, ...)
#endif

#if DEBUG_CAN_COMMANDS
#define DEBUG_CAN_CMD(fmt, ...) ESP_LOGI(TAG, "[CAN_CMD] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_CAN_CMD(fmt, ...)
#endif

#if DEBUG_ESPNOW_TELEMETRY
#define DEBUG_ESPNOW(fmt, ...) ESP_LOGI(TAG, "[ESPNOW] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_ESPNOW(fmt, ...)
#endif

static const char *TAG = "MAIN";

// Task to handle CRSF data and control
static void crsf_control_task(void *pvParameters) {
    uint16_t channels[16];
    uint32_t last_print = 0;
    uint32_t last_encoder_print = 0;
    uint32_t last_vesc_debug = 0;  // Rate limit VESC debug messages
    uint32_t last_position_debug = 0;  // Rate limit position control debug messages
    uint32_t last_can_cmd_debug = 0;  // Rate limit CAN command debug messages
    uint32_t last_vesc_params_update = 0;  // Rate limit VESC parameter updates
    
    // VESC position control variables
    float vesc_current_position = 0.0f;
    bool vesc_position_valid = false;
    
    ESP_LOGI(TAG, "CRSF control task start//ed with %d bytes stack", CRSF_TASK_STACK_SIZE);
    
    while (1) {
        // Update CRSF receiver
        crsf_update();
        
        // Update encoder
        encoder_update();
        
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Request VESC position every 200ms for responsive control
        // VESC sends status messages automatically - we just check if we have recent data
        can_status_msg *vesc_status_1 = comm_can_get_status_msg_id(CAN_VESC_ID);
        can_status_msg_4 *vesc_status = comm_can_get_status_msg_4_id(CAN_VESC_ID);
        
        // Check for VESC faults and status
        bool vesc_has_fault = false;
        float vesc_rpm = 0.0f;
        float vesc_current = 0.0f;
        float vesc_duty = 0.0f;
        float vesc_temp_fet = 0.0f;
        float vesc_temp_motor = 0.0f;
        
        if (vesc_status_1) {
            uint32_t age_ms_1 = current_time - (vesc_status_1->rx_time * portTICK_PERIOD_MS);
            if (age_ms_1 < 500) {
                vesc_rpm = vesc_status_1->rpm;
                vesc_current = vesc_status_1->current;
                vesc_duty = vesc_status_1->duty;
                
                // Check for basic fault conditions
                if (fabsf(vesc_current) > 80.0f) {  // High current fault
                    vesc_has_fault = true;
                    ESP_LOGW(TAG, "[VESC_FAULT] High current: %.2f A", vesc_current);
                }
                if (fabsf(vesc_duty) > 0.95f) {  // Duty cycle saturation
                    vesc_has_fault = true;
                    ESP_LOGW(TAG, "[VESC_FAULT] Duty saturation: %.2f%%", vesc_duty * 100.0f);
                }
            }
        }
        
        if (vesc_status) {
            uint32_t age_ms = current_time - (vesc_status->rx_time * portTICK_PERIOD_MS);
            if (age_ms < 300) {
                // We have recent VESC position data (within 300ms)
                vesc_current_position = vesc_status->pid_pos_now;
                vesc_position_valid = true;
                vesc_temp_fet = vesc_status->temp_fet;
                vesc_temp_motor = vesc_status->temp_motor;
                
                // Check for temperature faults
                if (vesc_temp_fet > 80.0f) {  // FET overtemp
                    vesc_has_fault = true;
                    ESP_LOGW(TAG, "[VESC_FAULT] FET overtemp: %.1f°C", vesc_temp_fet);
                }
                if (vesc_temp_motor > 100.0f) {  // Motor overtemp
                    vesc_has_fault = true;
                    ESP_LOGW(TAG, "[VESC_FAULT] Motor overtemp: %.1f°C", vesc_temp_motor);
                }
                
                // Periodically send VESC motion parameters (every 10 seconds after receiving status 4)
                if (current_time - last_vesc_params_update > 10000) {
                    DEBUG_VESC("Updating VESC motion parameters: Vel=%.0f, Accel=%.0f, Decel=%.0f", 
                              MAX_VEL, MAX_ACCEL, MAX_DECEL);
                    
                    // Send maximum velocity (ERPM units - RPM of the motor)
                    comm_can_set_max_sp_vel(CAN_VESC_ID, MAX_VEL);
                    DEBUG_VESC("Sent MAX_VEL=%.0f to VESC ID %d", MAX_VEL, CAN_VESC_ID);
                    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between CAN commands
                    
                    // Send maximum acceleration (ERPM/s units)
                    comm_can_set_max_sp_accel(CAN_VESC_ID, MAX_ACCEL);
                    DEBUG_VESC("Sent MAX_ACCEL=%.0f to VESC ID %d", MAX_ACCEL, CAN_VESC_ID);
                    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between CAN commands
                    
                    // Send maximum deceleration (ERPM/s units)
                    comm_can_set_max_sp_decel(CAN_VESC_ID, MAX_DECEL);
                    DEBUG_VESC("Sent MAX_DECEL=%.0f to VESC ID %d", MAX_DECEL, CAN_VESC_ID);
                    
                    last_vesc_params_update = current_time;
                }
                
                // Rate limit VESC debug messages to every 2 seconds
                if (current_time - last_vesc_debug > 2000) {
                    DEBUG_VESC("VESC ID %d - pos: %.3f rev, RPM: %.0f, Current: %.2fA, Temp: %.1f°C, Fault: %s", 
                              CAN_VESC_ID, vesc_current_position, vesc_rpm, vesc_current, vesc_temp_fet,
                              vesc_has_fault ? "YES" : "NO");
                    if (vesc_has_fault) {
                        ESP_LOGW(TAG, "[VESC_STATUS] ID %d Detailed - RPM: %.0f, Current: %.2fA, Duty: %.1f%%, Temp FET: %.1f°C, Temp Motor: %.1f°C", 
                                 CAN_VESC_ID, vesc_rpm, vesc_current, vesc_duty * 100.0f, vesc_temp_fet, vesc_temp_motor);
                    }
                    last_vesc_debug = current_time;
                }
            } else {
                // VESC position data is stale
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
        
        // Print encoder data for debugging (independent of CRSF)
        if (current_time - last_encoder_print > 500) { // Every 500ms
            if (encoder_is_valid()) {
                float angle = encoder_get_angle_deg();
                float velocity = encoder_get_velocity_deg_s();
                DEBUG_ENC("Angle: %.2f°, Velocity: %.1f°/s, Errors: %lu", 
                         angle, velocity, encoder_get_error_count());
            } else {
                DEBUG_ENC("INVALID - errors: %lu", encoder_get_error_count());
            }
            
            last_encoder_print = current_time;
        }
        
        // Check for new CRSF data
        if (crsf_has_new_data()) {
            // Get all channel values (scaled to 1000-2000)
            crsf_get_all_channels_scaled(channels);
            
            // Print channel data for debugging
            if (current_time - last_print > CRSF_DEBUG_PRINT_RATE_MS) {
                #if DEBUG_CRSF_CHANNELS
                uint32_t last_update = crsf_get_last_update_time();
                uint32_t age_ms = current_time - last_update;
                #endif
                DEBUG_CRSF("Channels: CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d Connected=%s Armed=%s Age=%lums",
                         channels[0], channels[1], channels[2], channels[3], channels[4],
                         crsf_is_connected() ? "YES" : "NO",
                         crsf_is_armed() ? "YES" : "NO",
                         #if DEBUG_CRSF_CHANNELS
                         age_ms
                         #else
                         (uint32_t)(current_time - last_update)
                         #endif
                         );
                last_print = current_time;
            }
            
            // Send motor commands based on CRSF input and encoder feedback
            // Normal operation - send actual motor commands
            if (crsf_is_connected()) {
                // Check if armed using utility function
                if (crsf_is_armed()) {
                    
                    // Position control logic
                    // Only proceed with position control if we have valid VESC position AND no faults
                    if (vesc_position_valid && !vesc_has_fault) {
                        // Convert CRSF channel to target angle (using board-configured control channel)
                        // CRSF channels are normalized -1.0 to +1.0, convert to 0.0 to 1.0, then to board-specific angle range
                        float normalized_input = (crsf_channel_to_normalized(CONTROL_CHANNEL) + 1.0f) / 2.0f; // Convert -1..+1 to 0..1
                        float angle_range = MAX_ANGLE - MIN_ANGLE;
                        float crsf_target_degrees = MIN_ANGLE + (normalized_input * angle_range);
                        
                        // Get current encoder position in degrees
                        float encoder_degrees = encoder_get_angle_deg();
                        
                        // Calculate position error (CRSF target - encoder position)
                        float position_error = crsf_target_degrees - encoder_degrees;
                        
                        // Apply gear ratio compensation
                        float gear_compensated_error = position_error * GEAR_RATIO;
                        
                        // Calculate new target position for VESC (in revolutions)
                        // vesc_current_position is already in revolutions, so convert degrees error to revolutions
                        float vesc_target_position_revolutions = vesc_current_position - (gear_compensated_error / 360.0f);
                        
                        // Send position command to VESC (in revolutions using floating point)
                        // Rate limit CAN command debug to every 2 seconds
                        if (current_time - last_can_cmd_debug > 2000) {
                            DEBUG_CAN_CMD("CMD_ID=%d (SET_POS_FLOATINGPOINT) to VESC_ID=%d, value=%.6f", CAN_PACKET_SET_POS_FLOATINGPOINT, CAN_VESC_ID, vesc_target_position_revolutions);
                            last_can_cmd_debug = current_time;
                        }
                        comm_can_set_pos_floatingpoint(CAN_VESC_ID, vesc_target_position_revolutions);
                        
                        // Update ESP-NOW telemetry data with current system values
                        #if ESP_NOW_TELEMETRY_ENABLE
                        telemetry_espnow_set_payload_data(BOARD_NAME, encoder_degrees, crsf_target_degrees, vesc_target_position_revolutions);
                        #endif
                        
                        // Rate limit position control debug messages to every 1 second
                        if (current_time - last_position_debug > 1000) {
                            DEBUG_POS("CRSF: %.1f°, Encoder: %.1f°, Error: %.1f°, VESC Target: %.6f rev, RPM: %.0f, Armed=YES", 
                                    crsf_target_degrees, encoder_degrees, position_error,
                                    vesc_target_position_revolutions, vesc_rpm);
                            last_position_debug = current_time;
                        }
                    } else if (vesc_has_fault) {
                        // VESC has fault - stop motor for safety
                        ESP_LOGW(TAG, "[SAFETY] VESC fault detected - stopping motor");
                        DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [FAULT_STOP]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, 0.0f);
                        comm_can_set_current(CAN_VESC_ID, 0.0f);
                    } else {
                        // No valid VESC position - fallback to current control
                        float throttle_normalized = crsf_channel_to_normalized(CRSF_THROTTLE_CHANNEL);
                        float current_command = throttle_normalized * CRSF_MAX_CURRENT_AMPS;
                        // Rate limit CAN command debug to every 2 seconds
                        if (current_time - last_can_cmd_debug > 2000) {
                            DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [FALLBACK]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, current_command);
                            last_can_cmd_debug = current_time;
                        }
                        comm_can_set_current(CAN_VESC_ID, current_command);
                        // Rate limit current control debug messages to every 1 second
                        if (current_time - last_position_debug > 1000) {
                            DEBUG_POS("Current control: %.2f A (throttle: %.2f), Armed=YES", current_command, throttle_normalized);
                            last_position_debug = current_time;
                        }
                    }
                } else {
                    // Show what position control would do if armed
                    if (vesc_position_valid && !vesc_has_fault) {
                        // Convert CRSF channel to target angle (using board-configured control channel)
                        float normalized_input = (crsf_channel_to_normalized(CONTROL_CHANNEL) + 1.0f) / 2.0f; // Convert -1..+1 to 0..1
                        float angle_range = MAX_ANGLE - MIN_ANGLE;
                        float crsf_target_degrees = MIN_ANGLE + (normalized_input * angle_range);
                        
                        // Get current encoder position in degrees
                        float encoder_degrees = encoder_get_angle_deg();
                        
                        // Calculate position error (CRSF target - encoder position)
                        float position_error = crsf_target_degrees - encoder_degrees;
                        // Apply gear ratio compensation
                        float gear_compensated_error = position_error * GEAR_RATIO;
                        // Calculate new target position for VESC (in revolutions)
                        // vesc_current_position is already in revolutions, so convert degrees error to revolutions
                        float vesc_target_position_revolutions = vesc_current_position - (gear_compensated_error / 360.0f);
                        
                        // Update ESP-NOW telemetry data with current system values (even when disarmed)
                        #if ESP_NOW_TELEMETRY_ENABLE
                        telemetry_espnow_set_payload_data(BOARD_NAME, encoder_degrees, crsf_target_degrees, vesc_target_position_revolutions);
                        #endif
                        
                        // Rate limit position control debug messages to every 1 second
                        if (current_time - last_position_debug > 1000) {
                            DEBUG_POS("CRSF: %.1f°, Encoder: %.1f°, Error: %.1f°, VESC Target: %.6f rev, RPM: %.0f, Armed=NO", 
                                    crsf_target_degrees, encoder_degrees, position_error,
                                    vesc_target_position_revolutions, vesc_rpm);
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
            } else {
                // No connection - apply safety behavior
                
                // Update ESP-NOW telemetry even during failsafe (with default CRSF target)
                #if ESP_NOW_TELEMETRY_ENABLE
                float encoder_degrees = encoder_get_angle_deg();
                float crsf_target_degrees = (MIN_ANGLE + MAX_ANGLE) / 2.0f; // Center position during failsafe
                float vesc_target_revolutions = 0.0f; // No target during failsafe
                telemetry_espnow_set_payload_data(BOARD_NAME, encoder_degrees, crsf_target_degrees, vesc_target_revolutions);
                #endif
                
                if (CRSF_FAILSAFE_ENABLE_BRAKE) {
                    DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT_BRAKE) to VESC_ID=%d, value=%.3f [FAILSAFE]", CAN_PACKET_SET_CURRENT_BRAKE, CAN_VESC_ID, CRSF_FAILSAFE_BRAKE_CURRENT);
                    comm_can_set_current_brake(CAN_VESC_ID, CRSF_FAILSAFE_BRAKE_CURRENT);
                } else {
                    DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [FAILSAFE]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, CRSF_FAILSAFE_CURRENT);
                    comm_can_set_current(CAN_VESC_ID, CRSF_FAILSAFE_CURRENT);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
    }
}

#if ESP_NOW_TELEMETRY_ENABLE
// ESP-NOW initialization task - runs with larger stack to avoid overflow
static void espnow_init_task(void *pvParameters) {
    
    ESP_LOGI(TAG, "[ESP-NOW] Starting ESP-NOW initialization task...");
    
    // Initialize NVS (required for WiFi)
    ESP_LOGI(TAG, "[ESP-NOW] Starting NVS initialization...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "[ESP-NOW] NVS needs erase and reinit...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "[ESP-NOW] NVS initialized successfully");
    
    // Initialize WiFi (required for ESP-NOW)
    ESP_LOGI(TAG, "[ESP-NOW] Starting WiFi initialization...");
    telemetry_wifi_init();
    ESP_LOGI(TAG, "[ESP-NOW] WiFi initialized successfully");
    
    ESP_LOGI(TAG, "[ESP-NOW] Starting ESP-NOW protocol initialization...");
    if (telemetry_espnow_init() == ESP_OK) {
        ESP_LOGI(TAG, "[ESP-NOW] ESP-NOW telemetry initialized successfully - sending every 500ms");
        ESP_LOGI(TAG, "[ESP-NOW] Configured for unicast communication to peer");
    } else {
        ESP_LOGE(TAG, "[ESP-NOW] Failed to initialize ESP-NOW telemetry");
    }
    
    ESP_LOGI(TAG, "[ESP-NOW] Initialization task complete - deleting task");
    vTaskDelete(NULL);  // Delete this task when done
}
#endif

void app_main(void) {
    ESP_LOGI(TAG, "=== VESC Express Encoder CRSF Control Starting ===");
    ESP_LOGI(TAG, "Firmware Version: %d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_TEST_VERSION_NUMBER);
    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "ESP32 CAN ID: %d", CAN_ESP32_ID);
    ESP_LOGI(TAG, "Target VESC CAN ID: %d", CAN_VESC_ID);
    
    // Initialize CAN communication
    #ifdef CAN_TX_GPIO_NUM
    ESP_LOGI(TAG, "Initializing CAN bus on TX: GPIO%d, RX: GPIO%d", CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
    comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
    ESP_LOGI(TAG, "CAN communication initialization completed");
    ESP_LOGI(TAG, "Looking for VESC at CAN ID %d", CAN_VESC_ID);
    // Small delay to allow CAN bus to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    #else
    ESP_LOGW(TAG, "CAN pins not defined - CAN communication disabled");
    #endif

    // ============================================================================
    // SENSOR INITIALIZATION WITH OPTIONAL DELAY
    // ============================================================================
    #if DEBUG_INIT_COUNTDOWN
    ESP_LOGI(TAG, "Starting sensor initialization in 30 seconds...");
    ESP_LOGI(TAG, "This delay allows you to see initial system status");
    
    for (int i = 30; i > 0; i--) {
        ESP_LOGI(TAG, "Sensor initialization in %d seconds...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "Proceeding with sensor initialization...");
    #else
    ESP_LOGI(TAG, "Starting sensor initialization...");
    #endif

    // Initialize encoder system based on board configuration
    #if ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID
    ESP_LOGI(TAG, "Encoder config - Type: DUAL_HYBRID, PPR: %d", ENCODER_PPR);
    ESP_LOGI(TAG, "Encoder pins - PWM: GPIO%d, A: GPIO%d, B: GPIO%d", 
             ENCODER_PWM_PIN, ENCODER_A_PIN, ENCODER_B_PIN);
    ESP_LOGI(TAG, "PWM range: %d-%d us", ENCODER_PWM_MIN_US, ENCODER_PWM_MAX_US);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "Dual hybrid encoder system initialized successfully");
        // Wait a moment for encoder to stabilize
        vTaskDelay(pdMS_TO_TICKS(100));
        DEBUG_ENC("Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO",
                 encoder_get_angle_deg());
    } else {
        ESP_LOGE(TAG, "Failed to initialize dual hybrid encoder system");
        DEBUG_ENC("Dual hybrid encoder initialization FAILED - check hardware connections");
        ESP_LOGE(TAG, "Check pins: PWM=GPIO%d, A=GPIO%d, B=GPIO%d", 
                 ENCODER_PWM_PIN, ENCODER_A_PIN, ENCODER_B_PIN);
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC
    ESP_LOGI(TAG, "Encoder config - Type: SPI_MAGNETIC (AS504x series)");
    ESP_LOGI(TAG, "SPI pins - CS: GPIO%d, MISO: GPIO%d, MOSI: GPIO%d, CLK: GPIO%d", 
             ENCODER_SPI_CS_PIN, ENCODER_SPI_MISO_PIN, ENCODER_SPI_MOSI_PIN, ENCODER_SPI_CLK_PIN);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "SPI magnetic encoder system initialized successfully");
        // Wait a moment for encoder to stabilize and complete initial reading
        vTaskDelay(pdMS_TO_TICKS(200));
        DEBUG_ENC("SPI Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO",
                 encoder_get_angle_deg());
        
        // Print additional SPI encoder diagnostics
        if (encoder_is_valid()) {
            ESP_LOGI(TAG, "SPI magnetic encoder ready - %s", encoder_get_type_name());
        } else {
            ESP_LOGW(TAG, "SPI magnetic encoder initialized but data not yet valid");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize SPI magnetic encoder system");
        DEBUG_ENC("SPI magnetic encoder initialization FAILED - check hardware connections");
        ESP_LOGE(TAG, "Check SPI pins: CS=GPIO%d, MISO=GPIO%d, MOSI=GPIO%d, CLK=GPIO%d", 
                 ENCODER_SPI_CS_PIN, ENCODER_SPI_MISO_PIN, ENCODER_SPI_MOSI_PIN, ENCODER_SPI_CLK_PIN);
        ESP_LOGE(TAG, "Verify AS504x encoder chip power supply and SPI connections");
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_PWM_MAGNETIC
    ESP_LOGI(TAG, "Encoder config - Type: PWM_MAGNETIC");
    ESP_LOGI(TAG, "PWM pin: GPIO%d, Range: %d-%d us", 
             ENCODER_PWM_PIN, ENCODER_PWM_MIN_US, ENCODER_PWM_MAX_US);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "PWM magnetic encoder system initialized successfully");
        vTaskDelay(pdMS_TO_TICKS(100));
        DEBUG_ENC("PWM Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO",
                 encoder_get_angle_deg());
    } else {
        ESP_LOGE(TAG, "Failed to initialize PWM magnetic encoder system");
        DEBUG_ENC("PWM magnetic encoder initialization FAILED - check hardware connections");
        ESP_LOGE(TAG, "Check PWM pin: GPIO%d", ENCODER_PWM_PIN);
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_QUADRATURE
    ESP_LOGI(TAG, "Encoder config - Type: QUADRATURE, PPR: %d", ENCODER_PPR);
    ESP_LOGI(TAG, "Quadrature pins - A: GPIO%d, B: GPIO%d", ENCODER_A_PIN, ENCODER_B_PIN);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "Quadrature encoder system initialized successfully");
        vTaskDelay(pdMS_TO_TICKS(100));
        DEBUG_ENC("Quadrature Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO",
                 encoder_get_angle_deg());
    } else {
        ESP_LOGE(TAG, "Failed to initialize quadrature encoder system");
        DEBUG_ENC("Quadrature encoder initialization FAILED - check hardware connections");
        ESP_LOGE(TAG, "Check quadrature pins: A=GPIO%d, B=GPIO%d", ENCODER_A_PIN, ENCODER_B_PIN);
        return;
    }
    
    #elif ENCODER_TYPE == ENCODER_TYPE_NONE
    ESP_LOGW(TAG, "No encoder configured - position feedback disabled");
    ESP_LOGW(TAG, "System will operate in current control mode only");
    
    #else
    ESP_LOGE(TAG, "Unknown encoder type: %d - check board_config.h", ENCODER_TYPE);
    ESP_LOGE(TAG, "System cannot continue without valid encoder configuration");
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
    xTaskCreate(espnow_init_task, "espnow_init", 8192, NULL, 3, NULL);  // 8KB stack, priority 3
    #else
    ESP_LOGI(TAG, "ESP-NOW telemetry DISABLED in board configuration");
    #endif
    
    ESP_LOGI(TAG, "Initialization complete - System ready");
    ESP_LOGI(TAG, "Angle Range: %.1f° to %.1f°", MIN_ANGLE, MAX_ANGLE);
}