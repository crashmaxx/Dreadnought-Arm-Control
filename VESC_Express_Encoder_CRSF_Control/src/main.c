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
#include "conf_general.h"
#include "hw_devkit_c3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "main.h"
#include <math.h>

// CRSF Configuration from config header
#define CONTROL_TASK_DELAY_MS (1000 / CRSF_CONTROL_UPDATE_RATE_HZ)

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

// Stub function for backup data storage (required by comm_can.c)
void main_store_backup_data(void) {
    // For ESP32-C3 minimal implementation, we don't need persistent storage
    // This is just a stub to satisfy the linker
    ESP_LOGI("BACKUP", "Backup data store requested (stub implementation)");
}

// Debug configuration - set to 1 to enable, 0 to disable
#define DEBUG_POSITION_CONTROL      0
#define DEBUG_CRSF_CHANNELS         0
#define DEBUG_ENCODER_DATA          1
#define DEBUG_VESC_STATUS           1
#define DEBUG_CONTROL_FLOW          0

// CAN Test Mode - set to 1 to enable CAN testing without motor commands
#define CAN_TEST_MODE               0

// Debug macros
#if DEBUG_POSITION_CONTROL
#define DEBUG_POS(fmt, ...) ESP_LOGI(TAG, "[POS] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_POS(fmt, ...)
#endif

#if DEBUG_CRSF_CHANNELS
#define DEBUG_CRSF(fmt, ...) ESP_LOGI(TAG, "[CRSF] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_CRSF(fmt, ...)
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

#if DEBUG_CONTROL_FLOW
#define DEBUG_FLOW(fmt, ...) ESP_LOGI(TAG, "[FLOW] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_FLOW(fmt, ...)
#endif

#if CAN_TEST_MODE
#define DEBUG_CAN_TEST(fmt, ...) ESP_LOGI(TAG, "[CAN_TEST] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_CAN_TEST(fmt, ...)
#endif

static const char *TAG = "MAIN";

#if CAN_TEST_MODE
// CAN Test Mode function - reads VESC status and sends harmless packets
static void can_test_mode(void) {
    static uint32_t last_status_request = 0;
    static uint32_t last_ping = 0;
    static uint32_t last_detailed_print = 0;
    static bool first_run = true;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // First run initialization message
    if (first_run) {
        ESP_LOGI(TAG, "[CAN_TEST] Starting CAN test mode - looking for VESC ID %d", CAN_VESC_ID);
        first_run = false;
    }
    
    // Request VESC status less frequently - every 5 seconds to avoid flooding
    if (current_time - last_status_request > 5000) {
        // Check if we have recent data before requesting more
        can_status_msg *status1 = comm_can_get_status_msg_id(CAN_VESC_ID);
        can_status_msg_4 *status4 = comm_can_get_status_msg_4_id(CAN_VESC_ID);
        
        uint32_t status1_age = status1 ? (current_time - (status1->rx_time * portTICK_PERIOD_MS)) : 999999;
        uint32_t status4_age = status4 ? (current_time - (status4->rx_time * portTICK_PERIOD_MS)) : 999999;
        
        // VESC Tool shows status messages every 50ms are configured, but we're not receiving them
        // This suggests either:
        // 1. CAN ID mismatch (VESC sending on different ID than 74)
        // 2. CAN message filtering issue
        // 3. VESC not actually sending despite configuration
        
        ESP_LOGI(TAG, "[CAN_TEST] VESC configured for 50ms status but not receiving fresh data");
        ESP_LOGI(TAG, "[CAN_TEST] Possible issues: CAN ID mismatch, filtering, or VESC not sending");
        ESP_LOGI(TAG, "[CAN_TEST] Current status ages: Status1=%lums, Status4=%lums", status1_age, status4_age);
        
        // Let's check all possible VESC status message storage slots to see if messages
        // are coming from a different CAN ID
        ESP_LOGI(TAG, "[CAN_TEST] Checking all status message slots for any recent data...");
        bool found_any_recent = false;
        for (int i = 0; i < 10; i++) {  // Check first 10 slots
            can_status_msg *check_status = comm_can_get_status_msg_index(i);
            if (check_status) {
                // Convert rx_time from ticks to milliseconds for proper comparison
                uint32_t rx_time_ms = check_status->rx_time * portTICK_PERIOD_MS;
                uint32_t age = current_time - rx_time_ms;
                ESP_LOGI(TAG, "[CAN_TEST] Slot %d: ID=%d, Age=%lums, RPM=%.1f, rx_time=%lu ticks (%lu ms), current_time=%lu ms", 
                         i, check_status->id, age, check_status->rpm, check_status->rx_time, rx_time_ms, current_time);
                if (age < 5000) {  // Less than 5 seconds old
                    found_any_recent = true;
                    ESP_LOGI(TAG, "[CAN_TEST] ^^^ This message is relatively recent!");
                }
            } else {
                ESP_LOGI(TAG, "[CAN_TEST] Slot %d: Empty", i);
            }
        }
        
        if (!found_any_recent) {
            ESP_LOGW(TAG, "[CAN_TEST] No recent status messages found in any slot!");
            ESP_LOGW(TAG, "[CAN_TEST] VESC may have stopped sending status messages");
        }
        
        // Try multiple approaches to wake up VESC status transmission
        ESP_LOGI(TAG, "[CAN_TEST] Attempting to trigger VESC status transmission...");
        
        // Method 1: Send zero current command (might trigger status response)
        ESP_LOGI(TAG, "[CAN_TEST] Sending zero current command to ID %d", CAN_VESC_ID);
        comm_can_set_current(CAN_VESC_ID, 0.0f);
        
        // Method 2: Try a different command - set RPM to 0
        ESP_LOGI(TAG, "[CAN_TEST] Sending zero RPM command to ID %d", CAN_VESC_ID);
        comm_can_set_rpm(CAN_VESC_ID, 0.0f);
        
        // Method 3: Try a small position adjustment (might wake up status)
        ESP_LOGI(TAG, "[CAN_TEST] Sending position hold command to ID %d", CAN_VESC_ID);
        if (status4 && status4->rx_time > 0) {
            // Send current position to hold it
            comm_can_set_pos(CAN_VESC_ID, status4->pid_pos_now);
        } else {
            // Send position 0 as fallback
            comm_can_set_pos(CAN_VESC_ID, 0.0f);
        }
        
        last_status_request = current_time;
    }
    
    // Send ping every 10 seconds (harmless)
    if (current_time - last_ping > 10000) {
        // Send a ping command - completely harmless
        ESP_LOGI(TAG, "[CAN_TEST] Sending ping to VESC ID %d...", CAN_VESC_ID);
        HW_TYPE hw_type;
        bool ping_result = comm_can_ping(CAN_VESC_ID, &hw_type);
        ESP_LOGI(TAG, "[CAN_TEST] Ping result: %s", ping_result ? "SUCCESS" : "FAILED");
        if (ping_result) {
            ESP_LOGI(TAG, "[CAN_TEST] Hardware type: %d", hw_type);
        }
        last_ping = current_time;
    }
    
    // Print detailed VESC status every 5 seconds
    if (current_time - last_detailed_print > 5000) {
        can_status_msg *status1 = comm_can_get_status_msg_id(CAN_VESC_ID);
        can_status_msg_4 *status4 = comm_can_get_status_msg_4_id(CAN_VESC_ID);
        
        ESP_LOGI(TAG, "[CAN_TEST] === Checking for VESC status messages ===");
        ESP_LOGI(TAG, "[CAN_TEST] ESP32 ID: %d, Looking for VESC ID: %d", CAN_ESP32_ID, CAN_VESC_ID);
        
        if (status1) {
            uint32_t age_ms = current_time - (status1->rx_time * portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "[CAN_TEST] Status1 - RPM: %.1f, Current: %.2fA, Duty: %.2f%%, Age: %lums", 
                          status1->rpm, status1->current, status1->duty * 100.0f, age_ms);
        } else {
            ESP_LOGW(TAG, "[CAN_TEST] Status1 - No data received from VESC ID %d", CAN_VESC_ID);
        }
        
        if (status4) {
            uint32_t age_ms = current_time - (status4->rx_time * portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "[CAN_TEST] Status4 - Position: %.3f rev, Current In: %.2fA, Temp FET: %.1f°C, Age: %lums", 
                          status4->pid_pos_now, status4->current_in, status4->temp_fet, age_ms);
        } else {
            ESP_LOGW(TAG, "[CAN_TEST] Status4 - No position data from VESC ID %d", CAN_VESC_ID);
        }
        
        last_detailed_print = current_time;
    }
}
#endif

// Task to handle CRSF data and control
static void crsf_control_task(void *pvParameters) {
    uint16_t channels[16];
    uint32_t last_print = 0;
    uint32_t last_encoder_print = 0;
    uint32_t last_stack_check = 0;
    uint32_t last_vesc_debug = 0;  // Rate limit VESC debug messages
    
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
        
        #if CAN_TEST_MODE
        // Run CAN test mode - reads VESC status and sends harmless packets
        can_test_mode();
        #endif
        
        // Check stack usage every 5 seconds
        if (current_time - last_stack_check > 5000) {
            UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL);
            if (stack_free < 500) {
                ESP_LOGW(TAG, "Low stack space: %u bytes free", stack_free);
            } else {
                ESP_LOGI(TAG, "Stack health: %u bytes free", stack_free);
            }
            last_stack_check = current_time;
        }
        
        // Request VESC position every 200ms for responsive control
        // VESC sends status messages automatically - we just check if we have recent data
        can_status_msg_4 *vesc_status = comm_can_get_status_msg_4_id(CAN_VESC_ID);
        if (vesc_status) {
            uint32_t age_ms = current_time - (vesc_status->rx_time * portTICK_PERIOD_MS);
            if (age_ms < 300) {
                // We have recent VESC position data (within 300ms)
                vesc_current_position = vesc_status->pid_pos_now;
                vesc_position_valid = true;
                // Rate limit VESC debug messages to every 2 seconds
                if (current_time - last_vesc_debug > 2000) {
                    DEBUG_VESC("VESC pos: %.3f rev, age: %lums", 
                              vesc_current_position, age_ms);
                    last_vesc_debug = current_time;
                }
            } else {
                // VESC position data is stale
                vesc_position_valid = false;
                // Rate limit VESC debug messages to every 2 seconds
                if (current_time - last_vesc_debug > 2000) {
                    DEBUG_VESC("VESC data STALE: %lums", age_ms);
                    last_vesc_debug = current_time;
                }
            }
        } else {
            // No VESC status message received at all
            vesc_position_valid = false;
            // Rate limit VESC debug messages to every 2 seconds
            if (current_time - last_vesc_debug > 2000) {
                DEBUG_VESC("VESC missing from CAN ID %d", CAN_VESC_ID);
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
                DEBUG_CRSF("Channels: CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d Connected=%s Failsafe=%s",
                         channels[0], channels[1], channels[2], channels[3], channels[4],
                         crsf_is_connected() ? "YES" : "NO",
                         crsf_is_failsafe() ? "YES" : "NO");
                last_print = current_time;
            }
            
            // Here you can add your control logic using the channel data and encoder feedback
            // For example:
            // - Use channels[0] (CH1) for position control setpoint
            // - Use channels[1] (CH2) for velocity control  
            // - Use channels[2] (CH3) for throttle/current control
            // - Use channels[3] (CH4) for yaw control
            // - Use channels[4] (CH5) for arm/disarm switch
            // - Use encoder position/velocity for closed-loop control
            // - Send commands via CAN based on channel values and encoder feedback
            
            // Send motor commands based on CRSF input and encoder feedback
            #if CAN_TEST_MODE
            DEBUG_CAN_TEST("CAN TEST MODE: Skipping motor commands - would send based on CRSF/encoder data");
            DEBUG_CAN_TEST("CRSF Connected: %s, Failsafe: %s, Armed: %s", 
                          crsf_is_connected() ? "YES" : "NO",
                          crsf_is_failsafe() ? "YES" : "NO",
                          crsf_is_armed() ? "YES" : "NO");
            if (crsf_is_connected() && !crsf_is_failsafe() && crsf_is_armed()) {
                float crsf_target_degrees = (crsf_channel_to_normalized(1) - 0.5f) * 360.0f;
                float encoder_degrees = encoder_get_angle_deg();
                DEBUG_CAN_TEST("Would command - CRSF target: %.1f°, Encoder: %.1f°, VESC valid: %s", 
                              crsf_target_degrees, encoder_degrees, vesc_position_valid ? "YES" : "NO");
            }
            #else
            // Normal operation - send actual motor commands
            if (crsf_is_connected() && !crsf_is_failsafe()) {
                DEBUG_FLOW("CRSF connected, no failsafe");
                // Check if armed using utility function
                if (crsf_is_armed()) {
                    DEBUG_FLOW("System ARMED - proceeding with control");
                    // Position control logic
                    // Only proceed with position control if we have valid VESC position
                    if (vesc_position_valid) {
                        DEBUG_FLOW("Using position control mode");
                        // Convert CRSF channel to target angle (using channel 1 for position control)
                        // CRSF channels are normalized 0.0 to 1.0, convert to -180 to +180 degrees
                        float crsf_target_degrees = (crsf_channel_to_normalized(1) - 0.5f) * 360.0f;
                        
                        // Get current encoder position in degrees
                        float encoder_degrees = encoder_get_angle_deg();
                        
                        // Calculate position error (CRSF target - encoder position)
                        float position_error = crsf_target_degrees - encoder_degrees;
                        
                        // Apply gear ratio compensation
                        float gear_compensated_error = position_error * GEAR_RATIO;
                        
                        // Calculate new target position for VESC (convert degrees to revolutions)
                        float vesc_target_position_degrees = vesc_current_position * 360.0f + gear_compensated_error;
                        float vesc_target_position_revolutions = vesc_target_position_degrees / 360.0f;
                        
                        // Send position command to VESC (in revolutions)
                        comm_can_set_pos_floatingpoint(CAN_VESC_ID, vesc_target_position_revolutions);
                        
                        DEBUG_POS("CRSF: %.1f°, Encoder: %.1f°, Error: %.1f°, Gear Error: %.1f°, VESC Target: %.3f rev (%.1f°)", 
                                crsf_target_degrees, encoder_degrees, position_error, gear_compensated_error,
                                vesc_target_position_revolutions, vesc_target_position_degrees);
                    } else {
                        DEBUG_FLOW("Using current control mode (no VESC position)");
                        // No valid VESC position - fallback to current control
                        float throttle_normalized = crsf_channel_to_normalized(CRSF_THROTTLE_CHANNEL);
                        float current_command = throttle_normalized * CRSF_MAX_CURRENT_AMPS;
                        comm_can_set_current(CAN_VESC_ID, current_command);
                        DEBUG_POS("Current control: %.2f A (throttle: %.2f)", current_command, throttle_normalized);
                    }
                } else {
                    DEBUG_FLOW("System DISARMED - sending zero current");
                    // Disarmed - send zero current
                    comm_can_set_current(CAN_VESC_ID, 0.0f);
                }
            } else {
                DEBUG_FLOW("CRSF disconnected or failsafe - applying failsafe behavior");
                // No connection or failsafe - apply failsafe behavior
                if (CRSF_FAILSAFE_ENABLE_BRAKE) {
                    comm_can_set_current_brake(CAN_VESC_ID, CRSF_FAILSAFE_BRAKE_CURRENT);
                    DEBUG_FLOW("Failsafe: brake current %.2f A", CRSF_FAILSAFE_BRAKE_CURRENT);
                } else {
                    comm_can_set_current(CAN_VESC_ID, CRSF_FAILSAFE_CURRENT);
                    DEBUG_FLOW("Failsafe: current %.2f A", CRSF_FAILSAFE_CURRENT);
                }
            }
            #endif // CAN_TEST_MODE
        }
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== VESC Express Encoder CRSF Control Starting ===");
    ESP_LOGI(TAG, "Firmware Version: %d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_TEST_VERSION_NUMBER);
    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "ESP32 CAN ID: %d", CAN_ESP32_ID);
    ESP_LOGI(TAG, "Target VESC CAN ID: %d", CAN_VESC_ID);
    
#if CAN_TEST_MODE
    ESP_LOGW(TAG, "=== CAN TEST MODE ENABLED ===");
    ESP_LOGW(TAG, "Motor commands are DISABLED for safety");
    ESP_LOGW(TAG, "Only reading VESC status and sending harmless packets");
    ESP_LOGW(TAG, "Set CAN_TEST_MODE to 0 to enable motor control");
#endif
    
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

    // Initialize encoder system
    ESP_LOGI(TAG, "Encoder config - Type: DUAL_HYBRID, PPR: %d", ENCODER_PPR);
    ESP_LOGI(TAG, "Encoder pins - PWM: GPIO%d, A: GPIO%d, B: GPIO%d", 
             ENCODER_PWM_PIN, ENCODER_A_PIN, ENCODER_B_PIN);
    ESP_LOGI(TAG, "PWM range: %d-%d us", ENCODER_PWM_MIN_US, ENCODER_PWM_MAX_US);
    
    if (encoder_init()) {
        ESP_LOGI(TAG, "Encoder system initialized successfully");
        // Wait a moment for encoder to stabilize
        vTaskDelay(pdMS_TO_TICKS(100));
        DEBUG_ENC("Encoder init: Valid=%s, Initial angle=%.2f°", 
                 encoder_is_valid() ? "YES" : "NO",
                 encoder_get_angle_deg());
    } else {
        ESP_LOGE(TAG, "Failed to initialize encoder system");
        DEBUG_ENC("Encoder initialization FAILED - check hardware connections");
        ESP_LOGE(TAG, "Check pins: PWM=GPIO%d, A=GPIO%d, B=GPIO%d", 
                 ENCODER_PWM_PIN, ENCODER_A_PIN, ENCODER_B_PIN);
        return;
    }

    // Initialize ESP-NOW telemetry
    #if ESP_NOW_TELEMETRY_ENABLE
    esp_now_telemetry_config_t telemetry_config = {
        .peer_mac = PEER_MAC_ADDR,
        .wifi_channel = ESP_NOW_WIFI_CHANNEL,
        .encrypt = ESP_NOW_ENCRYPT,
        .recv_callback = telemetry_recv_callback
    };
    
    if (esp_now_telemetry_init(&telemetry_config)) {
        ESP_LOGI(TAG, "ESP-NOW telemetry initialized successfully");
        ESP_LOGI(TAG, "Telemetry peer: %02X:%02X:%02X:%02X:%02X:%02X", 
                 telemetry_config.peer_mac[0], telemetry_config.peer_mac[1], 
                 telemetry_config.peer_mac[2], telemetry_config.peer_mac[3], 
                 telemetry_config.peer_mac[4], telemetry_config.peer_mac[5]);
    } else {
        ESP_LOGW(TAG, "Failed to initialize ESP-NOW telemetry");
    }
    #endif

    // Initialize CRSF receiver
    crsf_init(CRSF_UART_NUM, CRSF_TX_PIN, CRSF_RX_PIN, CRSF_BAUDRATE);
    ESP_LOGI(TAG, "CRSF receiver initialized");
    
    // Create CRSF control task
    xTaskCreate(crsf_control_task, "crsf_control", CRSF_TASK_STACK_SIZE, NULL, CRSF_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "CRSF control task created");
    
    ESP_LOGI(TAG, "Initialization complete - System ready");
    ESP_LOGI(TAG, "Angle Range: %.1f° to %.1f°", MIN_ANGLE, MAX_ANGLE);
}