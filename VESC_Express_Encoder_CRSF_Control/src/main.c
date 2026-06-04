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
#include "drivers/fram_i2c.h"
#include "datatypes.h"
#include "board_config.h"
#include "utils.h"
#include <string.h>

// Main task and debug configuration
static const char *TAG = "VESC_Express";

// ESP-NOW telemetry / shared Wi-Fi configuration
#if ESP_NOW_TELEMETRY_ENABLE || ESP_NOW_BIDIRECTIONAL_ENABLE
#include "comm/comm_espnow.h"
#endif

// ESP-NOW bidirectional communication includes
#if ESP_NOW_BIDIRECTIONAL_ENABLE
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
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
static float encoder_calibration_offset = 0.0f;  // Offset to make current encoder reading equal to REST_ANGLE
static bool encoder_calibrated = false;  // Flag to indicate if calibration has been performed

// FRAM storage variables
static fram_encoder_data_t fram_data = {0};
#if ESP_NOW_BIDIRECTIONAL_ENABLE
static fram_remote_data_t fram_remote_data = {0};
#endif
static uint32_t last_fram_save = 0;
#if ESP_NOW_BIDIRECTIONAL_ENABLE
static uint32_t last_fram_remote_save = 0;
#endif
static bool fram_initialized = false;

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

// Get calibrated encoder reading - applies calibration offset if encoder is calibrated
float get_calibrated_encoder_angle_deg(void) {
    if (!encoder_is_valid()) {
        return -999.0f;  // Invalid encoder reading
    }
    
    float raw_angle = encoder_get_angle_deg();
    
    if (encoder_calibrated) {
        return raw_angle + encoder_calibration_offset;
    } else {
        return raw_angle;  // Return raw angle if not calibrated yet
    }
}

// Initialize FRAM and load saved encoder data on startup
void fram_init_and_load_data(void) {
    // Initialize FRAM I2C interface
    esp_err_t ret = fram_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FRAM initialization failed: %s", esp_err_to_name(ret));
        fram_initialized = false;
        return;
    }

    fram_initialized = true;

    // Load previously saved encoder data
    ret = fram_load_encoder_data(&fram_data);
    if (ret == ESP_OK) {
        // Restore calibration data if valid
        if (fram_data.calibrated) {
            encoder_calibration_offset = fram_data.calibration_offset;
            encoder_calibrated = fram_data.calibrated;
            ESP_LOGI(TAG, "FRAM: Restored calibration - Offset:%.2f°, Last angle:%.2f°, Boot count:%lu", 
                    fram_data.calibration_offset, fram_data.current_angle, fram_data.boot_count);
        } else {
            ESP_LOGI(TAG, "FRAM: No previous calibration found");
        }

        // Increment boot count
        fram_data.boot_count++;
        ESP_LOGI(TAG, "FRAM: Boot count incremented to %lu", fram_data.boot_count);
    } else {
        ESP_LOGW(TAG, "FRAM: Failed to load data, starting with defaults: %s", esp_err_to_name(ret));
        // Initialize with default values
        fram_data.current_angle = 0.0f;
        fram_data.calibration_offset = 0.0f;
        fram_data.calibrated = false;
        fram_data.boot_count = 1;
        fram_data.last_save_time = 0;
    }

    // Load remote ESP-NOW data
    #if ESP_NOW_BIDIRECTIONAL_ENABLE
    ret = fram_load_remote_data(&fram_remote_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FRAM: Loaded remote data - CH2:%.2f°(%s), CH3:%.2f°(%s), RX:%lu, TX:%lu", 
                fram_remote_data.remote_angle_ch2, fram_remote_data.remote_ch2_valid ? "OK" : "BAD",
                fram_remote_data.remote_angle_ch3, fram_remote_data.remote_ch3_valid ? "OK" : "BAD",
                fram_remote_data.packets_received, fram_remote_data.packets_sent);
    } else {
        ESP_LOGI(TAG, "FRAM: No previous remote data, initializing defaults");
        fram_remote_data.remote_angle_ch2 = 0.0f;
        fram_remote_data.remote_angle_ch3 = 0.0f;
        fram_remote_data.remote_timestamp = 0;
        fram_remote_data.remote_ch2_valid = false;
        fram_remote_data.remote_ch3_valid = false;
        fram_remote_data.packets_received = 0;
        fram_remote_data.packets_sent = 0;
        fram_remote_data.last_communication_time = 0;
    }
    #endif
}

// Save current encoder state to FRAM (called periodically)
void fram_save_current_encoder_data(uint32_t current_time) {
    if (!fram_initialized) {
        return;
    }

    // Update current encoder data
    encoder_update();
    if (encoder_is_valid()) {
        fram_data.current_angle = get_calibrated_encoder_angle_deg();
    }
    fram_data.calibration_offset = encoder_calibration_offset;
    fram_data.calibrated = encoder_calibrated;
    fram_data.last_save_time = current_time;

    // Save to FRAM
    esp_err_t ret = fram_save_encoder_data(&fram_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "FRAM: Failed to save encoder data: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "FRAM: Saved encoder data - Angle:%.2f°, Offset:%.2f°, Calibrated:%s", 
                fram_data.current_angle, fram_data.calibration_offset, 
                fram_data.calibrated ? "YES" : "NO");
    }
}

#if ESP_NOW_BIDIRECTIONAL_ENABLE
// ESP-NOW bidirectional communication data structure
typedef struct __attribute__((packed)) {
    char device_role[16];        // Device identifier (e.g., "LEFT_SHOULDER", "LEFT_ARM")
    uint8_t channel_count;       // Number of channels in this packet (1 for shoulder, 2 for arm)
    float channel_2;             // CRSF channel 2 angle (shoulder or upper arm)
    float channel_3;             // CRSF channel 3 angle (elbow)
    uint8_t armed;               // Arm/disarm status (uint8_t instead of bool for C/C++ compatibility)
    uint8_t calibrate_command;   // Calibrate command (uint8_t instead of bool for C/C++ compatibility)
    uint32_t timestamp;          // Timestamp of the measurement
    uint32_t sequence_number;    // Packet sequence number
    uint8_t checksum;            // Simple checksum for data integrity
} espnow_angle_packet_t;

// ESP-NOW receive callback for bidirectional communication
void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(espnow_angle_packet_t)) {
        ESP_LOGW(TAG, "ESP-NOW: Received packet wrong size: %d bytes", len);
        return;
    }

    espnow_angle_packet_t received_packet;
    memcpy(&received_packet, data, sizeof(espnow_angle_packet_t));

    // Verify checksum (simple sum of bytes)
    uint8_t calc_checksum = 0;
    uint8_t *packet_bytes = (uint8_t*)&received_packet;
    for (int i = 0; i < sizeof(espnow_angle_packet_t) - 1; i++) {  // Exclude checksum byte
        calc_checksum += packet_bytes[i];
    }

    if (calc_checksum != received_packet.checksum) {
        ESP_LOGW(TAG, "ESP-NOW: Checksum mismatch - packet corrupted");
        return;
    }

    // Update remote data using configured channels
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    fram_remote_data.remote_timestamp = received_packet.timestamp;
    fram_remote_data.packets_received++;
    fram_remote_data.last_communication_time = current_time;

    if (received_packet.channel_count >= 1) {
        // Upper arm angle (CRSF channel 2)
        fram_remote_data.remote_angle_ch2 = received_packet.channel_2;
        fram_remote_data.remote_ch2_valid = true;
    }
    
    if (received_packet.channel_count >= 2) {
        // Elbow angle (CRSF channel 3)
        fram_remote_data.remote_angle_ch3 = received_packet.channel_3;
        fram_remote_data.remote_ch3_valid = true;
    }

    // Process armed/disarmed and calibrate command status from remote device
    static bool remote_armed = false;
    static bool remote_calibrate_command = false;
    remote_armed = received_packet.armed;
    remote_calibrate_command = received_packet.calibrate_command;

    ESP_LOGI(TAG, "ESP-NOW RX: %s CH2:%.1f° CH3:%.1f° Armed:%s Cal:%s (seq:%lu, age:%lums)", 
             received_packet.device_role, 
             received_packet.channel_count >= 1 ? received_packet.channel_2 : 0.0f,
             received_packet.channel_count >= 2 ? received_packet.channel_3 : 0.0f,
             received_packet.armed ? "YES" : "NO",
             received_packet.calibrate_command ? "YES" : "NO",
             received_packet.sequence_number, current_time - received_packet.timestamp);

    // Save to FRAM (rate limited)
    #ifdef FRAM_I2C_SDA_PIN
    if (fram_initialized && (current_time - last_fram_remote_save > 1000)) {  // Save remote data every 1 second
        if (received_packet.channel_count >= 1) {
            // Save upper arm angle (RX channel 0)
            fram_update_remote_angle(2, fram_remote_data.remote_angle_ch2, fram_remote_data.remote_timestamp);
        }
        if (received_packet.channel_count >= 2) {
            // Save elbow angle (RX channel 1)
            fram_update_remote_angle(3, fram_remote_data.remote_angle_ch3, fram_remote_data.remote_timestamp);
        }
        last_fram_remote_save = current_time;
    }
    #endif
}

// Send CRSF channel data via ESP-NOW to remote arm
void espnow_send_crsf_channels(uint32_t timestamp) {
    static uint32_t sequence_number = 0;
    static bool crsf_initialized = false;
    
    // Wait for CRSF to be initialized before sending
    if (!crsf_initialized) {
        crsf_initialized = true;  // Set flag after first call
        return;  // Skip first send to avoid accessing uninitialized CRSF
    }
    
    espnow_angle_packet_t packet;
    memset(&packet, 0, sizeof(packet));  // Clear packet
    strncpy(packet.device_role, LOCAL_ESP32_ROLE, sizeof(packet.device_role) - 1);
    packet.device_role[sizeof(packet.device_role) - 1] = '\0';  // Ensure null termination
    packet.channel_count = 2;            // Sending 2 channels (upper arm + elbow control)
    packet.channel_2 = (float)channels[CRSF_CHANNEL_PITCH - 1];    // CRSF channel 2 (Pitch, 1000-2000) -> upper arm
    packet.channel_3 = (float)channels[CRSF_CHANNEL_THROTTLE - 1]; // CRSF channel 3 (Throttle, 1000-2000) -> elbow
    packet.armed = crsf_is_armed();      // Channel 5 arm/disarm status
    packet.calibrate_command = (channels[CRSF_CHANNEL_AUX2 - 1] > 1700);  // Channel 6 calibrate command (high position)
    packet.timestamp = timestamp;
    packet.sequence_number = ++sequence_number;
    
    // Calculate checksum
    packet.checksum = 0;
    uint8_t *packet_bytes = (uint8_t*)&packet;
    for (int i = 0; i < sizeof(espnow_angle_packet_t) - 1; i++) {  // Exclude checksum byte
        packet.checksum += packet_bytes[i];
    }

    // Send via ESP-NOW (using existing telemetry infrastructure)
    uint8_t remote_mac[] = REMOTE_ESP32_MAC_ADDR;
    esp_err_t ret = esp_now_send(remote_mac, (uint8_t*)&packet, sizeof(packet));
    
    if (ret == ESP_OK) {
        fram_remote_data.packets_sent++;
        // Rate limit logging to once per second
        static uint32_t last_log_time = 0;
        if ((timestamp - last_log_time) >= 1000) {
            ESP_LOGI(TAG, "ESP-NOW TX: %s CH2:%.0f CH3:%.0f Armed:%s Cal:%s (seq:%lu)", 
                     LOCAL_ESP32_ROLE, packet.channel_2, packet.channel_3,
                     packet.armed ? "YES" : "NO", 
                     packet.calibrate_command ? "YES" : "NO", sequence_number);
            last_log_time = timestamp;
        }
    } else {
        ESP_LOGW(TAG, "ESP-NOW TX failed: %s", esp_err_to_name(ret));
    }
}

// Initialize bidirectional ESP-NOW communication (independent of telemetry)
esp_err_t espnow_bidirectional_init(void) {
    esp_err_t ret;
    
    ESP_ERROR_CHECK(espnow_wifi_init_station(1));
    
    // Initialize ESP-NOW
    ret = espnow_init_core();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register receive callback
    ret = esp_now_register_recv_cb(espnow_receive_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW register receive callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add remote peer
    uint8_t remote_mac[] = REMOTE_ESP32_MAC_ADDR;
    ret = espnow_add_peer_open(remote_mac, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW add peer failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW bidirectional communication initialized");
    ESP_LOGI(TAG, "Local role: %s, Remote MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             LOCAL_ESP32_ROLE, remote_mac[0], remote_mac[1], remote_mac[2], 
             remote_mac[3], remote_mac[4], remote_mac[5]);

    return ESP_OK;
}
#endif

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
            // Reset failsafe state when connection is restored
            if (!was_connected) {
                // Connection restored - reset state for next time
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
            // SAFETY: Only allow calibration when CRSF connected, VESC connected, but system DISARMED
            bool channel6_high = (channels[CRSF_CHANNEL_AUX2 - 1] > 1700);  // Channel 6 = AUX2 (high position)
            if (crsf_is_connected() && !crsf_is_armed() && vesc_position_valid && channel6_high && !last_channel6_state) {
                // Channel 6 transitioned from low to high - trigger calibration
                // Rate limit calibration to prevent accidental repeated triggers
                if (current_time - last_calibration_time > 2000) {  // Rate limit to once per 2 seconds
                    ESP_LOGI(TAG, "[CALIBRATION] Channel 6 triggered - calibrating current position to %.1f degrees", REST_ANGLE);
                    
                    // Update encoder data to get current reading
                    encoder_update();
                    
                    if (encoder_is_valid()) {
                        // Calculate offset so current encoder reading equals REST_ANGLE
                        float current_raw_angle = encoder_get_angle_deg();
                        encoder_calibration_offset = REST_ANGLE - current_raw_angle;
                        encoder_calibrated = true;
                        
                        ESP_LOGI(TAG, "[CALIBRATION] Encoder calibrated: Raw=%.1f°, Offset=%.1f°, Calibrated=%.1f°", 
                                current_raw_angle, encoder_calibration_offset, REST_ANGLE);
                        
                        // Save calibration data to FRAM immediately
                        #ifdef FRAM_I2C_SDA_PIN
                        if (fram_initialized) {
                            fram_save_current_encoder_data(current_time);
                            ESP_LOGI(TAG, "[CALIBRATION] Calibration data saved to FRAM");
                        }
                        #endif
                    } else {
                        ESP_LOGW(TAG, "[CALIBRATION] Encoder calibration failed - encoder not valid");
                    }
                    
                    last_calibration_time = current_time;
                } else {
                    ESP_LOGW(TAG, "[CALIBRATION] Ignoring calibration request (rate limited)");
                }
            }
            last_channel6_state = channel6_high;
            
        } else {
            // No connection - apply safety behavior
            comm_can_set_current(CAN_VESC_ID, 0.0f);  // Send zero current command to ensure motor is not driven
            if (was_connected) {
                // Just lost connection - reset state to prepare for next time
                was_connected = false;
            }
                
            // Update ESP-NOW telemetry even during failsafe (with default CRSF target)
            #if ESP_NOW_TELEMETRY_ENABLE && !DEBUG_ESPNOW_TEST
            encoder_update();  // Update encoder data before reading
            float encoder_degrees = get_calibrated_encoder_angle_deg();
            float crsf_target_degrees = 0.0f; // Default  value
            float vesc_target_revolutions = 0.0f; // Default value
            telemetry_espnow_set_payload_data(BOARD_NAME, encoder_degrees, crsf_target_degrees, vesc_target_revolutions);
            #endif
        }
        
        // ESP-NOW test function - sends random data when DEBUG_ESPNOW_TEST is enabled
        #if DEBUG_ESPNOW_TEST
        #if ESP_NOW_TELEMETRY_ENABLE
        espnow_test_send_random_data();
        #endif
        #endif
        
        // ESP-NOW bidirectional communication - send CRSF channel data to remote arm
        #if ESP_NOW_BIDIRECTIONAL_ENABLE
        static uint32_t last_espnow_send = 0;
        if (current_time - last_espnow_send > 100) {  // Send every 100ms (10Hz)
            espnow_send_crsf_channels(current_time);
            last_espnow_send = current_time;
        }
        #endif
        
        // Periodic FRAM saving - save complete encoder data every 60 seconds (angle saved after each CAN frame)
        #ifdef FRAM_I2C_SDA_PIN
        if (fram_initialized && (current_time - last_fram_save > 60000)) {  // Save complete data every 60 seconds
            fram_save_current_encoder_data(current_time);
            
            // Also save remote ESP-NOW data periodically
            #if ESP_NOW_BIDIRECTIONAL_ENABLE
            fram_save_remote_data(&fram_remote_data);
            #endif
            
            last_fram_save = current_time;
        }
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
    
    // Check if armed using utility function (requires CRSF connection)
    if (crsf_is_connected() && crsf_is_armed()) {
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
                current_position_degrees = get_calibrated_encoder_angle_deg();
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
            
            // Save current encoder angle to FRAM after each CAN position command
            #ifdef FRAM_I2C_SDA_PIN
            if (fram_initialized) {
                fram_write_float(FRAM_ADDR_ENCODER_ANGLE, current_position_degrees);
                fram_write_uint32(FRAM_ADDR_TIMESTAMP, current_time);
            }
            #endif
            
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
                current_position_degrees = get_calibrated_encoder_angle_deg();
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
            #else
            (void)vesc_target_position_revolutions;  // Prevent unused variable warning
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
        
        // Disarmed or no connection - send zero current command IMMEDIATELY for safety
        // Send every time this function is called to ensure it overrides any position commands
        wait_for_safe_can_slot();
        static uint32_t last_disarmed_debug = 0;
        if (current_time - last_disarmed_debug > 2000) { // Debug every 2 seconds, but command every time
            DEBUG_CAN_CMD("CMD_ID=%d (SET_CURRENT) to VESC_ID=%d, value=%.3f [DISARMED/NO_CONNECTION SAFETY]", CAN_PACKET_SET_CURRENT, CAN_VESC_ID, 0.0f);
            last_disarmed_debug = current_time;
        }
        comm_can_set_current(CAN_VESC_ID, 0.0f);  // Send 0A current EVERY time for maximum safety
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

// ESP-NOW bidirectional initialization task (runs once then deletes itself)
#if ESP_NOW_BIDIRECTIONAL_ENABLE
void espnow_bidirectional_init_task(void *pvParameters) {
    ESP_LOGI(TAG, "ESP-NOW bidirectional initialization task starting...");
    
    // Initialize ESP-NOW bidirectional with retry mechanism
    int retry_count = 0;
    const int max_retries = 3;
    
    while (retry_count < max_retries) {
        esp_err_t ret = espnow_bidirectional_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ESP-NOW bidirectional initialized successfully on attempt %d", retry_count + 1);
            break;
        } else {
            retry_count++;
            ESP_LOGW(TAG, "ESP-NOW bidirectional init failed (attempt %d/%d): %s", retry_count, max_retries, esp_err_to_name(ret));
            if (retry_count < max_retries) {
                vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second before retry
            }
        }
    }
    
    if (retry_count >= max_retries) {
        ESP_LOGE(TAG, "ESP-NOW bidirectional initialization failed after %d attempts - bidirectional communication disabled", max_retries);
    }
    
    ESP_LOGI(TAG, "ESP-NOW bidirectional initialization task completed - deleting self");
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

    // Initialize FRAM for persistent data storage
    #ifdef FRAM_I2C_SDA_PIN
    ESP_LOGI(TAG, "Initializing FRAM for encoder data storage...");
    fram_init_and_load_data();
    #else
    ESP_LOGI(TAG, "FRAM not configured for this board");
    #endif

    // Initialize WiFi/networking if needed by ESP-NOW
    #if ESP_NOW_TELEMETRY_ENABLE
    ESP_ERROR_CHECK(espnow_wifi_init_station(1));
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
    
    // Create bidirectional ESP-NOW initialization task (independent of telemetry)
    #if ESP_NOW_BIDIRECTIONAL_ENABLE
    ESP_LOGI(TAG, "ESP-NOW bidirectional ENABLED - Creating initialization task...");
    xTaskCreate(espnow_bidirectional_init_task, "espnow_bidir_init", 8192, NULL, 3, NULL);
    #else
    ESP_LOGI(TAG, "ESP-NOW bidirectional DISABLED in board configuration");
    #endif
    
    ESP_LOGI(TAG, "Initialization complete - System ready");
    ESP_LOGI(TAG, "Hybrid control: Periodic ping + Event-driven VESC Status 4 position control");
    ESP_LOGI(TAG, "Angle Range: %.1f° to %.1f°", MIN_ANGLE, MAX_ANGLE);
}
