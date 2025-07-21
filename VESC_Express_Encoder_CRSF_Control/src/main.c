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
   
#include "comm_can.h"
#include "crsf_receiver.h"
#include "crsf_utils.h"
#include "crsf_config.h"
#include "encoder_interface.h"
#include "board_config.h"
#include "esp_now_telemetry.h"
#include "esp_now_telemetry_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

// CRSF Configuration from config header
#define CONTROL_TASK_DELAY_MS (1000 / CRSF_CONTROL_UPDATE_RATE_HZ)

static const char *TAG = "MAIN";

// ESP-NOW telemetry callback (for receiving telemetry from other devices)
static void telemetry_recv_callback(const esp_now_telemetry_packet_t* packet, const uint8_t* mac, int len) {
    ESP_LOGI(TAG, "Received telemetry from %02X:%02X:%02X:%02X:%02X:%02X: %s = %.2f, %.2f, %.2f", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             packet->name, packet->value1, packet->value2, packet->value3);
}

// Task to handle CRSF data and control
static void crsf_control_task(void *pvParameters) {
    uint16_t channels[16];
    uint32_t last_print = 0;
    uint32_t last_encoder_print = 0;
    uint32_t last_telemetry_send = 0;
    
    while (1) {
        // Update CRSF receiver
        crsf_update();
        
        // Update encoder
        encoder_update();
        
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Send telemetry data
        if (esp_now_telemetry_is_ready() && (current_time - last_telemetry_send > ESP_NOW_TELEMETRY_RATE_MS)) {
            if (encoder_is_valid()) {
                // Send encoder telemetry using helper macro
                TELEMETRY_SEND_ENCODER(encoder_get_angle_deg(), 
                                     encoder_get_velocity_deg_s(), 
                                     encoder_is_valid());
            }
            // Send system status
            TELEMETRY_SEND_STATUS(crsf_is_armed(), crsf_is_connected(), encoder_get_error_count());
            
            last_telemetry_send = current_time;
        }
        
        // Check for new CRSF data
        if (crsf_has_new_data()) {
            // Get all channel values (scaled to 1000-2000)
            crsf_get_all_channels_scaled(channels);
            
            // Print channel data for debugging
            if (current_time - last_print > CRSF_DEBUG_PRINT_RATE_MS) {
                #if CRSF_ENABLE_LOGGING
                ESP_LOGI(TAG, "CRSF Channels: CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d Connected=%s Failsafe=%s",
                         channels[0], channels[1], channels[2], channels[3], channels[4],
                         crsf_is_connected() ? "YES" : "NO",
                         crsf_is_failsafe() ? "YES" : "NO");
                #endif
                last_print = current_time;
            }
            
            // Print encoder data for debugging
            if (current_time - last_encoder_print > 500) { // Every 500ms
                if (encoder_is_valid()) {
                    ESP_LOGI(TAG, "Encoder: %.2f° (%.3f rad), Vel: %.1f°/s, Type: %s", 
                             encoder_get_angle_deg(), encoder_get_angle_rad(),
                             encoder_get_velocity_deg_s(), encoder_get_type_name());
                } else {
                    ESP_LOGW(TAG, "Encoder: INVALID (errors: %d)", encoder_get_error_count());
                }
                last_encoder_print = current_time;
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
            if (crsf_is_connected() && !crsf_is_failsafe()) {
                // Check if armed using utility function
                if (crsf_is_armed()) {
                    // Example: Use throttle channel for direct current control
                    float throttle_normalized = crsf_channel_to_normalized(CRSF_THROTTLE_CHANNEL);
                    float current_command = throttle_normalized * CRSF_MAX_CURRENT_AMPS;
                    
                    // Optional: Add encoder-based position or velocity control here
                    // For position control:
                    // float target_angle = crsf_channel_to_normalized(CRSF_POSITION_CHANNEL) * 360.0f;
                    // float position_error = target_angle - encoder_get_angle_deg();
                    // float pid_output = pid_controller(position_error);
                    // current_command = pid_output;
                    
                    // Send current command to VESC via CAN
                    comm_can_set_current(CRSF_VESC_CONTROLLER_ID, current_command);
                } else {
                    // Disarmed - send zero current
                    comm_can_set_current(CRSF_VESC_CONTROLLER_ID, 0.0f);
                }
            } else {
                // No connection or failsafe - apply failsafe behavior
                if (CRSF_FAILSAFE_ENABLE_BRAKE) {
                    comm_can_set_current_brake(CRSF_VESC_CONTROLLER_ID, CRSF_FAILSAFE_BRAKE_CURRENT);
                } else {
                    comm_can_set_current(CRSF_VESC_CONTROLLER_ID, CRSF_FAILSAFE_CURRENT);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting VESC Express Encoder CRSF Control");
    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "Encoder Type: %s", encoder_get_type_name_static());
    
    // Initialize CAN communication
    #ifdef CAN_TX_GPIO_NUM
    comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
    ESP_LOGI(TAG, "CAN communication started");
    #endif

    // Initialize encoder system
    if (encoder_init()) {
        ESP_LOGI(TAG, "Encoder system initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize encoder system");
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
    ESP_LOGI(TAG, "Control Channel: %d, PID: Kp=%.2f Ki=%.3f Kd=%.3f", 
             CONTROL_CHANNEL, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    ESP_LOGI(TAG, "Angle Range: %.1f° to %.1f°", MIN_ANGLE, MAX_ANGLE);
}