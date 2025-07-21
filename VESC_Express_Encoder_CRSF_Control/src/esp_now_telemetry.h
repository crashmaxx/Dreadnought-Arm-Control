/*
	Copyright 2025

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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Telemetry packet structure (packed for consistent transmission)
typedef struct __attribute__((packed)) {
    char name[16];          // Name/identifier for the telemetry data
    float value1;           // First data value
    float value2;           // Second data value  
    float value3;           // Third data value
    uint32_t timestamp_ms;  // Timestamp in milliseconds
} esp_now_telemetry_packet_t;

// Callback type for received telemetry
typedef void (*esp_now_telemetry_callback_t)(const esp_now_telemetry_packet_t* packet, const uint8_t* mac, int len);

// Configuration structure for ESP-NOW telemetry
typedef struct {
    uint8_t peer_mac[6];                        // MAC address of peer device
    uint8_t wifi_channel;                       // WiFi channel (0 = current channel)
    bool encrypt;                               // Enable encryption
    esp_now_telemetry_callback_t recv_callback; // Callback for received packets
} esp_now_telemetry_config_t;

/**
 * @brief Initialize ESP-NOW telemetry system
 * @param config Configuration structure with peer MAC, channel, etc.
 * @return true if initialization successful, false otherwise
 */
bool esp_now_telemetry_init(const esp_now_telemetry_config_t* config);

/**
 * @brief Deinitialize ESP-NOW telemetry system
 * @return true if deinitialization successful, false otherwise
 */
bool esp_now_telemetry_deinit(void);

/**
 * @brief Send telemetry data to the configured peer
 * @param name Identifier string for the telemetry data (max 15 chars)
 * @param v1 First data value
 * @param v2 Second data value
 * @param v3 Third data value
 * @return true if send successful, false otherwise
 */
bool esp_now_telemetry_send(const char* name, float v1, float v2, float v3);

/**
 * @brief Send raw telemetry packet to the configured peer
 * @param packet Pointer to telemetry packet structure
 * @return true if send successful, false otherwise
 */
bool esp_now_telemetry_send_packet(const esp_now_telemetry_packet_t* packet);

/**
 * @brief Add a new peer for telemetry communication
 * @param peer_mac MAC address of the peer device
 * @param channel WiFi channel (0 = current channel)
 * @param encrypt Enable encryption for this peer
 * @return true if peer added successfully, false otherwise
 */
bool esp_now_telemetry_add_peer(const uint8_t* peer_mac, uint8_t channel, bool encrypt);

/**
 * @brief Remove a peer from telemetry communication
 * @param peer_mac MAC address of the peer device to remove
 * @return true if peer removed successfully, false otherwise
 */
bool esp_now_telemetry_remove_peer(const uint8_t* peer_mac);

/**
 * @brief Check if ESP-NOW telemetry is initialized and ready
 * @return true if ready, false otherwise
 */
bool esp_now_telemetry_is_ready(void);

/**
 * @brief Get the number of successfully sent packets
 * @return Number of packets sent successfully
 */
uint32_t esp_now_telemetry_get_send_count(void);

/**
 * @brief Get the number of failed send attempts
 * @return Number of failed send attempts
 */
uint32_t esp_now_telemetry_get_send_errors(void);

/**
 * @brief Get the number of received packets
 * @return Number of packets received
 */
uint32_t esp_now_telemetry_get_recv_count(void);

/**
 * @brief Reset telemetry statistics counters
 */
void esp_now_telemetry_reset_stats(void);

#ifdef __cplusplus
}
#endif
