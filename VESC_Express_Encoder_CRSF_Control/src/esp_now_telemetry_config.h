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

#ifndef ESP_NOW_TELEMETRY_CONFIG_H_
#define ESP_NOW_TELEMETRY_CONFIG_H_

// ================= ESP-NOW TELEMETRY CONFIGURATION =================

// Enable/disable ESP-NOW telemetry
#define ESP_NOW_TELEMETRY_ENABLE    1

// Default WiFi channel for ESP-NOW (1-14, 0 = use current)
#define ESP_NOW_WIFI_CHANNEL        1

// Enable encryption for ESP-NOW communication
#define ESP_NOW_ENCRYPT             false

// Telemetry transmission rates
#define ESP_NOW_TELEMETRY_RATE_MS   100     // Send telemetry every 100ms
#define ESP_NOW_STATUS_RATE_MS      1000    // Send status every 1000ms

// Maximum number of peers
#define ESP_NOW_MAX_PEERS           5

// Enable debug logging for ESP-NOW
#define ESP_NOW_DEBUG_LOGGING       1

// Telemetry packet types
typedef enum {
    TELEMETRY_TYPE_ENCODER = 0,     // Encoder position/velocity data
    TELEMETRY_TYPE_CONTROL,         // Control system data (PID output, setpoint)
    TELEMETRY_TYPE_STATUS,          // System status (armed state, errors)
    TELEMETRY_TYPE_CRSF,            // CRSF channel data
    TELEMETRY_TYPE_CUSTOM           // Custom user data
} esp_now_telemetry_type_t;

// Predefined telemetry data names
#define TELEMETRY_NAME_ENCODER      "ENCODER"
#define TELEMETRY_NAME_CONTROL      "CONTROL"
#define TELEMETRY_NAME_STATUS       "STATUS"
#define TELEMETRY_NAME_CRSF         "CRSF"
#define TELEMETRY_NAME_MOTOR        "MOTOR"
#define TELEMETRY_NAME_VOLTAGE      "VOLTAGE"
#define TELEMETRY_NAME_CURRENT      "CURRENT"
#define TELEMETRY_NAME_TEMPERATURE  "TEMP"

// Common peer MAC addresses for different system roles
// Ground Control Station
#define GCS_MAC_ADDR    {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}

// Data Logger
#define LOGGER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE5}

// Remote Display
#define DISPLAY_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE6}

// Broadcast to all devices
#define BROADCAST_MAC_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// Helper macros for telemetry data
#define TELEMETRY_SEND_ENCODER(angle, velocity, valid) \
    esp_now_telemetry_send(TELEMETRY_NAME_ENCODER, angle, velocity, valid ? 1.0f : 0.0f)

#define TELEMETRY_SEND_CONTROL(setpoint, output, error) \
    esp_now_telemetry_send(TELEMETRY_NAME_CONTROL, setpoint, output, error)

#define TELEMETRY_SEND_STATUS(armed, connected, errors) \
    esp_now_telemetry_send(TELEMETRY_NAME_STATUS, armed ? 1.0f : 0.0f, connected ? 1.0f : 0.0f, (float)errors)

#define TELEMETRY_SEND_CRSF(ch1, ch2, ch3) \
    esp_now_telemetry_send(TELEMETRY_NAME_CRSF, (float)ch1, (float)ch2, (float)ch3)

// Configuration validation
#if ESP_NOW_TELEMETRY_ENABLE && !defined(PEER_MAC_ADDR)
    #warning "ESP-NOW telemetry enabled but no peer MAC address defined. Using broadcast."
    #define PEER_MAC_ADDR BROADCAST_MAC_ADDR
#endif

#if ESP_NOW_WIFI_CHANNEL < 0 || ESP_NOW_WIFI_CHANNEL > 14
    #error "Invalid WiFi channel for ESP-NOW. Must be 0-14."
#endif

#endif /* ESP_NOW_TELEMETRY_CONFIG_H_ */
