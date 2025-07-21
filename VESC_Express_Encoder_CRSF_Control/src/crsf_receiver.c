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

#include "crsf_receiver.h"
#include "crsf_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "CRSF";

// UART configuration
static int crsf_uart_num = UART_NUM_1;
static QueueHandle_t uart_queue = NULL;

// CRSF data
static crsf_channels_t crsf_channels = {0};
static uint8_t rx_buffer[CRSF_FRAME_SIZE_MAX];
static uint8_t rx_index = 0;
static bool new_data = false;

// CRC8 lookup table (DVB-S2 polynomial)
static const uint8_t crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

// CRC8 calculation
uint8_t crsf_crc8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

// Validate CRSF frame
bool crsf_validate_frame(const uint8_t *frame, uint8_t length) {
    if (length < 4) return false;  // Minimum frame size
    if (frame[0] != CRSF_SYNC_BYTE) return false;
    if (frame[1] != length - 2) return false;  // Length field
    
    uint8_t calculated_crc = crsf_crc8(&frame[2], length - 3);
    return calculated_crc == frame[length - 1];
}

// Parse RC channels from CRSF frame
void crsf_parse_rc_channels(const uint8_t *payload) {
    // CRSF channels are packed in 11-bit values
    // 16 channels * 11 bits = 176 bits = 22 bytes
    
    const uint8_t *data = payload;
    
    crsf_channels.channels[0]  = (uint16_t)(data[0] | data[1] << 8) & 0x07FF;
    crsf_channels.channels[1]  = (uint16_t)(data[1] >> 3 | data[2] << 5) & 0x07FF;
    crsf_channels.channels[2]  = (uint16_t)(data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF;
    crsf_channels.channels[3]  = (uint16_t)(data[4] >> 1 | data[5] << 7) & 0x07FF;
    crsf_channels.channels[4]  = (uint16_t)(data[5] >> 4 | data[6] << 4) & 0x07FF;
    crsf_channels.channels[5]  = (uint16_t)(data[6] >> 7 | data[7] << 1 | data[8] << 9) & 0x07FF;
    crsf_channels.channels[6]  = (uint16_t)(data[8] >> 2 | data[9] << 6) & 0x07FF;
    crsf_channels.channels[7]  = (uint16_t)(data[9] >> 5 | data[10] << 3) & 0x07FF;
    crsf_channels.channels[8]  = (uint16_t)(data[11] | data[12] << 8) & 0x07FF;
    crsf_channels.channels[9]  = (uint16_t)(data[12] >> 3 | data[13] << 5) & 0x07FF;
    crsf_channels.channels[10] = (uint16_t)(data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF;
    crsf_channels.channels[11] = (uint16_t)(data[15] >> 1 | data[16] << 7) & 0x07FF;
    crsf_channels.channels[12] = (uint16_t)(data[16] >> 4 | data[17] << 4) & 0x07FF;
    crsf_channels.channels[13] = (uint16_t)(data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF;
    crsf_channels.channels[14] = (uint16_t)(data[19] >> 2 | data[20] << 6) & 0x07FF;
    crsf_channels.channels[15] = (uint16_t)(data[20] >> 5 | data[21] << 3) & 0x07FF;

    // Check for failsafe flag (bit 22 of the data)
    crsf_channels.failsafe = (data[22] & 0x01) != 0;
    
    crsf_channels.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
    crsf_channels.valid = true;
    new_data = true;
}

// Parse CRSF frame
void crsf_parse_frame(const uint8_t *frame, uint8_t length) {
    if (!crsf_validate_frame(frame, length)) {
        return;
    }
    
    uint8_t frame_type = frame[CRSF_FRAME_TYPE_ADDR];
    const uint8_t *payload = &frame[CRSF_FRAME_PAYLOAD_ADDR];
    
    switch (frame_type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            crsf_parse_rc_channels(payload);
            break;
        default:
            // Ignore other frame types for now
            break;
    }
}

// UART task for receiving CRSF data
static void crsf_uart_task(void *pvParameters) {
    uint8_t data;
    
    while (1) {
        int len = uart_read_bytes(crsf_uart_num, &data, 1, portMAX_DELAY);
        
        if (len > 0) {
            if (data == CRSF_SYNC_BYTE && rx_index > 0) {
                // New frame starting, reset buffer
                rx_index = 0;
            }
            
            rx_buffer[rx_index++] = data;
            
            // Check if we have at least sync + length
            if (rx_index >= 2) {
                uint8_t expected_length = rx_buffer[1] + 2;  // Length field + sync + length bytes
                
                if (rx_index >= expected_length) {
                    // Complete frame received
                    crsf_parse_frame(rx_buffer, expected_length);
                    rx_index = 0;
                }
                
                // Prevent buffer overflow
                if (rx_index >= CRSF_FRAME_SIZE_MAX) {
                    rx_index = 0;
                }
            }
        }
    }
}

// Initialize CRSF receiver
void crsf_init(int uart_num, int tx_pin, int rx_pin, int baudrate) {
    crsf_uart_num = uart_num;
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(crsf_uart_num, CRSF_UART_BUFFER_SIZE, CRSF_UART_BUFFER_SIZE, CRSF_UART_QUEUE_SIZE, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(crsf_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(crsf_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Initialize channel data
    memset(&crsf_channels, 0, sizeof(crsf_channels));
    for (int i = 0; i < CRSF_MAX_CHANNELS; i++) {
        crsf_channels.channels[i] = CRSF_CHANNEL_VALUE_MID;  // Set to middle position
    }
    
    // Create UART task
    xTaskCreate(crsf_uart_task, "crsf_uart", CRSF_TASK_STACK_SIZE, NULL, CRSF_TASK_PRIORITY, NULL);
    
    #if CRSF_ENABLE_LOGGING
    ESP_LOGI(TAG, "CRSF initialized on UART%d (TX:%d, RX:%d, %d baud)", 
             uart_num, tx_pin, rx_pin, baudrate);
    #endif
}

// Update function (can be called periodically if needed)
void crsf_update(void) {
    // Check for timeout (connection lost)
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (crsf_channels.valid && (current_time - crsf_channels.last_update) > CRSF_CONNECTION_TIMEOUT_MS) {
        crsf_channels.valid = false;
        crsf_channels.failsafe = true;
        #if CRSF_ENABLE_LOGGING
        ESP_LOGW(TAG, "CRSF timeout - connection lost");
        #endif
    }
}

// Check if new data is available
bool crsf_has_new_data(void) {
    bool result = new_data;
    new_data = false;
    return result;
}

// Get raw channel value (172-1811)
uint16_t crsf_get_channel(uint8_t channel) {
    if (channel == 0 || channel > CRSF_MAX_CHANNELS) {
        return CRSF_CHANNEL_VALUE_MID;
    }
    return crsf_channels.channels[channel - 1];  // Convert 1-based to 0-based indexing
}

// Get scaled channel value (1000-2000)
uint16_t crsf_get_channel_scaled(uint8_t channel) {
    uint16_t raw = crsf_get_channel(channel);
    
    // Convert from 172-1811 to 1000-2000
    float scaled = (raw - CRSF_CHANNEL_VALUE_MIN) * 1000.0f / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) + 1000.0f;
    
    // Clamp to valid range
    if (scaled < 1000.0f) scaled = 1000.0f;
    if (scaled > 2000.0f) scaled = 2000.0f;
    
    return (uint16_t)scaled;
}

// Check if in failsafe mode
bool crsf_is_failsafe(void) {
    return crsf_channels.failsafe || !crsf_channels.valid;
}

// Check if receiver is connected
bool crsf_is_connected(void) {
    return crsf_channels.valid && !crsf_channels.failsafe;
}

// Get last update time
uint32_t crsf_get_last_update_time(void) {
    return crsf_channels.last_update;
}

// Get all raw channel values
void crsf_get_all_channels(uint16_t *channels) {
    memcpy(channels, crsf_channels.channels, sizeof(crsf_channels.channels));
}

// Get all scaled channel values
void crsf_get_all_channels_scaled(uint16_t *channels) {
    for (int i = 0; i < CRSF_MAX_CHANNELS; i++) {
        channels[i] = crsf_get_channel_scaled(i + 1);
    }
}
