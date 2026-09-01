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

#ifndef CRSF_RECEIVER_H_
#define CRSF_RECEIVER_H_

#include <stdint.h>
#include <stdbool.h>

// CRSF Protocol Constants
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_PAYLOAD_SIZE_MAX 60
#define CRSF_FRAME_NOT_COUNTED_BYTES 2
#define CRSF_FRAME_CRC_SIZE 1
#define CRSF_FRAME_LENGTH_ADDR 1
#define CRSF_FRAME_TYPE_ADDR 2
#define CRSF_FRAME_PAYLOAD_ADDR 3

// CRSF Channel Configuration
#define CRSF_MAX_CHANNELS 16
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992  
#define CRSF_CHANNEL_VALUE_MAX 1811
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

// Channel value mapping (172-1811 maps to 1000-2000)
#define CRSF_RC_CHANNEL_SCALE_OFFSET 881.0f
#define CRSF_RC_CHANNEL_SCALE_FACTOR 0.62477120195241f

// CRSF Frame structure
typedef struct {
    uint8_t sync;               // Sync byte (0xC8)
    uint8_t length;             // Frame length
    uint8_t type;               // Frame type
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX]; // Payload data
    uint8_t crc;                // CRC8
} crsf_frame_t;

// CRSF Channel data structure
typedef struct {
    uint16_t channels[CRSF_MAX_CHANNELS];   // Raw channel values (172-1811)
    uint32_t last_update;                   // Last update timestamp (ms)
    bool valid;                             // Data validity flag
    bool failsafe;                          // Failsafe state
} crsf_channels_t;

// Function declarations
void crsf_init(int uart_num, int tx_pin, int rx_pin, int baudrate);
void crsf_update(void);
bool crsf_has_new_data(void);
uint16_t crsf_get_channel(uint8_t channel);
uint16_t crsf_get_channel_scaled(uint8_t channel);
bool crsf_is_failsafe(void);
bool crsf_is_connected(void);
uint32_t crsf_get_last_update_time(void);
void crsf_get_all_channels(uint16_t *channels);
void crsf_get_all_channels_scaled(uint16_t *channels);

// Internal functions
void crsf_parse_frame(const uint8_t *frame, uint8_t length);
uint8_t crsf_crc8(const uint8_t *data, uint8_t length);
bool crsf_validate_frame(const uint8_t *frame, uint8_t length);

#endif /* CRSF_RECEIVER_H_ */
