#pragma once

// CRSF protocol definitions based on official TBS CRSF specification
// https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md

#include <stdint.h>

// CRSF Frame Format
#define CRSF_SYNC_BYTE                  0xC8
#define CRSF_MAX_FRAME_SIZE             64
#define CRSF_PAYLOAD_SIZE_MAX           62

// CRSF Frame Types
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16

// CRSF Addresses
#define CRSF_ADDRESS_BROADCAST          0x00
#define CRSF_ADDRESS_USB                0x10
#define CRSF_ADDRESS_BLUETOOTH          0x12
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO   0x80
#define CRSF_ADDRESS_CURRENT_SENSOR     0xC0
#define CRSF_ADDRESS_GPS                0xC2
#define CRSF_ADDRESS_TBS_BLACKBOX       0xC4
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8
#define CRSF_ADDRESS_RESERVED1          0xCA
#define CRSF_ADDRESS_RACE_TAG           0xCC
#define CRSF_ADDRESS_VTX                0xCE
#define CRSF_ADDRESS_RADIO_TRANSMITTER  0xEA
#define CRSF_ADDRESS_CRSF_RECEIVER      0xEC
#define CRSF_ADDRESS_CRSF_TRANSMITTER   0xEE

// RC Channels constants
#define CRSF_RC_CHANNEL_COUNT           16
#define CRSF_RC_CHANNEL_BITS            11
#define CRSF_RC_CHANNEL_MASK            0x7FF
#define CRSF_RC_CHANNEL_CENTER          992
#define CRSF_RC_CHANNEL_MIN             172
#define CRSF_RC_CHANNEL_MAX             1811

// Convert microseconds to CRSF channel value
#define CRSF_US_TO_TICKS(x)             ((x - 1500) * 8 / 5 + 992)
#define CRSF_TICKS_TO_US(x)             ((x - 992) * 5 / 8 + 1500)

// CRSF Frame structure
typedef struct {
    uint8_t sync;                // Sync byte (0xC8)
    uint8_t length;              // Frame length (not including sync and length bytes)
    uint8_t type;                // Frame type
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX]; // Payload
} crsf_frame_t;

// RC Channels packed structure (16 channels, 11 bits each)
// Note: Channels are numbered 1-16 in CRSF protocol
typedef struct {
    unsigned ch1  : 11;  // Channel 1
    unsigned ch2  : 11;  // Channel 2
    unsigned ch3  : 11;  // Channel 3
    unsigned ch4  : 11;  // Channel 4
    unsigned ch5  : 11;  // Channel 5
    unsigned ch6  : 11;  // Channel 6
    unsigned ch7  : 11;  // Channel 7
    unsigned ch8  : 11;  // Channel 8
    unsigned ch9  : 11;  // Channel 9
    unsigned ch10 : 11;  // Channel 10
    unsigned ch11 : 11;  // Channel 11
    unsigned ch12 : 11;  // Channel 12
    unsigned ch13 : 11;  // Channel 13
    unsigned ch14 : 11;  // Channel 14
    unsigned ch15 : 11;  // Channel 15
    unsigned ch16 : 11;  // Channel 16
} __attribute__((packed)) crsf_channels_t;
