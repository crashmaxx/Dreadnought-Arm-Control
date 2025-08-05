#include "crsf.h"

// CRC8 lookup table for CRSF (polynom = 0xD5)
static const uint8_t crc8tab[256] = {
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

CrsfSerial::CrsfSerial(HardwareSerial& uart, int8_t rxPin, int8_t txPin, bool halfDuplex) 
    : _uart(uart), _rxPin(rxPin), _txPin(txPin), _halfDuplex(halfDuplex), _lastChannelUpdate(0) {
    // Initialize default channel values (all centered)
    for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; i++) {
        _channels[i] = CRSF_RC_CHANNEL_CENTER;
    }
}

void CrsfSerial::begin(uint32_t baudrate) {
    // Begin UART with specified pins
    _uart.begin(baudrate, SERIAL_8N1, _rxPin, _txPin);
    
    Serial.println("CRSF UART initialized");
    Serial.print("Baudrate: ");
    Serial.println(baudrate);
    Serial.print("RX Pin: ");
    Serial.println(_rxPin);
    Serial.print("TX Pin: ");
    Serial.println(_txPin);
    Serial.print("Half-duplex: ");
    Serial.println(_halfDuplex ? "YES" : "NO");
}

uint8_t CrsfSerial::crc8(const uint8_t* ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}

void CrsfSerial::setChannel(uint8_t channel, uint16_t value) {
    // CRSF channels are 1-16, convert to 0-15 for internal array indexing
    if (channel >= 1 && channel <= CRSF_RC_CHANNEL_COUNT) {
        // Constrain value to valid CRSF range
        if (value < CRSF_RC_CHANNEL_MIN) value = CRSF_RC_CHANNEL_MIN;
        if (value > CRSF_RC_CHANNEL_MAX) value = CRSF_RC_CHANNEL_MAX;
        
        _channels[channel - 1] = value; // Convert to 0-based indexing
    }
}

void CrsfSerial::setChannelUs(uint8_t channel, uint16_t microseconds) {
    // CRSF channels are 1-16
    if (channel >= 1 && channel <= CRSF_RC_CHANNEL_COUNT) {
        // Convert microseconds (1000-2000) to CRSF ticks
        uint16_t crsfValue = CRSF_US_TO_TICKS(microseconds);
        setChannel(channel, crsfValue);
    }
}

void CrsfSerial::setChannelFloat(uint8_t channel, float value) {
    // CRSF channels are 1-16
    if (channel >= 1 && channel <= CRSF_RC_CHANNEL_COUNT) {
        // Convert float (-1.0 to +1.0) to CRSF ticks
        // -1.0 = min, 0.0 = center, +1.0 = max
        float range = (CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN) / 2.0f;
        uint16_t crsfValue = CRSF_RC_CHANNEL_CENTER + (int16_t)(value * range);
        setChannel(channel, crsfValue);
    }
}

void CrsfSerial::packChannels(uint8_t* buffer) {
    // Pack 16 channels into 22 bytes using 11-bit resolution
    // This is more complex than using the struct due to bit alignment
    uint32_t bits = 0;
    uint8_t bitsAvailable = 0;
    uint8_t byteIndex = 0;
    
    for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; i++) {
        uint32_t channelValue = _channels[i] & CRSF_RC_CHANNEL_MASK;
        bits |= channelValue << bitsAvailable;
        bitsAvailable += CRSF_RC_CHANNEL_BITS;
        
        while (bitsAvailable >= 8) {
            buffer[byteIndex++] = bits & 0xFF;
            bits >>= 8;
            bitsAvailable -= 8;
        }
    }
}

void CrsfSerial::sendChannels() {
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t frameIndex = 0;
    
    // Frame header
    frame[frameIndex++] = CRSF_SYNC_BYTE;               // Sync byte
    frame[frameIndex++] = 24;                           // Frame length (22 payload + 1 type + 1 CRC)
    frame[frameIndex++] = CRSF_FRAMETYPE_RC_CHANNELS;   // Frame type
    
    // Pack channel data
    packChannels(&frame[frameIndex]);
    frameIndex += 22; // 22 bytes for 16 channels
    
    // Calculate CRC over type and payload (not sync and length)
    uint8_t crcData[23]; // type + payload
    crcData[0] = CRSF_FRAMETYPE_RC_CHANNELS;
    memcpy(&crcData[1], &frame[3], 22);
    
    frame[frameIndex++] = crc8(crcData, 23);
    
    // Send frame
    _uart.write(frame, frameIndex);
    _uart.flush(); // Ensure all data is transmitted
    
    _lastChannelUpdate = millis();
}

void CrsfSerial::update() {
    // For ELRS, we typically send channels at 250Hz (4ms) or 500Hz (2ms)
    // Let's use 250Hz to be safe
    if (millis() - _lastChannelUpdate >= 4) {
        sendChannels();
    }
}

uint16_t CrsfSerial::getChannel(uint8_t channel) {
    // CRSF channels are 1-16, convert to 0-15 for internal array indexing
    if (channel >= 1 && channel <= CRSF_RC_CHANNEL_COUNT) {
        return _channels[channel - 1]; // Convert to 0-based indexing
    }
    return CRSF_RC_CHANNEL_CENTER;
}

uint16_t CrsfSerial::getChannelUs(uint8_t channel) {
    // CRSF channels are 1-16
    if (channel >= 1 && channel <= CRSF_RC_CHANNEL_COUNT) {
        return CRSF_TICKS_TO_US(_channels[channel - 1]); // Convert to 0-based indexing
    }
    return 1500; // Center value in microseconds
}

float CrsfSerial::getChannelFloat(uint8_t channel) {
    // CRSF channels are 1-16
    if (channel >= 1 && channel <= CRSF_RC_CHANNEL_COUNT) {
        float range = (CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN) / 2.0f;
        return (_channels[channel - 1] - CRSF_RC_CHANNEL_CENTER) / range; // Convert to 0-based indexing
    }
    return 0.0f; // Center value as float
}
