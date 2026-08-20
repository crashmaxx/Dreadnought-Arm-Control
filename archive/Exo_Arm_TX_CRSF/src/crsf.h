#pragma once

#include <Arduino.h>
#include "crsf_protocol.h"

class CrsfSerial {
public:
    CrsfSerial(HardwareSerial& uart, int8_t rxPin = -1, int8_t txPin = -1, bool halfDuplex = true);
    
    // Initialize UART communication
    void begin(uint32_t baudrate = 400000);
    
    // Channel manipulation methods
    void setChannel(uint8_t channel, uint16_t value);           // Set channel with raw CRSF value
    void setChannelUs(uint8_t channel, uint16_t microseconds);  // Set channel with microsecond value (1000-2000)
    void setChannelFloat(uint8_t channel, float value);         // Set channel with float value (-1.0 to +1.0)
    
    uint16_t getChannel(uint8_t channel);                       // Get raw CRSF channel value
    uint16_t getChannelUs(uint8_t channel);                     // Get channel as microseconds
    float getChannelFloat(uint8_t channel);                     // Get channel as float
    
    // Send RC channels frame
    void sendChannels();
    
    // Call this regularly to maintain transmission rate
    void update();
    
private:
    HardwareSerial& _uart;
    int8_t _rxPin;
    int8_t _txPin;
    bool _halfDuplex;
    uint16_t _channels[CRSF_RC_CHANNEL_COUNT];
    uint32_t _lastChannelUpdate;
    
    // Internal methods
    uint8_t crc8(const uint8_t* ptr, uint8_t len);
    void packChannels(uint8_t* buffer);
};
