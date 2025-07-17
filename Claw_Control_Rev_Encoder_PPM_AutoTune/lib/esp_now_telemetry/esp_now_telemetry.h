#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct __attribute__((packed)) {
    char name[16];
    float value1;
    float value2;
    float value3;
} TelemetryPacket;

// Initialize ESP-NOW and add a peer (receiver)
bool setTelemetryPeer(const uint8_t *peer_addr);

// Send telemetry data to the configured peer
void sendTelemetry(const char* name, float v1, float v2, float v3);

// Callback type for received telemetry
typedef void (*TelemetryCallback)(const TelemetryPacket* packet, const uint8_t* mac, int len);

// Register a callback to receive telemetry messages
void getTelemetry(TelemetryCallback cb);
