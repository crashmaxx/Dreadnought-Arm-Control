#include <esp_now.h>
#include <WiFi.h>
#include <string.h>
#include "esp_now_telemetry.h"

// Static global variables
static uint8_t global_peer_addr[6] = {0};
static bool peer_set = false;
static TelemetryCallback user_cb = nullptr;

// Function to send telemetry data via ESP-NOW
void sendTelemetry(const char* name, float v1, float v2, float v3) {
    if (!peer_set) return;
    TelemetryPacket packet;
    strncpy(packet.name, name, sizeof(packet.name) - 1);
    packet.name[sizeof(packet.name) - 1] = '\0';
    packet.value1 = v1;
    packet.value2 = v2;
    packet.value3 = v3;
    esp_now_send(global_peer_addr, (uint8_t*)&packet, sizeof(packet));
}

// Internal ESP-NOW receive callback
void onTelemetryRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (user_cb && len == sizeof(TelemetryPacket)) {
        user_cb((const TelemetryPacket*)data, mac, len);
    }
}

// Register a callback to receive telemetry messages
void getTelemetry(TelemetryCallback cb) {
    user_cb = cb;
    esp_now_register_recv_cb(onTelemetryRecv);
}

// Function to initialize ESP-NOW and add a peer (receiver)
bool setTelemetryPeer(const uint8_t *peer_addr) {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peer_addr, 6);
    memcpy(global_peer_addr, peer_addr, 6);
    peer_set = true;
    peerInfo.channel = 0;  // use current WiFi channel
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        peer_set = false;
        return false;
    }
    return true;
}
