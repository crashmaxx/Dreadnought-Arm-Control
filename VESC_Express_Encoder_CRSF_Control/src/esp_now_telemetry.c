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

#include "esp_now_telemetry.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

// MAC address formatting macros
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

static const char *TAG = "ESP_NOW_TELEMETRY";

// ESP-NOW telemetry state
typedef struct {
    bool initialized;
    uint8_t peer_mac[6];
    bool peer_added;
    esp_now_telemetry_callback_t recv_callback;
    
    // Statistics
    uint32_t send_count;
    uint32_t send_errors;
    uint32_t recv_count;
    
    // Synchronization
    SemaphoreHandle_t mutex;
} esp_now_telemetry_state_t;

static esp_now_telemetry_state_t telemetry_state = {0};

// ESP-NOW send callback
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (xSemaphoreTake(telemetry_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            telemetry_state.send_count++;
        } else {
            telemetry_state.send_errors++;
            ESP_LOGW(TAG, "Send failed to " MACSTR, MAC2STR(mac_addr));
        }
        xSemaphoreGive(telemetry_state.mutex);
    }
}

// ESP-NOW receive callback
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(esp_now_telemetry_packet_t) && telemetry_state.recv_callback) {
        if (xSemaphoreTake(telemetry_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            telemetry_state.recv_count++;
            xSemaphoreGive(telemetry_state.mutex);
        }
        
        // Call user callback
        telemetry_state.recv_callback((const esp_now_telemetry_packet_t*)data, recv_info->src_addr, len);
    }
}

bool esp_now_telemetry_init(const esp_now_telemetry_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "Configuration is NULL");
        return false;
    }
    
    if (telemetry_state.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }
    
    // Create mutex for thread safety
    telemetry_state.mutex = xSemaphoreCreateMutex();
    if (!telemetry_state.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(ret));
        esp_wifi_deinit();
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
        esp_wifi_deinit();
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    // Set WiFi channel if specified
    if (config->wifi_channel > 0) {
        ret = esp_wifi_set_channel(config->wifi_channel, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WiFi set channel failed: %s", esp_err_to_name(ret));
        }
    }
    
    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        esp_wifi_stop();
        esp_wifi_deinit();
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    // Register callbacks
    ret = esp_now_register_send_cb(esp_now_send_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW register send callback failed: %s", esp_err_to_name(ret));
        esp_now_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW register recv callback failed: %s", esp_err_to_name(ret));
        esp_now_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    // Store configuration
    memcpy(telemetry_state.peer_mac, config->peer_mac, 6);
    telemetry_state.recv_callback = config->recv_callback;
    
    // Add peer
    if (!esp_now_telemetry_add_peer(config->peer_mac, config->wifi_channel, config->encrypt)) {
        ESP_LOGE(TAG, "Failed to add peer");
        esp_now_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();
        vSemaphoreDelete(telemetry_state.mutex);
        return false;
    }
    
    // Initialize statistics
    telemetry_state.send_count = 0;
    telemetry_state.send_errors = 0;
    telemetry_state.recv_count = 0;
    telemetry_state.initialized = true;
    
    ESP_LOGI(TAG, "ESP-NOW telemetry initialized successfully");
    ESP_LOGI(TAG, "Peer MAC: " MACSTR, MAC2STR(config->peer_mac));
    
    return true;
}

bool esp_now_telemetry_deinit(void) {
    if (!telemetry_state.initialized) {
        return true;
    }
    
    // Remove peer if added
    if (telemetry_state.peer_added) {
        esp_now_del_peer(telemetry_state.peer_mac);
    }
    
    // Deinitialize ESP-NOW
    esp_now_deinit();
    
    // Stop and deinitialize WiFi
    esp_wifi_stop();
    esp_wifi_deinit();
    
    // Delete mutex
    if (telemetry_state.mutex) {
        vSemaphoreDelete(telemetry_state.mutex);
    }
    
    // Reset state
    memset(&telemetry_state, 0, sizeof(telemetry_state));
    
    ESP_LOGI(TAG, "ESP-NOW telemetry deinitialized");
    return true;
}

bool esp_now_telemetry_send(const char* name, float v1, float v2, float v3) {
    if (!telemetry_state.initialized || !telemetry_state.peer_added) {
        ESP_LOGW(TAG, "Telemetry not ready for sending");
        return false;
    }
    
    esp_now_telemetry_packet_t packet;
    
    // Fill packet data
    strncpy(packet.name, name, sizeof(packet.name) - 1);
    packet.name[sizeof(packet.name) - 1] = '\0';
    packet.value1 = v1;
    packet.value2 = v2;
    packet.value3 = v3;
    packet.timestamp_ms = esp_timer_get_time() / 1000;
    
    return esp_now_telemetry_send_packet(&packet);
}

bool esp_now_telemetry_send_packet(const esp_now_telemetry_packet_t* packet) {
    if (!telemetry_state.initialized || !telemetry_state.peer_added || !packet) {
        return false;
    }
    
    esp_err_t ret = esp_now_send(telemetry_state.peer_mac, (uint8_t*)packet, sizeof(*packet));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    return true;
}

bool esp_now_telemetry_add_peer(const uint8_t* peer_mac, uint8_t channel, bool encrypt) {
    if (!telemetry_state.initialized || !peer_mac) {
        return false;
    }
    
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, peer_mac, 6);
    peer_info.channel = channel;
    peer_info.encrypt = encrypt;
    
    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer " MACSTR ": %s", MAC2STR(peer_mac), esp_err_to_name(ret));
        return false;
    }
    
    telemetry_state.peer_added = true;
    ESP_LOGI(TAG, "Added peer " MACSTR, MAC2STR(peer_mac));
    
    return true;
}

bool esp_now_telemetry_remove_peer(const uint8_t* peer_mac) {
    if (!telemetry_state.initialized || !peer_mac) {
        return false;
    }
    
    esp_err_t ret = esp_now_del_peer(peer_mac);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to remove peer " MACSTR ": %s", MAC2STR(peer_mac), esp_err_to_name(ret));
        return false;
    }
    
    if (memcmp(peer_mac, telemetry_state.peer_mac, 6) == 0) {
        telemetry_state.peer_added = false;
    }
    
    ESP_LOGI(TAG, "Removed peer " MACSTR, MAC2STR(peer_mac));
    return true;
}

bool esp_now_telemetry_is_ready(void) {
    return telemetry_state.initialized && telemetry_state.peer_added;
}

uint32_t esp_now_telemetry_get_send_count(void) {
    uint32_t count = 0;
    if (telemetry_state.mutex && xSemaphoreTake(telemetry_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = telemetry_state.send_count;
        xSemaphoreGive(telemetry_state.mutex);
    }
    return count;
}

uint32_t esp_now_telemetry_get_send_errors(void) {
    uint32_t errors = 0;
    if (telemetry_state.mutex && xSemaphoreTake(telemetry_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        errors = telemetry_state.send_errors;
        xSemaphoreGive(telemetry_state.mutex);
    }
    return errors;
}

uint32_t esp_now_telemetry_get_recv_count(void) {
    uint32_t count = 0;
    if (telemetry_state.mutex && xSemaphoreTake(telemetry_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = telemetry_state.recv_count;
        xSemaphoreGive(telemetry_state.mutex);
    }
    return count;
}

void esp_now_telemetry_reset_stats(void) {
    if (telemetry_state.mutex && xSemaphoreTake(telemetry_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        telemetry_state.send_count = 0;
        telemetry_state.send_errors = 0;
        telemetry_state.recv_count = 0;
        xSemaphoreGive(telemetry_state.mutex);
    }
}
