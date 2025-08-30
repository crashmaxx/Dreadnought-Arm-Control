#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "comm_espnow.h"
#include "board_config.h"  // For PEER_MAC_ADDR
#include "debug_config.h"  // For debug configuration

// Debug configuration for ESP-NOW telemetry - set to 1 to enable, 0 to disable
#if DEBUG_ESPNOW_TEST
#define DEBUG_ESPNOW_INTERNAL       1  // ESP-NOW internal debugging (enabled by DEBUG_ESPNOW_TEST)
#else
#define DEBUG_ESPNOW_INTERNAL       0  // ESP-NOW internal debugging
#endif

#if DEBUG_ESPNOW_INTERNAL
#define DEBUG_ESPNOW_INT(fmt, ...) ESP_LOGI(TAG, "[ESPNOW_INT] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_ESPNOW_INT(fmt, ...)
#endif

// MACSTR and MAC2STR macros for MAC address formatting
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

// ESP-NOW configuration defaults
#ifndef CONFIG_ESPNOW_CHANNEL
#define CONFIG_ESPNOW_CHANNEL 1
#endif

#ifndef CONFIG_ESPNOW_SEND_DELAY
#define CONFIG_ESPNOW_SEND_DELAY 100
#endif

#ifndef CONFIG_ESPNOW_SEND_LEN
#define CONFIG_ESPNOW_SEND_LEN 200
#endif

#ifndef CONFIG_ESPNOW_PMK
#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#endif

#ifndef CONFIG_ESPNOW_LMK
#define CONFIG_ESPNOW_LMK "lmk1234567890123"
#endif

#ifndef CONFIG_ESPNOW_ENABLE_LONG_RANGE
#define CONFIG_ESPNOW_ENABLE_LONG_RANGE 0
#endif

#ifndef CONFIG_ESPNOW_ENABLE_POWER_SAVE
#define CONFIG_ESPNOW_ENABLE_POWER_SAVE 0
#endif

#ifndef CONFIG_ESPNOW_WAKE_WINDOW
#define CONFIG_ESPNOW_WAKE_WINDOW 50
#endif

#ifndef CONFIG_ESPNOW_WAKE_INTERVAL
#define CONFIG_ESPNOW_WAKE_INTERVAL 100
#endif

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_telemetry";

static QueueHandle_t s_telemetry_espnow_queue = NULL;

static uint8_t s_telemetry_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_telemetry_peer_mac[ESP_NOW_ETH_ALEN] = PEER_MAC_ADDR;  // Specific peer MAC from board config
static uint16_t s_telemetry_espnow_seq[TELEMETRY_ESPNOW_DATA_MAX] = { 0, 0 };

// Global telemetry data storage
static telemetry_payload_t s_telemetry_data = {
    .board_name = "Unknown",
    .encoder_degrees = 0.0f,
    .crsf_target_degrees = 0.0f,
    .vesc_target_revolutions = 0.0f
};

static void telemetry_espnow_deinit(telemetry_espnow_send_param_t *send_param);

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void telemetry_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    telemetry_espnow_event_t evt;
    telemetry_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = TELEMETRY_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_telemetry_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void telemetry_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    telemetry_espnow_event_t evt;
    telemetry_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = TELEMETRY_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_telemetry_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int telemetry_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    telemetry_espnow_data_t *buf = (telemetry_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(telemetry_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Update telemetry payload data */
void telemetry_espnow_set_payload_data(const char *board_name, float encoder_degrees, float crsf_target_degrees, float vesc_target_revolutions)
{
    // Copy board name safely (ensure null termination)
    strncpy(s_telemetry_data.board_name, board_name, sizeof(s_telemetry_data.board_name) - 1);
    s_telemetry_data.board_name[sizeof(s_telemetry_data.board_name) - 1] = '\0';
    
    // Update telemetry values
    s_telemetry_data.encoder_degrees = encoder_degrees;
    s_telemetry_data.crsf_target_degrees = crsf_target_degrees;
    s_telemetry_data.vesc_target_revolutions = vesc_target_revolutions;
    
    // Debug output (can be enabled/disabled)
    static uint32_t last_debug = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_debug > 2000) {  // Debug every 2 seconds
        ESP_LOGI(TAG, "Telemetry update: %s, Enc:%.1f°, CRSF:%.1f°, VESC:%.3frev", 
                board_name, encoder_degrees, crsf_target_degrees, vesc_target_revolutions);
        last_debug = current_time;
    }
}

/* Prepare ESPNOW data to be sent. */
void telemetry_espnow_data_prepare(telemetry_espnow_send_param_t *send_param)
{
    telemetry_espnow_data_t *buf = (telemetry_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(telemetry_espnow_data_t) + sizeof(telemetry_payload_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? TELEMETRY_ESPNOW_DATA_BROADCAST : TELEMETRY_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_telemetry_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    
    // Copy actual telemetry data instead of random data
    memcpy(buf->payload, &s_telemetry_data, sizeof(telemetry_payload_t));
    
    // Fill any remaining bytes with zeros
    size_t telemetry_size = sizeof(telemetry_espnow_data_t) + sizeof(telemetry_payload_t);
    if (send_param->len > telemetry_size) {
        memset((uint8_t *)buf + telemetry_size, 0, send_param->len - telemetry_size);
    }
    
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void telemetry_espnow_task(void *pvParameter)
{
    telemetry_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    // Give WiFi interface time to be fully ready for ESP-NOW
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "ESP-NOW task starting - sending to peer "MACSTR, MAC2STR(s_telemetry_peer_mac));

    /* Start sending broadcast ESPNOW data. */
    telemetry_espnow_send_param_t *send_param = (telemetry_espnow_send_param_t *)pvParameter;
    ESP_LOGI(TAG, "Sending first ESP-NOW packet...");
    esp_err_t send_result = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
    if (send_result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW send failed: %s (%d)", esp_err_to_name(send_result), send_result);
        telemetry_espnow_deinit(send_param);
        vTaskDelete(NULL);
    } else {
        ESP_LOGI(TAG, "ESP-NOW packet sent successfully");
    }

    while (xQueueReceive(s_telemetry_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case TELEMETRY_ESPNOW_SEND_CB:
            {
                telemetry_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    // Continuous sending - no count limit
                    // Just continue sending telemetry data
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                ESP_LOGD(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                telemetry_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    telemetry_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case TELEMETRY_ESPNOW_RECV_CB:
            {
                telemetry_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = telemetry_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == TELEMETRY_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            telemetry_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = false;  // Changed from true to false for consistency
                        // Removed LMK key setting since encrypt = false
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            telemetry_espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                telemetry_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == TELEMETRY_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

esp_err_t telemetry_espnow_init(void)
{
    telemetry_espnow_send_param_t *send_param;

    s_telemetry_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(telemetry_espnow_event_t));
    if (s_telemetry_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(telemetry_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(telemetry_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key - COMMENTED OUT FOR COMPATIBILITY */
    // ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_telemetry_espnow_queue);
        s_telemetry_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_telemetry_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    
    // Also add the specific peer MAC address from board config
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_telemetry_peer_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    DEBUG_ESPNOW_INT("Added ESP-NOW peer: "MACSTR, MAC2STR(s_telemetry_peer_mac));
    
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(telemetry_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_telemetry_espnow_queue);
        s_telemetry_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(telemetry_espnow_send_param_t));
    send_param->unicast = true;   // Use unicast to specific peer
    send_param->broadcast = false; // Not broadcast
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    // Calculate exact size needed for telemetry data
    send_param->len = sizeof(telemetry_espnow_data_t) + sizeof(telemetry_payload_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_telemetry_espnow_queue);
        s_telemetry_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_telemetry_peer_mac, ESP_NOW_ETH_ALEN);  // Use specific peer MAC
    telemetry_espnow_data_prepare(send_param);

    ESP_LOGI(TAG, "Creating ESP-NOW telemetry task with peer "MACSTR, MAC2STR(s_telemetry_peer_mac));
    ESP_LOGI(TAG, "Telemetry packet size: %d bytes, delay: %dms", send_param->len, send_param->delay);
    xTaskCreate(telemetry_espnow_task, "telemetry_espnow_task", 8192, send_param, 4, NULL);  // Increased from 2048 to 8192 bytes

    return ESP_OK;
}

static void telemetry_espnow_deinit(telemetry_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_telemetry_espnow_queue);
    s_telemetry_espnow_queue = NULL;
    esp_now_deinit();
}

/* Send custom telemetry data via ESP-NOW */
esp_err_t telemetry_espnow_send_data(const uint8_t *dest_mac, const void *data, size_t data_len)
{
    if (s_telemetry_espnow_queue == NULL) {
        ESP_LOGE(TAG, "ESP-NOW not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (dest_mac == NULL || data == NULL || data_len == 0) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate total packet size (header + payload)
    size_t packet_len = sizeof(telemetry_espnow_data_t) + data_len;
    
    // Allocate buffer for the complete packet
    uint8_t *packet_buffer = malloc(packet_len);
    if (packet_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate packet buffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Fill the ESP-NOW header
    telemetry_espnow_data_t *packet = (telemetry_espnow_data_t *)packet_buffer;
    packet->type = IS_BROADCAST_ADDR(dest_mac) ? TELEMETRY_ESPNOW_DATA_BROADCAST : TELEMETRY_ESPNOW_DATA_UNICAST;
    packet->state = 1;  // Ready state
    packet->seq_num = s_telemetry_espnow_seq[packet->type]++;
    packet->magic = esp_random();
    packet->crc = 0;    // Will be calculated below
    
    // Copy the payload data
    memcpy(packet->payload, data, data_len);
    
    // Calculate CRC over the entire packet
    packet->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)packet, packet_len);
    
    // Send the packet
    esp_err_t ret = esp_now_send(dest_mac, packet_buffer, packet_len);
    
    // Clean up
    free(packet_buffer);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(ret));  // Always show errors
        return ret;
    }
    
    DEBUG_ESPNOW_INT("Sent %d bytes to " MACSTR, data_len, MAC2STR(dest_mac));
    return ESP_OK;
}