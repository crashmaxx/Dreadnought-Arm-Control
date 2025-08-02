#ifndef ESPNOW_TELEMETRY_H
#define ESPNOW_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_err.h"

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_telemetry_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    TELEMETRY_ESPNOW_SEND_CB,
    TELEMETRY_ESPNOW_RECV_CB,
} telemetry_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} telemetry_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} telemetry_espnow_event_recv_cb_t;

typedef union {
    telemetry_espnow_event_send_cb_t send_cb;
    telemetry_espnow_event_recv_cb_t recv_cb;
} telemetry_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    telemetry_espnow_event_id_t id;
    telemetry_espnow_event_info_t info;
} telemetry_espnow_event_t;

enum {
    TELEMETRY_ESPNOW_DATA_BROADCAST,
    TELEMETRY_ESPNOW_DATA_UNICAST,
    TELEMETRY_ESPNOW_DATA_MAX,
};

/* Telemetry payload structure for ESP-NOW data */
typedef struct {
    char board_name[16];                  // Board name (null-terminated string)
    float encoder_degrees;                // Current encoder position in degrees
    float crsf_target_degrees;            // CRSF target position in degrees
    float vesc_target_revolutions;        // VESC target position in revolutions
} __attribute__((packed)) telemetry_payload_t;

/* User defined field of ESPNOW data in this telemetry. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) telemetry_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} telemetry_espnow_send_param_t;

// Function declarations
esp_err_t telemetry_espnow_init(void);
void telemetry_wifi_init(void);
esp_err_t telemetry_espnow_send_data(const uint8_t *dest_mac, const void *data, size_t data_len);
void telemetry_espnow_data_prepare(telemetry_espnow_send_param_t *send_param);
void telemetry_espnow_set_payload_data(const char *board_name, float encoder_degrees, float crsf_target_degrees, float vesc_target_revolutions);

#endif