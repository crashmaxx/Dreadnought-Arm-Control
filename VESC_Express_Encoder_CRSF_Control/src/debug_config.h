#ifndef DEBUG_CONFIG_H_
#define DEBUG_CONFIG_H_

#include "esp_log.h"

// =============================================================================
// DEBUG CONFIGURATION
// =============================================================================
// Debug configuration - set to 1 to enable, 0 to disable
// These control what debug messages are shown in the serial monitor
#define DEBUG_POSITION_CONTROL      1  // Position control debugging
#define DEBUG_ENCODER_DATA          1  // Encoder data debugging
#define DEBUG_VESC_STATUS           1  // VESC status debugging (includes parameter updates)
#define DEBUG_CRSF_CHANNELS         1  // CRSF channel data
#define DEBUG_CAN_COMMANDS          1  // CAN command transmission
#define DEBUG_ESPNOW_TELEMETRY      1  // ESP-NOW telemetry debugging
#define DEBUG_ESPNOW_TEST           0  // Send random test data to ESP-NOW telemetry
#define DEBUG_VERBOSE               0  // Verbose SPI encoder and diagnostics debugging
#define DEBUG_INIT_COUNTDOWN        0  // Enable 30 second sensor initialization countdown

// =============================================================================
// DEBUG MACROS
// =============================================================================
#if DEBUG_VERBOSE
#define DEBUG_VERBOSE_LOG(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define DEBUG_VERBOSE_LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define DEBUG_VERBOSE_LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#define DEBUG_VERBOSE_LOGV(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)
#else
#define DEBUG_VERBOSE_LOG(tag, format, ...)
#define DEBUG_VERBOSE_LOGW(tag, format, ...)
#define DEBUG_VERBOSE_LOGD(tag, format, ...)
#define DEBUG_VERBOSE_LOGV(tag, format, ...)
#endif

#endif /* DEBUG_CONFIG_H_ */
