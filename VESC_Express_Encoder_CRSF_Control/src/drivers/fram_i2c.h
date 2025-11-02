#ifndef FRAM_I2C_H_
#define FRAM_I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// FRAM memory addresses for different data types
#define FRAM_ADDR_ENCODER_ANGLE     0x0000  // Float (4 bytes) - Current encoder angle
#define FRAM_ADDR_CALIBRATION_OFFSET 0x0004  // Float (4 bytes) - Calibration offset
#define FRAM_ADDR_CALIBRATED_FLAG   0x0008  // Bool (1 byte) - Calibration status
#define FRAM_ADDR_BOOT_COUNT        0x0009  // uint32_t (4 bytes) - Boot counter
#define FRAM_ADDR_TIMESTAMP         0x000D  // uint32_t (4 bytes) - Last save timestamp

// ESP-NOW remote data addresses (starting at 0x0020 to avoid conflicts)
#define FRAM_ADDR_REMOTE_ANGLE_CH2   0x0020  // Float (4 bytes) - Remote upper arm angle (channel 2)
#define FRAM_ADDR_REMOTE_ANGLE_CH3   0x0024  // Float (4 bytes) - Remote elbow angle (channel 3)
#define FRAM_ADDR_REMOTE_TIMESTAMP   0x0028  // uint32_t (4 bytes) - Remote data timestamp
#define FRAM_ADDR_REMOTE_VALID_CH2   0x002C  // Bool (1 byte) - Remote channel 2 validity flag
#define FRAM_ADDR_REMOTE_VALID_CH3   0x002D  // Bool (1 byte) - Remote channel 3 validity flag
#define FRAM_ADDR_COMM_STATS         0x0030  // Communication statistics area (16 bytes)

// Data structure for encoder state
typedef struct {
    float current_angle;
    float calibration_offset;
    bool calibrated;
    uint32_t boot_count;
    uint32_t last_save_time;
} fram_encoder_data_t;

// Data structure for remote ESP-NOW angle data (multi-channel)
typedef struct {
    float remote_angle_ch2;        // Upper arm angle (channel 2)
    float remote_angle_ch3;        // Elbow angle (channel 3)
    uint32_t remote_timestamp;
    bool remote_ch2_valid;         // Channel 2 data validity
    bool remote_ch3_valid;         // Channel 3 data validity
    uint32_t packets_received;
    uint32_t packets_sent;
    uint32_t last_communication_time;
} fram_remote_data_t;

/**
 * @brief Initialize I2C interface for FRAM communication
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_i2c_init(void);

/**
 * @brief Deinitialize I2C interface
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_i2c_deinit(void);

/**
 * @brief Write a single byte to FRAM
 * @param address FRAM memory address (0x0000-0x7FFF for 32KB FRAM)
 * @param data Byte to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_write_byte(uint16_t address, uint8_t data);

/**
 * @brief Read a single byte from FRAM
 * @param address FRAM memory address
 * @param data Pointer to store read byte
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_read_byte(uint16_t address, uint8_t *data);

/**
 * @brief Write multiple bytes to FRAM
 * @param address Starting FRAM memory address
 * @param data Pointer to data buffer
 * @param length Number of bytes to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_write_buffer(uint16_t address, const uint8_t *data, size_t length);

/**
 * @brief Read multiple bytes from FRAM
 * @param address Starting FRAM memory address
 * @param data Pointer to buffer for read data
 * @param length Number of bytes to read
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_read_buffer(uint16_t address, uint8_t *data, size_t length);

/**
 * @brief Write float value to FRAM
 * @param address FRAM memory address
 * @param value Float value to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_write_float(uint16_t address, float value);

/**
 * @brief Read float value from FRAM
 * @param address FRAM memory address
 * @param value Pointer to store float value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_read_float(uint16_t address, float *value);

/**
 * @brief Write uint32_t value to FRAM
 * @param address FRAM memory address
 * @param value Uint32_t value to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_write_uint32(uint16_t address, uint32_t value);

/**
 * @brief Read uint32_t value from FRAM
 * @param address FRAM memory address
 * @param value Pointer to store uint32_t value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_read_uint32(uint16_t address, uint32_t *value);

/**
 * @brief Save current encoder data to FRAM
 * @param encoder_data Encoder data structure to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_save_encoder_data(const fram_encoder_data_t *encoder_data);

/**
 * @brief Load encoder data from FRAM
 * @param encoder_data Pointer to encoder data structure to populate
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_load_encoder_data(fram_encoder_data_t *encoder_data);

/**
 * @brief Test FRAM connectivity and basic read/write operations
 * @return ESP_OK if FRAM is working correctly, error code on failure
 */
esp_err_t fram_test_connectivity(void);

/**
 * @brief Clear all FRAM data (write zeros to encoder data area)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_clear_encoder_data(void);

/**
 * @brief Save remote ESP-NOW angle data to FRAM
 * @param remote_data Remote data structure to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_save_remote_data(const fram_remote_data_t *remote_data);

/**
 * @brief Load remote ESP-NOW angle data from FRAM
 * @param remote_data Pointer to remote data structure to populate
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_load_remote_data(fram_remote_data_t *remote_data);

/**
 * @brief Update remote angle data in FRAM (quick update for frequent data)
 * @param channel Channel number (2 for upper arm, 3 for elbow)
 * @param remote_angle New remote angle value
 * @param timestamp Timestamp of the remote data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fram_update_remote_angle(uint8_t channel, float remote_angle, uint32_t timestamp);

#endif /* FRAM_I2C_H_ */