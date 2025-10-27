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

// Data structure for encoder state
typedef struct {
    float current_angle;
    float calibration_offset;
    bool calibrated;
    uint32_t boot_count;
    uint32_t last_save_time;
} fram_encoder_data_t;

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

#endif /* FRAM_I2C_H_ */