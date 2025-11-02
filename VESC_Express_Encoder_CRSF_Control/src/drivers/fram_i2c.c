#include "fram_i2c.h"
#include "board_config.h"
#include "debug_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "FRAM_I2C";

// I2C master configuration
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS    1000
#define ACK_CHECK_EN             0x1
#define ACK_CHECK_DIS            0x0

// Debug macros with rate limiting
#if DEBUG_FRAM_I2C
#define DEBUG_FRAM_RATE_LIMIT_MS 2000  // Debug messages every 2 seconds maximum
#define DEBUG_FRAM(fmt, ...) do { \
    static uint32_t last_debug_time = 0; \
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS; \
    if (current_time - last_debug_time > DEBUG_FRAM_RATE_LIMIT_MS) { \
        ESP_LOGI(TAG, "[FRAM] " fmt, ##__VA_ARGS__); \
        last_debug_time = current_time; \
    } \
} while(0)
#else
#define DEBUG_FRAM(fmt, ...)
#endif

static bool i2c_initialized = false;

esp_err_t fram_i2c_init(void) {
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = FRAM_I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = FRAM_I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = FRAM_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_initialized = true;
    ESP_LOGI(TAG, "FRAM I2C initialized - SDA:%d, SCL:%d, Freq:%dHz, Addr:0x%02X", 
             FRAM_I2C_SDA_PIN, FRAM_I2C_SCL_PIN, FRAM_I2C_FREQ_HZ, FRAM_I2C_ADDRESS);

    // Test connectivity
    esp_err_t test_result = fram_test_connectivity();
    if (test_result != ESP_OK) {
        ESP_LOGW(TAG, "FRAM connectivity test failed: %s", esp_err_to_name(test_result));
    } else {
        ESP_LOGI(TAG, "FRAM connectivity test passed");
    }

    return ESP_OK;
}

esp_err_t fram_i2c_deinit(void) {
    if (!i2c_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_driver_delete(I2C_MASTER_NUM);
    if (ret == ESP_OK) {
        i2c_initialized = false;
        ESP_LOGI(TAG, "FRAM I2C deinitialized");
    }
    return ret;
}

esp_err_t fram_write_byte(uint16_t address, uint8_t data) {
    if (!i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FRAM_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (address >> 8) & 0xFF, ACK_CHECK_EN);  // Address high byte
    i2c_master_write_byte(cmd, address & 0xFF, ACK_CHECK_EN);         // Address low byte
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    DEBUG_FRAM("Write byte 0x%02X to address 0x%04X: %s", data, address, esp_err_to_name(ret));
    return ret;
}

esp_err_t fram_read_byte(uint16_t address, uint8_t *data) {
    if (!i2c_initialized || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    // Write address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FRAM_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (address >> 8) & 0xFF, ACK_CHECK_EN);  // Address high byte
    i2c_master_write_byte(cmd, address & 0xFF, ACK_CHECK_EN);         // Address low byte
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        DEBUG_FRAM("Failed to write address 0x%04X for read: %s", address, esp_err_to_name(ret));
        return ret;
    }

    // Read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FRAM_I2C_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    DEBUG_FRAM("Read byte 0x%02X from address 0x%04X: %s", *data, address, esp_err_to_name(ret));
    return ret;
}

esp_err_t fram_write_buffer(uint16_t address, const uint8_t *data, size_t length) {
    if (!i2c_initialized || !data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FRAM_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (address >> 8) & 0xFF, ACK_CHECK_EN);  // Address high byte
    i2c_master_write_byte(cmd, address & 0xFF, ACK_CHECK_EN);         // Address low byte
    
    for (size_t i = 0; i < length; i++) {
        i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
    }
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    DEBUG_FRAM("Write %d bytes to address 0x%04X: %s", length, address, esp_err_to_name(ret));
    return ret;
}

esp_err_t fram_read_buffer(uint16_t address, uint8_t *data, size_t length) {
    if (!i2c_initialized || !data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Write address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FRAM_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (address >> 8) & 0xFF, ACK_CHECK_EN);  // Address high byte
    i2c_master_write_byte(cmd, address & 0xFF, ACK_CHECK_EN);         // Address low byte
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        DEBUG_FRAM("Failed to write address 0x%04X for buffer read: %s", address, esp_err_to_name(ret));
        return ret;
    }

    // Read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FRAM_I2C_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    
    for (size_t i = 0; i < length; i++) {
        if (i == length - 1) {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_NACK);  // Last byte gets NACK
        } else {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);   // Other bytes get ACK
        }
    }
    
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    DEBUG_FRAM("Read %d bytes from address 0x%04X: %s", length, address, esp_err_to_name(ret));
    return ret;
}

esp_err_t fram_write_float(uint16_t address, float value) {
    uint8_t *bytes = (uint8_t *)&value;
    return fram_write_buffer(address, bytes, sizeof(float));
}

esp_err_t fram_read_float(uint16_t address, float *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    return fram_read_buffer(address, (uint8_t *)value, sizeof(float));
}

esp_err_t fram_write_uint32(uint16_t address, uint32_t value) {
    uint8_t *bytes = (uint8_t *)&value;
    return fram_write_buffer(address, bytes, sizeof(uint32_t));
}

esp_err_t fram_read_uint32(uint16_t address, uint32_t *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    return fram_read_buffer(address, (uint8_t *)value, sizeof(uint32_t));
}

esp_err_t fram_save_encoder_data(const fram_encoder_data_t *encoder_data) {
    if (!encoder_data) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Save current angle
    ret = fram_write_float(FRAM_ADDR_ENCODER_ANGLE, encoder_data->current_angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save encoder angle: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save calibration offset
    ret = fram_write_float(FRAM_ADDR_CALIBRATION_OFFSET, encoder_data->calibration_offset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration offset: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save calibration flag
    ret = fram_write_byte(FRAM_ADDR_CALIBRATED_FLAG, encoder_data->calibrated ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration flag: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save boot count
    ret = fram_write_uint32(FRAM_ADDR_BOOT_COUNT, encoder_data->boot_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save boot count: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save timestamp
    ret = fram_write_uint32(FRAM_ADDR_TIMESTAMP, encoder_data->last_save_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save timestamp: %s", esp_err_to_name(ret));
        return ret;
    }

    DEBUG_FRAM("Saved encoder data - Angle:%.2f°, Offset:%.2f°, Calibrated:%s, Boot:%lu, Time:%lu", 
               encoder_data->current_angle, encoder_data->calibration_offset, 
               encoder_data->calibrated ? "YES" : "NO", encoder_data->boot_count, encoder_data->last_save_time);

    return ESP_OK;
}

esp_err_t fram_load_encoder_data(fram_encoder_data_t *encoder_data) {
    if (!encoder_data) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t calibrated_byte;

    // Load current angle
    ret = fram_read_float(FRAM_ADDR_ENCODER_ANGLE, &encoder_data->current_angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load encoder angle: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load calibration offset
    ret = fram_read_float(FRAM_ADDR_CALIBRATION_OFFSET, &encoder_data->calibration_offset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load calibration offset: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load calibration flag
    ret = fram_read_byte(FRAM_ADDR_CALIBRATED_FLAG, &calibrated_byte);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load calibration flag: %s", esp_err_to_name(ret));
        return ret;
    }
    encoder_data->calibrated = (calibrated_byte != 0);

    // Load boot count
    ret = fram_read_uint32(FRAM_ADDR_BOOT_COUNT, &encoder_data->boot_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load boot count: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load timestamp
    ret = fram_read_uint32(FRAM_ADDR_TIMESTAMP, &encoder_data->last_save_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load timestamp: %s", esp_err_to_name(ret));
        return ret;
    }

    DEBUG_FRAM("Loaded encoder data - Angle:%.2f°, Offset:%.2f°, Calibrated:%s, Boot:%lu, Time:%lu", 
               encoder_data->current_angle, encoder_data->calibration_offset, 
               encoder_data->calibrated ? "YES" : "NO", encoder_data->boot_count, encoder_data->last_save_time);

    return ESP_OK;
}

esp_err_t fram_test_connectivity(void) {
    if (!i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint16_t test_address = 0x7FF0;  // Use address near end of FRAM
    const uint8_t test_pattern[] = {0xAA, 0x55, 0xCC, 0x33};
    uint8_t read_buffer[sizeof(test_pattern)];

    // Write test pattern
    esp_err_t ret = fram_write_buffer(test_address, test_pattern, sizeof(test_pattern));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FRAM connectivity test - write failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read back test pattern
    ret = fram_read_buffer(test_address, read_buffer, sizeof(read_buffer));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FRAM connectivity test - read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify data
    if (memcmp(test_pattern, read_buffer, sizeof(test_pattern)) != 0) {
        ESP_LOGE(TAG, "FRAM connectivity test - data mismatch");
        ESP_LOGE(TAG, "Written: 0x%02X 0x%02X 0x%02X 0x%02X", 
                 test_pattern[0], test_pattern[1], test_pattern[2], test_pattern[3]);
        ESP_LOGE(TAG, "Read:    0x%02X 0x%02X 0x%02X 0x%02X", 
                 read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3]);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "FRAM connectivity test passed");
    return ESP_OK;
}

esp_err_t fram_clear_encoder_data(void) {
    fram_encoder_data_t clear_data = {
        .current_angle = 0.0f,
        .calibration_offset = 0.0f,
        .calibrated = false,
        .boot_count = 0,
        .last_save_time = 0
    };

    esp_err_t ret = fram_save_encoder_data(&clear_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FRAM encoder data cleared");
    } else {
        ESP_LOGE(TAG, "Failed to clear FRAM encoder data: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t fram_save_remote_data(const fram_remote_data_t *remote_data) {
    if (!remote_data) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Save remote angle channel 2 (upper arm)
    ret = fram_write_float(FRAM_ADDR_REMOTE_ANGLE_CH2, remote_data->remote_angle_ch2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save remote angle CH2: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save remote angle channel 3 (elbow)
    ret = fram_write_float(FRAM_ADDR_REMOTE_ANGLE_CH3, remote_data->remote_angle_ch3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save remote angle CH3: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save remote timestamp
    ret = fram_write_uint32(FRAM_ADDR_REMOTE_TIMESTAMP, remote_data->remote_timestamp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save remote timestamp: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save remote data validity flags
    ret = fram_write_byte(FRAM_ADDR_REMOTE_VALID_CH2, remote_data->remote_ch2_valid ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save remote CH2 validity: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = fram_write_byte(FRAM_ADDR_REMOTE_VALID_CH3, remote_data->remote_ch3_valid ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save remote CH3 validity: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save communication statistics
    uint32_t comm_stats[4] = {
        remote_data->packets_received,
        remote_data->packets_sent,
        remote_data->last_communication_time,
        0  // Reserved for future use
    };
    
    ret = fram_write_buffer(FRAM_ADDR_COMM_STATS, (uint8_t*)comm_stats, sizeof(comm_stats));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save communication stats: %s", esp_err_to_name(ret));
        return ret;
    }

    DEBUG_FRAM("Saved remote data - CH2:%.2f°(%s), CH3:%.2f°(%s), RX:%lu, TX:%lu", 
               remote_data->remote_angle_ch2, remote_data->remote_ch2_valid ? "OK" : "BAD",
               remote_data->remote_angle_ch3, remote_data->remote_ch3_valid ? "OK" : "BAD",
               remote_data->packets_received, remote_data->packets_sent);

    return ESP_OK;
}

esp_err_t fram_load_remote_data(fram_remote_data_t *remote_data) {
    if (!remote_data) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t valid_byte;

    // Load remote angle channel 2 (upper arm)
    ret = fram_read_float(FRAM_ADDR_REMOTE_ANGLE_CH2, &remote_data->remote_angle_ch2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load remote angle CH2: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load remote angle channel 3 (elbow)
    ret = fram_read_float(FRAM_ADDR_REMOTE_ANGLE_CH3, &remote_data->remote_angle_ch3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load remote angle CH3: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load remote timestamp
    ret = fram_read_uint32(FRAM_ADDR_REMOTE_TIMESTAMP, &remote_data->remote_timestamp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load remote timestamp: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load remote data validity flags
    ret = fram_read_byte(FRAM_ADDR_REMOTE_VALID_CH2, &valid_byte);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load remote CH2 validity: %s", esp_err_to_name(ret));
        return ret;
    }
    remote_data->remote_ch2_valid = (valid_byte != 0);

    ret = fram_read_byte(FRAM_ADDR_REMOTE_VALID_CH3, &valid_byte);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load remote CH3 validity: %s", esp_err_to_name(ret));
        return ret;
    }
    remote_data->remote_ch3_valid = (valid_byte != 0);

    // Load communication statistics
    uint32_t comm_stats[4];
    ret = fram_read_buffer(FRAM_ADDR_COMM_STATS, (uint8_t*)comm_stats, sizeof(comm_stats));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load communication stats: %s", esp_err_to_name(ret));
        return ret;
    }

    remote_data->packets_received = comm_stats[0];
    remote_data->packets_sent = comm_stats[1];
    remote_data->last_communication_time = comm_stats[2];

    DEBUG_FRAM("Loaded remote data - CH2:%.2f°(%s), CH3:%.2f°(%s), RX:%lu, TX:%lu", 
               remote_data->remote_angle_ch2, remote_data->remote_ch2_valid ? "OK" : "BAD",
               remote_data->remote_angle_ch3, remote_data->remote_ch3_valid ? "OK" : "BAD",
               remote_data->packets_received, remote_data->packets_sent);

    return ESP_OK;
}

esp_err_t fram_update_remote_angle(uint8_t channel, float remote_angle, uint32_t timestamp) {
    esp_err_t ret;

    // Determine which addresses to use based on channel
    uint16_t angle_addr, valid_addr;
    if (channel == 2) {
        angle_addr = FRAM_ADDR_REMOTE_ANGLE_CH2;
        valid_addr = FRAM_ADDR_REMOTE_VALID_CH2;
    } else if (channel == 3) {
        angle_addr = FRAM_ADDR_REMOTE_ANGLE_CH3;
        valid_addr = FRAM_ADDR_REMOTE_VALID_CH3;
    } else {
        ESP_LOGE(TAG, "Invalid channel %d for remote angle update", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Quick update of just angle and timestamp (for frequent updates)
    ret = fram_write_float(angle_addr, remote_angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update remote angle CH%d: %s", channel, esp_err_to_name(ret));
        return ret;
    }

    ret = fram_write_uint32(FRAM_ADDR_REMOTE_TIMESTAMP, timestamp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update remote timestamp: %s", esp_err_to_name(ret));
        return ret;
    }

    // Mark data as valid for this channel
    ret = fram_write_byte(valid_addr, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update remote CH%d validity: %s", channel, esp_err_to_name(ret));
        return ret;
    }

    DEBUG_FRAM("Updated remote CH%d angle: %.2f° at time %lu", channel, remote_angle, timestamp);
    return ESP_OK;
}