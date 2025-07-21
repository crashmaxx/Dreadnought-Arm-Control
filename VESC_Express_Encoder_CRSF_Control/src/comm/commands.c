/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

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

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "commands.h"
#include "datatypes.h"
#include "comm_can.h"
#include "utils.h"
#include "packet.h"
#include "buffer.h"
#include "main.h"
#include "crc.h"

// Project-specific definitions
#define FW_VERSION_MAJOR        1
#define FW_VERSION_MINOR        0
#define FW_TEST_VERSION_NUMBER  0
#define HW_NAME                 "ESP32-C3 VESC Express"
#define FW_NAME                 "VESC Express Encoder CRSF"

// Settings
#define PRINT_BUFFER_SIZE	400

// Private variables
static SemaphoreHandle_t print_mutex;
static bool init_done = false;

static const esp_partition_t *update_partition = NULL;
static esp_ota_handle_t update_handle = 0;

// Function pointers
static send_func_t send_func = 0;
static send_func_t send_func_can_fwd = 0;
static send_func_t send_func_blocking = 0;

// Blocking thread
static SemaphoreHandle_t block_sem;
static uint8_t blocking_thread_cmd_buffer[PACKET_MAX_PL_LEN];
static volatile unsigned int blocking_thread_cmd_len = 0;
static volatile bool is_blocking = false;

// Private functions
static void send_func_dummy(unsigned char *data, unsigned int len) {
	(void)data; (void)len;
}

static void block_task(void *arg) {
	for (;;) {
		is_blocking = false;

		xSemaphoreTake(block_sem, portMAX_DELAY);

		uint8_t *data = blocking_thread_cmd_buffer;
		unsigned int len = blocking_thread_cmd_len;

		COMM_PACKET_ID packet_id;
		static uint8_t send_buffer[512];

		packet_id = data[0];
		data++;
		len--;

		switch (packet_id) {
		case COMM_PING_CAN: {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_PING_CAN;

			for (uint8_t i = 0; i < 255; i++) {
				HW_TYPE hw_type;
				if (comm_can_ping(i, &hw_type)) {
					send_buffer[ind++] = i;
				}
			}

			if (send_func_blocking) {
				send_func_blocking(send_buffer, ind);
			}
		} break;

		default:
			break;
		}
	}

	vTaskDelete(NULL);
}

void commands_init(void) {
	print_mutex = xSemaphoreCreateMutex();
	block_sem = xSemaphoreCreateBinary();
	xTaskCreatePinnedToCore(block_task, "comm_block", 2500, NULL, 7, NULL, tskNO_AFFINITY);
	init_done = true;
}

void commands_process_packet(unsigned char *data, unsigned int len,
		send_func_t reply_func) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	send_func = reply_func;

	if (!send_func_can_fwd) {
		send_func_can_fwd = reply_func;
	}

	// Avoid calling invalid function pointer if it is null.
	if (!reply_func) {
		reply_func = send_func_dummy;
	}

	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[65];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		// Read MAC address from eFuse
		size_t size_bits = esp_efuse_get_field_size(ESP_EFUSE_MAC_FACTORY);
	    esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, send_buffer + ind, size_bits);
	    ind += 6;
		memset(send_buffer + ind, 0, 6); // UUID (not used)
		ind += 6;

		send_buffer[ind++] = 0; // Paired (not used)
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		send_buffer[ind++] = HW_TYPE_CUSTOM_MODULE;
		send_buffer[ind++] = 1; // One custom config

		send_buffer[ind++] = 0; // No phase filters
		send_buffer[ind++] = 0; // No HW QML
		send_buffer[ind++] = 0; // No QML flags
		send_buffer[ind++] = 0; // No NRF flags

		strcpy((char*)(send_buffer + ind), FW_NAME);
		ind += strlen(FW_NAME) + 1;

		// Hardware CRC (calculate simple checksum)
		uint32_t hw_crc = 0;
		for (int i = 0; i < ind; i++) {
			hw_crc += send_buffer[i];
		}
		buffer_append_uint32(send_buffer, hw_crc, &ind);

		reply_func(send_buffer, ind);
	} break;

	case COMM_JUMP_TO_BOOTLOADER:
		// Simple restart for bootloader mode
		esp_restart();
		break;

	case COMM_ERASE_NEW_APP: {
		int32_t ind = 0;

		if (update_handle != 0) {
			esp_ota_abort(update_handle);
			update_handle = 0;
		}

		update_partition = esp_ota_get_next_update_partition(NULL);
		bool ok = false;
		if (update_partition != NULL) {
			esp_err_t res = esp_ota_begin(update_partition, buffer_get_uint32(data, &ind), &update_handle);
			ok = res == ESP_OK;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_NEW_APP;
		send_buffer[ind++] = ok;
		reply_func(send_buffer, ind);
	} break;

	case COMM_WRITE_NEW_APP_DATA: {
		int32_t ind = 0;
		uint32_t new_app_offset = buffer_get_uint32(data, &ind);

		bool ok = false;
		if (update_handle != 0) {
			esp_err_t res = esp_ota_write(update_handle, data + ind, len - ind);
			ok = res == ESP_OK;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
		send_buffer[ind++] = ok;
		buffer_append_uint32(send_buffer, new_app_offset, &ind);
		reply_func(send_buffer, ind);
	} break;

	case COMM_REBOOT: {
		esp_restart();
	} break;

	case COMM_FORWARD_CAN:
		send_func_can_fwd = reply_func;
		comm_can_send_buffer(data[0], data + 1, len - 1, 0);
		break;

	case COMM_CAN_FWD_FRAME: {
		int32_t ind = 0;
		uint32_t id = buffer_get_uint32(data, &ind);
		bool is_ext = data[ind++];

		if (is_ext) {
			comm_can_transmit_eid(id, data + ind, len - ind);
		} else {
			comm_can_transmit_sid(id, data + ind, len - ind);
		}
	} break;

	// Blocking commands
	case COMM_PING_CAN:
		if (!is_blocking) {
			memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
			blocking_thread_cmd_len = len + 1;
			is_blocking = true;
			send_func_blocking = reply_func;
			xSemaphoreGive(block_sem);
		}
		break;

	default:
		// Unknown command - ignore silently
		break;
	}
}

/**
 * Send a packet using the last can fwd function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet_can_last(unsigned char *data, unsigned int len) {
	if (send_func_can_fwd) {
		send_func_can_fwd(data, len);
	}
}

void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

send_func_t commands_get_send_func(void) {
	return send_func;
}

void commands_set_send_func(send_func_t func) {
	send_func = func;
}

int commands_printf(const char* format, ...) {
	if (!init_done) {
		return 0;
	}

	xSemaphoreTake(print_mutex, portMAX_DELAY);

	va_list arg;
	va_start (arg, format);
	int len;

	char *print_buffer = malloc(PRINT_BUFFER_SIZE);

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, (PRINT_BUFFER_SIZE - 1), format, arg);
	va_end (arg);

	int len_to_print = (len < (PRINT_BUFFER_SIZE - 1)) ? len + 1 : PRINT_BUFFER_SIZE;

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, len_to_print);
	}

	free(print_buffer);
	xSemaphoreGive(print_mutex);

	return len_to_print - 1;
}

void commands_send_app_data(unsigned char *data, unsigned int len) {
	int32_t index = 0;
	uint8_t *send_buffer = malloc(len + 10);
	if (send_buffer) {
		send_buffer[index++] = COMM_CUSTOM_APP_DATA;
		memcpy(send_buffer + index, data, len);
		index += len;
		commands_send_packet(send_buffer, index);
		free(send_buffer);
	}
}
