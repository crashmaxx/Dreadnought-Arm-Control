/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "spi_bb.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "utils.h"
#include "board_config.h"  // For ENCODER_SPI_MOSI_ALWAYS_HIGH

// Software SPI

// Use modern ESP-IDF GPIO functions instead of direct register access
#define SET_PIN(pin) 			gpio_set_level(pin, 1)
#define CLEAR_PIN(pin) 			gpio_set_level(pin, 0)
#define WRITE_PIN(pin, level)	gpio_set_level(pin, level)
#define READ_PIN(pin)			gpio_get_level(pin)

void spi_bb_init(spi_bb_state *s) {
	gpio_reset_pin(s->miso_pin);
	gpio_reset_pin(s->sck_pin);
	gpio_reset_pin(s->nss_pin);

	gpio_set_direction(s->miso_pin, GPIO_MODE_INPUT);
	gpio_set_direction(s->sck_pin, GPIO_MODE_INPUT_OUTPUT);
	gpio_set_direction(s->nss_pin, GPIO_MODE_INPUT_OUTPUT);

	gpio_set_pull_mode(s->miso_pin, GPIO_PULLUP_ONLY);

	if (s->mosi_pin >= 0) {
		gpio_reset_pin(s->mosi_pin);
		gpio_set_direction(s->mosi_pin, GPIO_MODE_INPUT_OUTPUT);
		
#if ENCODER_SPI_MOSI_ALWAYS_HIGH
		// Set MOSI high and keep it there
		SET_PIN(s->mosi_pin);
#endif
	}
}

void spi_bb_deinit(spi_bb_state *s) {
	gpio_reset_pin(s->miso_pin);
	gpio_reset_pin(s->sck_pin);
	gpio_reset_pin(s->nss_pin);

	if (s->mosi_pin >= 0) {
		gpio_reset_pin(s->mosi_pin);
	}
}

uint8_t spi_bb_exchange_8(spi_bb_state *s, uint8_t x) {
	uint8_t rx;
	spi_bb_transfer_8(s, &rx, &x, 1);
	return rx;
}

void spi_bb_transfer_8(
		spi_bb_state *s, 
		uint8_t *in_buf, 
		const uint8_t *out_buf,
		int length
		) {
	for (int i = 0; i < length; i++) {
		uint8_t send = out_buf ? out_buf[i] : 0xFF;
		uint8_t receive = 0;

		for (int bit = 0; bit < 8; bit++) {
			if(s->mosi_pin >= 0) {
#if ENCODER_SPI_MOSI_ALWAYS_HIGH
				// MOSI is already set high, don't change it
#else
				WRITE_PIN(s->mosi_pin, send >> 7);
				send <<= 1;
#endif
			}

			SET_PIN(s->sck_pin);
			spi_bb_delay();

			int samples = 0;
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);

			CLEAR_PIN(s->sck_pin);

			// does 5 samples of each pad read, to minimize noise
			receive <<= 1;
			if (samples > 2) {
				receive |= 1;
			}

			spi_bb_delay();
		}

		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

void spi_bb_transfer_16(
		spi_bb_state *s, 
		uint16_t *in_buf, 
		const uint16_t *out_buf, 
		int length
		) {
	for (int i = 0; i < length; i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t receive = 0;

		for (int bit = 0; bit < 16; bit++) {
			if(s->mosi_pin >= 0) {
#if ENCODER_SPI_MOSI_ALWAYS_HIGH
				// MOSI is already set high, don't change it
#else
				WRITE_PIN(s->mosi_pin, send >> 15);  // Fixed: use bit 15 for 16-bit data
				send <<= 1;
#endif
			}

			SET_PIN(s->sck_pin);
			spi_bb_delay_short();

			int samples = 0;
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);
			__NOP();
			samples += READ_PIN(s->miso_pin);

			receive <<= 1;
			if (samples > 2) {
				receive |= 1;
			}

			CLEAR_PIN(s->sck_pin);
			spi_bb_delay_short();
		}

		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

void spi_bb_begin(spi_bb_state *s) {
	spi_bb_delay();
	CLEAR_PIN(s->nss_pin);
	spi_bb_delay();
}

void spi_bb_end(spi_bb_state *s) {
	spi_bb_delay();
	SET_PIN(s->nss_pin);
	spi_bb_delay();
}

void spi_bb_delay(void) {
	// ~1500 ns long
	for (volatile int i = 0; i < 6; i++) {
		__NOP();
	}
}

void spi_bb_delay_short(void) {
	__NOP(); __NOP();
	__NOP(); __NOP();
}

bool spi_bb_check_parity(uint16_t x) {
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return (~x) & 1;
}
