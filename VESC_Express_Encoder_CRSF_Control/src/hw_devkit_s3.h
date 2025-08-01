/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_DEVKIT_S3_H_
#define MAIN_HWCONF_DEVKIT_S3_H_

#define HW_NAME						"Devkit S3"
#define HW_UART_COMM

#define HW_INIT_HOOK()				hw_init()

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
// #define UART_TX						43
// #define UART_RX						44

// CAN Bus pins (using available GPIO pins on ESP32-S3)
#define CAN_TX_GPIO_NUM				13
#define CAN_RX_GPIO_NUM				12


// CRSF pins (using different pins for ESP32-S3)
#define CRSF_TX_PIN					2
#define CRSF_RX_PIN					1

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_DEVKIT_S3_H_ */
