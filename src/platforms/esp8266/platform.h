/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __PLATFORM_H
#define __PLATFORM_H

#undef PRIx32
#define PRIx32 "x"

#undef SCNx32
#define SCNx32 "x"

#define NO_USB_PLEASE

#include "esp_log.h"

void platform_buffer_flush(void);
void platform_set_baud(uint32_t baud);

#define SET_RUN_STATE(state)
#define SET_IDLE_STATE(state)
#define SET_ERROR_STATE(state)

#define ENABLE_DEBUG
//#define USE_GPIO2_UART

#define DEBUG(x, ...) do { ESP_LOGD("BMP", x, ##__VA_ARGS__); } while (0)
//#define DEBUG_GDBPACKET



#include "timing.h"
#include "driver/gpio.h"

#define LWIP_OPEN_SRC

#define TMS_SET_MODE() do { } while (0)

#ifdef USE_GPIO2_UART
#define TMS_PIN 1
#define TCK_PIN 0 //
#else
// no-connects on ESP-01: 12,13,14,15
#define TMS_PIN 0
#define TCK_PIN 2 //
// 2 is GPIO2, broken out
// 3 is RXD
#endif

#define TDI_PIN 13 // "
#define TDO_PIN 14 // "

#if defined(USE_GPIO2_UART) && (TMS_PIN==2 || TDI_PIN==2 || TDO_PIN==2 || TCK_PIN==2)
#error "GPIO2 is used for UART TX"
#endif

#define SWDIO_PIN TMS_PIN
#define SWCLK_PIN TCK_PIN

#define SWCLK_PORT 0
#define SWDIO_PORT 0

#define gpio_set_val(port, pin, value) 		gpio_set_level(pin, value);		

#define gpio_enable(pin, mode) gpio_set_direction(pin, mode);
#define gpio_set(port, pin) gpio_set_level(pin, 1);
#define gpio_clear(port, pin) gpio_set_level(pin, 0);
#define gpio_get(port, pin) gpio_get_level(pin)

#define GPIO_INPUT GPIO_MODE_INPUT
#define GPIO_OUTPUT GPIO_MODE_OUTPUT

#define SWDIO_MODE_FLOAT() 			\
		gpio_set_direction(SWDIO_PIN, GPIO_MODE_INPUT);	\


#define SWDIO_MODE_DRIVE() 				\
		gpio_set_direction(SWDIO_PIN, GPIO_MODE_OUTPUT);	\
	

#define PLATFORM_HAS_DEBUG // do we?
#define BOARD_IDENT "esp8266"
#endif
