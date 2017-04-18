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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>

jmp_buf fatal_error_jmpbuf;

void platform_init(void)
{
	/* Check the USER button*/
	rcc_periph_clock_enable(RCC_GPIOA);
	if(gpio_get(GPIOA, GPIO0)) {
		platform_request_boot();
		scb_reset_core();
	}

	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_48MHZ]);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_CRC);

	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOC_OSPEEDR &=~0xF30;
	GPIOC_OSPEEDR |= 0xA20;
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			TMS_PIN | TCK_PIN | TDI_PIN);

	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT,
			GPIO_PUPD_NONE,
			TDO_PIN);

	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			LED_UART | LED_IDLE_RUN | LED_ERROR | LED_BOOTLOADER);

	platform_timing_init();
	usbuart_init();
	cdcacm_init();
}

void platform_srst_set_val(bool assert) { (void)assert; }
bool platform_srst_get_val(void) { return false; }

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}

void platform_request_boot(void)
{
	/* Disconnect USB cable */
	if (RCC_AHB2ENR & RCC_AHB2ENR_OTGFSEN) {
		usbd_disconnect(usbdev, 1);
		nvic_disable_irq(USB_IRQ);
		rcc_periph_clock_disable(RCC_OTGFS);
	}

	/* Assert blue LED as indicator we are in the bootloader */
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
		GPIO_PUPD_NONE, LED_BOOTLOADER);
	gpio_set(LED_PORT, LED_BOOTLOADER);
	/* Disable used ports beside PORTD.*/
	rcc_periph_clock_disable(RCC_GPIOA);
	rcc_periph_clock_disable(RCC_GPIOC);
	rcc_periph_clock_disable(USBUSART_CLK);
	/* Reset Systick.*/
	systick_interrupt_disable();
	STK_CSR = 0;
	STK_RVR = 0;
	STK_CVR = 0;
	/*Disable Interrupts.*/
	cm_disable_interrupts();
	/* Switch back to HSI.*/
	while (!(RCC_CR & RCC_CR_HSIRDY))
		RCC_CR |= RCC_CR_HSION;
	while (RCC_CFGR & (RCC_CFGR_SWS_HSE | RCC_CFGR_SWS_PLL))
		RCC_CFGR &= ~(RCC_CFGR_SWS_HSE | RCC_CFGR_SWS_PLL);

	/* Map System flash at 0.*/
	rcc_periph_clock_enable(RCC_SYSCFG);
	SYSCFG_MEMRM &= ~3;
	SYSCFG_MEMRM |=  1;
}
