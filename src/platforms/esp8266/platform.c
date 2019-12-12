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
#include "general.h"
#include "gdb_if.h"
#include "version.h"

#include "gdb_packet.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"

#include <assert.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <esplibs/libphy.h>

#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "espressif/esp_wifi.h"
#include "ssid_config.h"

#include "dhcpserver/dhcpserver.h"
#include "uart.h"
#include <sysparam.h>

#   include "lwip/err.h"
#   include "lwip/sockets.h"
#   include "lwip/sys.h"
#   include "lwip/netdb.h"
#   include "lwip/dns.h"
#include "lwip/api.h"

#define ACCESS_POINT_MODE
#define AP_SSID	 "blackmagic"
#define AP_PSK	 "helloworld"

static struct netconn *uart_client_sock;
static struct netconn *uart_serv_sock;
static int uart_sockerr;
static SemaphoreHandle_t sem_poll;
static int client_sock_pending_bytes;
static int clients_pending;

#define __IOMUX(x) IOMUX_GPIO ## x ##_FUNC_GPIO
#define _IOMUX(x)  __IOMUX(x)

void platform_init() {

#ifdef USE_GPIO2_UART
  gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_UART1_TXD);
#else
  gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_GPIO);
#endif

  gpio_set_iomux_function(SWDIO_PIN, _IOMUX(SWDIO_PIN));
  gpio_set_iomux_function(SWCLK_PIN, _IOMUX(SWCLK_PIN));

  gpio_clear(_, SWCLK_PIN);
  gpio_clear(_, SWDIO_PIN);

  gpio_enable(SWCLK_PIN, GPIO_OUTPUT);
  gpio_enable(SWDIO_PIN, GPIO_OUTPUT);

  assert(gdb_if_init() == 0);
}

void platform_buffer_flush(void) {
  ;
}

void platform_srst_set_val(bool assert) {
  (void) assert;
}

bool platform_srst_get_val(void) {
  return false;
}

const char*
platform_target_voltage(void) {
  static char voltage[16];
  int vdd = sdk_readvdd33();
  sprintf(voltage, "%dmV", vdd);
  return voltage;
}

uint32_t platform_time_ms(void) {
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

#define vTaskDelayMs(ms)	vTaskDelay((ms)/portTICK_PERIOD_MS)

void platform_delay(uint32_t ms) {
  vTaskDelayMs(ms);
}

int platform_hwversion(void) {
  return 0;
}

/* This is a transplanted main() from main.c */
void main_task(void *parameters) {
  (void) parameters;

  platform_init();

  while (true) {

    volatile struct exception e;
    TRY_CATCH(e, EXCEPTION_ALL) {
      gdb_main();
    }
    if (e.type) {
      gdb_putpacketz("EFF");
      target_list_free();
      morse("TARGET LOST.", 1);
    }
  }

  /* Should never get here */
}



void netconn_serv_cb(struct netconn *nc, enum netconn_evt evt, u16_t len) {
  DEBUG("evt %d len %d\n", evt, len);

  if (evt == NETCONN_EVT_RCVPLUS)
  {
    clients_pending++;
    xSemaphoreGive(sem_poll);
  }
  if (evt == NETCONN_EVT_RCVMINUS)
  {
    clients_pending--;
  }
}

void netconn_cb(struct netconn *nc, enum netconn_evt evt, u16_t len) {
  //printf("evt %d len %d\n", evt, len);

  if (evt == NETCONN_EVT_RCVPLUS) {
    client_sock_pending_bytes += len;
    if (len == 0) {
      uart_sockerr = 1;
      client_sock_pending_bytes = 0;
    }
    xSemaphoreGive(sem_poll);

  } else if (evt == NETCONN_EVT_RCVMINUS) {
    client_sock_pending_bytes -= len;
  }

  else if (evt == NETCONN_EVT_ERROR) {
    DEBUG("NETCONN_EVT_ERROR\n");

    uart_sockerr = 1;
    client_sock_pending_bytes = 0;
    xSemaphoreGive(sem_poll);
  }
}

void uart_rx_task(void *parameters) {
  static uint8_t buf[256];
  int bufpos = 0;
  struct netconn *nc;

  sem_poll = xSemaphoreCreateCounting(1024, 0);

  uart_serv_sock = netconn_new(NETCONN_TCP);
  uart_serv_sock->callback = netconn_serv_cb;

  netconn_bind(uart_serv_sock, IP_ADDR_ANY, 23);
  netconn_listen(uart_serv_sock);

  int opt;
  int ret;

  DEBUG("Listening on :23\n");

  uart_add_rx_poll(sem_poll);

  while (1) {
    int c;
    if (xSemaphoreTake(sem_poll, 1000) != pdFALSE) {

      if (uart_sockerr && uart_client_sock) {
        uart_client_sock->callback = NULL;
        netconn_delete(uart_client_sock);
        uart_client_sock = 0;
        uart_sockerr = 0;
        DEBUG("Finish telnet connection\n");
      }

      if (!uart_client_sock && clients_pending) {
        netconn_accept(uart_serv_sock, &uart_client_sock);
        uart_sockerr = 0;
        client_sock_pending_bytes = 0;

        if (uart_client_sock) {
          DEBUG("New telnet connection ovr:%d\n", uart0_overruns());
          uart_client_sock->callback = netconn_cb;
        }
      }


      int rxcount = uart0_rxcount();
      while (rxcount--) {
        if (bufpos == sizeof(buf))
          break;
        buf[bufpos++] = uart0_getchar();
      }
      //DEBUG("uart rx:%d\n", bufpos);
      if(bufpos) {
        http_term_broadcast_data(buf, bufpos);

        if (uart_client_sock) {
          if (bufpos) {
            netconn_write(uart_client_sock, buf, bufpos, NETCONN_COPY |
                (uart0_rxcount() > 0 ? NETCONN_MORE : 0));
          }
        } // if (uart_client_sock)
        bufpos = 0;
      } //if(bufpos)

      if (uart_client_sock && client_sock_pending_bytes) {
        struct netbuf *nb = 0;
        netconn_recv(uart_client_sock, &nb);

        char *data = 0;
        u16_t len = 0;
        netbuf_data(nb, (void*) &data, &len);

        //fwrite(data, len, 1, stdout);
        for (int i = 0; i < len; i++) {
#ifdef USE_GPIO2_UART
          uart_putc(1, data[i]);
#else
          uart_putc(0, data[i]);
#endif
        }

        netbuf_delete(nb);
      }

    }

  }
}

bool cmd_setbaud(target *t, int argc, const char **argv) {

  if (argc == 1) {
    gdb_outf("Current baud: %d\n", uart_get_baud(0));
  }
  if (argc == 2) {
    int baud = atoi(argv[1]);
    gdb_outf("Setting baud: %d\n", baud);

    uart_set_baud(0, baud);
    uart_set_baud(1, baud);

    if (sysparam_set_int32("uartbaud", baud) != SYSPARAM_OK) {
      gdb_outf("Failed to save baudrate to flash\n");
    }
  }

  return 1;
}

void user_init(void) {
  int32_t baud = 460800;
  sysparam_get_int32("uartbaud", &baud);

  uart_set_baud(0, baud);
  uart_set_baud(1, baud);

  printf("SDK version:%s\n", sdk_system_get_sdk_version());

  sdk_system_update_cpu_freq(160);
  sdk_wifi_set_phy_mode(PHY_MODE_11N);

  uint32_t base_addr, num_sectors;
  sysparam_status_t status = sysparam_get_info(&base_addr, &num_sectors);

  if (status == SYSPARAM_OK) {
    DEBUG("[current sysparam region is at 0x%08x (%d sectors)]\n", base_addr,
        num_sectors);
  } else {
    DEBUG(
        "[NOTE: No current sysparam region (initialization problem during boot?)]\n");
    // Default to the same place/size as the normal system initialization
    // stuff, so if the user uses this utility to reformat it, it will put
    // it somewhere the system will find it later
    num_sectors = DEFAULT_SYSPARAM_SECTORS;
    base_addr = sdk_flashchip.chip_size
        - (5 + num_sectors) * sdk_flashchip.sector_size;

    if (sysparam_create_area(base_addr, num_sectors, 1) == SYSPARAM_OK) {
      DEBUG("Created sysparam area\n");
      status = sysparam_init(base_addr, 0);
      if (status != SYSPARAM_OK) {
        DEBUG("Failed to init sysparam area\n");
      }
    }

  }

#ifndef ACCESS_POINT_MODE
  struct sdk_station_config config =
  {
    .ssid = WIFI_SSID,
    .password = WIFI_PASS,
  };

  sdk_wifi_set_opmode(STATION_MODE);
  sdk_wifi_station_set_config(&config);
#else

  /* required to call wifi_set_opmode before station_set_config */
  sdk_wifi_set_opmode(SOFTAP_MODE);

  struct ip_info ap_ip;
  IP4_ADDR(&ap_ip.ip, 192, 168, 4, 1);
  IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
  IP4_ADDR(&ap_ip.netmask, 255, 255, 0, 0);
  sdk_wifi_set_ip_info(1, &ap_ip);

  struct sdk_softap_config ap_config = {
      .ssid_hidden = 0,
      .channel = 3,
      .authmode = AUTH_WPA_WPA2_PSK,
      .password = AP_PSK,
      .max_connection = 3,
      .beacon_interval = 100,
  };

  ap_config.ssid_len = sprintf((char*)ap_config.ssid, AP_SSID "_%X", sdk_system_get_chip_id());

  sdk_wifi_softap_set_config(&ap_config);

  ip_addr_t first_client_ip;
  IP4_ADDR(&first_client_ip, 192, 168, 4, 2);
  dhcpserver_start(&first_client_ip, 4);

  uart0_rx_init();

  httpd_start();

#endif
  xTaskCreate(&main_task, "main", 2560, NULL, 2, NULL);
  xTaskCreate(&uart_rx_task, "uart rx", 512, NULL, 2, NULL);
}

#ifndef ENABLE_DEBUG
__attribute((used))
int ets_printf(const char *__restrict c, ...) { return 0; }

__attribute((used))
int printf(const char *__restrict c, ...) { return 0; }
#endif
