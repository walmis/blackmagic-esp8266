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

#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "espressif/esp_wifi.h"
#include "ssid_config.h"

#include "dhcpserver/dhcpserver.h"
#include "uart.h"

#   include "lwip/err.h"
#   include "lwip/sockets.h"
#   include "lwip/sys.h"
#   include "lwip/netdb.h"
#   include "lwip/dns.h"
#include "lwip/api.h"

#define ACCESS_POINT_MODE
#define AP_SSID	 "blackmagic"
#define AP_PSK	 "helloworld"

struct netconn* uart_client_sock;
struct netconn* uart_serv_sock;
int uart_sockerr;

#define PASTER(x) IOMUX_GPIO ## x ##_FUNC_GPIO
#define EVALUATOR(x)  PASTER(x)
#define _IOMUX(fun) EVALUATOR(fun)


void platform_init() {

#if TMS_PIN==3 || TCK_PIN==3 || TDI_PIN==3 || TDO_PIN==3
  gpio_set_iomux_function(3, IOMUX_GPIO3_FUNC_GPIO);
#endif

#ifdef USE_GPIO2_UART
  gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_UART1_TXD);
#else
  gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_GPIO);
#endif

  gpio_set_iomux_function(TMS_PIN, _IOMUX(TMS_PIN));
  gpio_set_iomux_function(TCK_PIN, _IOMUX(TCK_PIN));

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

const char *
platform_target_voltage(void) {
  return "not supported";
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

SemaphoreHandle_t sem_poll;
int client_sock_pending_bytes;

void netconn_cb(struct netconn * nc, enum netconn_evt evt, u16_t len) {
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
    xSemaphoreGive(sem_poll);
    uart_sockerr = 1;
    client_sock_pending_bytes = 0;
  }
}

void uart_rx_task(void *parameters) {
  static uint8_t buf[256];
  int bufpos = 0;
  struct netconn* nc;

  sem_poll = xSemaphoreCreateCounting(1024, 0);

  uart_serv_sock = netconn_new(NETCONN_TCP);

  netconn_bind(uart_serv_sock, IP_ADDR_ANY, 23);
  netconn_listen(uart_serv_sock);

  int opt;
  int ret;

  DEBUG("Listening on :23\n");

  uart_add_rx_poll(sem_poll);

  while (1) {
    int c;
    if (uart_sockerr) {
      uart_client_sock->callback = NULL;
      netconn_delete(uart_client_sock);
      uart_client_sock = 0;
      DEBUG("Finish telnet connection\n");
    }

    if (!uart_client_sock) {
      netconn_accept(uart_serv_sock, &uart_client_sock);
      uart_sockerr = 0;
      client_sock_pending_bytes = 0;

      if (uart_client_sock) {
        DEBUG("New telnet connection ovr:%d\n", uart0_overruns());
        uart_client_sock->callback = netconn_cb;
      }
    }

    if (xSemaphoreTake(sem_poll, 1000) != pdFALSE) {

      int rxcount = uart0_rxcount();
      while (rxcount--) {
        if (bufpos == sizeof(buf))
          break;
        buf[bufpos++] = uart0_getchar();
      }
      //DEBUG("uart rx:%d\n", bufpos);
      if (uart_client_sock) {
        if (bufpos) {
          netconn_write(uart_client_sock, buf, bufpos,
              uart0_rxcount() > 0 ? NETCONN_MORE : 0);
          bufpos = 0;
        }

        if (client_sock_pending_bytes) {
          struct netbuf* nb = 0;
          netconn_recv(uart_client_sock, &nb);

          char* data = 0;
          u16_t len = 0;
          netbuf_data(nb, (void*)&data, &len);

          //fwrite(data, len, 1, stdout);
          for(int i = 0; i < len; i++) {
            uart_putc(1, data[i]);
            uart_putc(0, data[i]);
          }

          netbuf_delete(nb);
        }
      }

    }

  }
}

bool cmd_setbaud(target *t, int argc, const char **argv) {

  return 1;
}

void user_init(void) {
  uart_set_baud(0, 460800);
  uart_set_baud(1, 460800);

  printf("SDK version:%s\n", sdk_system_get_sdk_version());

  sdk_system_update_cpu_freq(160);


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
      .ssid = AP_SSID,
      .ssid_hidden = 0,
      .channel = 3,
      .ssid_len = strlen(AP_SSID),
      .authmode = AUTH_WPA_WPA2_PSK,
      .password = AP_PSK,
      .max_connection = 3,
      .beacon_interval = 100,
  };

  sdk_wifi_softap_set_config(&ap_config);

  ip_addr_t first_client_ip;
  IP4_ADDR(&first_client_ip, 192, 168, 4, 2);
  dhcpserver_start(&first_client_ip, 4);

  uart0_rx_init();

#endif
  xTaskCreate(&main_task, "main", 4096, NULL, 2, NULL);
  xTaskCreate(&uart_rx_task, "uart rx", 512, NULL, 2, NULL);
}
