#include <string.h>
#include <stdio.h>
#define  ICACHE_FLASH_ATTR
typedef uint8_t uint8;

#include <espressif/esp_common.h>
#include <etstimer.h>
#include <libesphttpd/httpd.h>
#include <libesphttpd/httpdespfs.h>
#include <libesphttpd/cgiwifi.h>
#include <libesphttpd/cgiflash.h>
#include <libesphttpd/auth.h>
#include <libesphttpd/espfs.h>
#include <libesphttpd/captdns.h>
#include <libesphttpd/webpages-espfs.h>
#include <libesphttpd/cgiwebsocket.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <esp/uart.h>
#include "platform.h"

extern char _binary_platforms_esp8266_build_web_espfs_bin_start[] ;

static void on_term_recv(Websock *ws, char *data, int len, int flags) {
  for(int i = 0; i < len; i++) {
#ifdef USE_GPIO2_UART
            uart_putc(1, data[i]);
#else
            uart_putc(0, data[i]);
#endif
  }

}

static void on_term_connect(Websock *ws) {
  ws->recvCb = on_term_recv;

  //cgiWebsocketSend(ws, "Hi, Websocket!", 14, WEBSOCK_FLAG_NONE);
}

HttpdBuiltInUrl builtInUrls[]={
//  {"*", cgiRedirectApClientToHostname, "esp8266.nonet"},
  {"/", cgiRedirect, "/index.html"},
//  {"/led.tpl", cgiEspFsTemplate, tplLed},
//  {"/index.tpl", cgiEspFsTemplate, tplCounter},
//  {"/led.cgi", cgiLed, NULL},
//#ifndef ESP32
//  {"/flash/", cgiRedirect, "/flash/index.html"},
//  {"/flash/next", cgiGetFirmwareNext, &uploadParams},
//  {"/flash/upload", cgiUploadFirmware, &uploadParams},
//  {"/flash/reboot", cgiRebootFirmware, NULL},
//#endif
//  //Routines to make the /wifi URL and everything beneath it work.
//
////Enable the line below to protect the WiFi configuration with an username/password combo.
////  {"/wifi/*", authBasic, myPassFn},
//
//  {"/wifi", cgiRedirect, "/wifi/wifi.tpl"},
//  {"/wifi/", cgiRedirect, "/wifi/wifi.tpl"},
//  {"/wifi/wifiscan.cgi", cgiWiFiScan, NULL},
//  {"/wifi/wifi.tpl", cgiEspFsTemplate, tplWlan},
//  {"/wifi/connect.cgi", cgiWiFiConnect, NULL},
//  {"/wifi/connstatus.cgi", cgiWiFiConnStatus, NULL},
//  {"/wifi/setmode.cgi", cgiWiFiSetMode, NULL},
//
  {"/terminal", cgiWebsocket, on_term_connect},
//  {"/websocket/echo.cgi", cgiWebsocket, myEchoWebsocketConnect},
//
//  {"/test", cgiRedirect, "/test/index.html"},
//  {"/test/", cgiRedirect, "/test/index.html"},
//  {"/test/test.cgi", cgiTestbed, NULL},

  {"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
  {NULL, NULL, NULL}
};

void http_term_broadcast_data(uint8_t* data, size_t len) {
  cgiWebsockBroadcast("/terminal", data, len, WEBSOCK_FLAG_NONE);
}



void httpd_start() {
  espFsInit((void*)(_binary_platforms_esp8266_build_web_espfs_bin_start));

  httpdInit(builtInUrls, 80);


}
