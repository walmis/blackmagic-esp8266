# -*-Makefile-*-
RTOS_PATH=platforms/esp8266/esp-open-rtos
ESPPORT = /dev/ttyUSB0
ESPBAUD = 921600
FLASH_SPEED=80
PROGRAM=blackmagic
LIBESPHTTPD_HTML_DIR=platforms/esp8266/html

CFLAGS += -DLWIP -DNO_LIBOPENCM3 -DFREERTOS
CFLAGS += -Wno-unused-parameter
CFLAGS += -Wno-sign-compare
CFLAGS += -Wno-old-style-declaration
CFLAGS += -Wno-type-limits
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-empty-body
CFLAGS += -Wno-error
CFLAGS += -DconfigENABLE_BACKWARD_COMPATIBILITY=1
CFLAGS += -DconfigUSE_COUNTING_SEMAPHORES=1
#CFLAGS += -DLWIP_HTTPD_CUSTOM_FILES=1



# Requires hack from https://groups.google.com/forum/#!searchin/esp-open-rtos/hostname/esp-open-rtos/bptnLZDxaEY/bBsTYf02AAAJ
CFLAGS +=  -DLWIP_NETIF_HOSTNAME=1 -DESP8266_HOSTNAME=\"BLACKMAGIC01\"

# we need to supercede these, or borrow specific versions from other platforms
SRC := $(filter-out main.c, $(SRC))

SRC += timing.c jtag_scan.c jtagtap.c swdptap.c
SRC += remote.c

# yep, manual VPATH search, thanks to the wondrous esp-open-rtos build system
PROGRAM_SRC_FILES = $(foreach sdir, . $(VPATH), $(foreach f, $(SRC), $(wildcard $(sdir)/$(f))))

# use the Berkeley sockets interface
PROGRAM_SRC_FILES += platforms/esp8266/gdb_if.c platforms/esp8266/uart.c platforms/esp8266/dhcpserver/dhcpserver.c platforms/esp8266/http.c

# ... and the bitbang JTAG interface
#PROGRAM_SRC_FILES += platforms/stm32/jtagtap.c

BUILD_DIR = platforms/esp8266/build/
FIRMWARE_DIR = platforms/esp8266/firmware/

#EXTRA_COMPONENTS=extras/dhcpserver

EXTRA_COMPONENTS=extras/rboot-ota extras/libesphttpd

all: include/version.h

include $(RTOS_PATH)/common.mk

host_clean:
	$(Q) rm -rf $(BUILD_DIR) $(FIRMWARE_DIR)

debug_shell: flash
	miniterm.py $(ESPPORT) 460800
