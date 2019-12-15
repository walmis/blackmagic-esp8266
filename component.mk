#
# Component Makefile
#

COMPONENT_ADD_INCLUDEDIRS := src/platforms/esp8266 src/include src/platforms/common

COMPONENT_SRCDIRS := src/target src src/platforms/esp8266 src/platforms/common
CFLAGS += -DNO_LIBOPENCM3=1 -Wno-error=char-subscripts -Wno-char-subscripts -DPROBE_HOST=esp8266
COMPONENT_OBJEXCLUDE := src/platforms/common/cdcacm.o

$(COMPONENT_PATH)/src/include/version.h: 
	$(MAKE) -C $(COMPONENT_PATH)/src include/version.h
    
build: $(COMPONENT_PATH)/src/include/version.h
    

