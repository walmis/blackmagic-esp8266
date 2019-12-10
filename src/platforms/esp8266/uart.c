/*
 * uart.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: walmis
 */

#include <FreeRTOS.h>
#include <esp8266.h>
#include <queue.h>
#include "uart.h"


#define UART0 0
#define UART0_RX_SIZE  (128)


static xQueueHandle rxqueue;
static uint32_t overruns;

static inline uint8_t uart_get() {
	return UART(UART0).FIFO & (UART_FIFO_DATA_M << UART_FIFO_DATA_S);
}
//returns character count in uart hw fifo
static inline uint32_t uart0_num_char(void)
{
    uint32_t count;
    count = UART(UART0).STATUS & (UART_STATUS_RXFIFO_COUNT_M << UART_STATUS_RXFIFO_COUNT_S);
    return count;
}

static bool uart0_rxqueue_push(uint8_t c) {
    long int xHigherPriorityTaskWoken;
    bool res = false;

	if(xQueueSendToBackFromISR(rxqueue, (void*)&c, &xHigherPriorityTaskWoken) == errQUEUE_FULL) {
		overruns++;
	} else {
		res = true;
	}

    //xSemaphoreGiveFromISR(uart0_sem, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD();
    }

    return res;
}

IRAM void uart0_rx_handler(void* arg)
{
    // TODO: Handle UART1, see reg 0x3ff20020, bit2, bit0 represents uart1 and uart0 respectively
    if (!UART(UART0).INT_STATUS & UART_INT_STATUS_RXFIFO_FULL) {
        return;
    }
//    printf(" [%08x (%d)]\n", READ_PERI_REG(UART_INT_ST(UART0)), READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S));
    if (UART(UART0).INT_STATUS & UART_INT_STATUS_RXFIFO_FULL) {
        UART(UART0).INT_CLEAR = UART_INT_CLEAR_RXFIFO_FULL;
        if (UART(UART0).STATUS & (UART_STATUS_RXFIFO_COUNT_M << UART_STATUS_RXFIFO_COUNT_S)) {
        	uint32_t count = uart0_num_char();
          // uint32_t t = micros();
            //t -= count*5; //~5us per byte

            uint8_t tmp[count];

            for(int i = 0 ; i < count; i++) {
            	uart0_rxqueue_push(uart_get());
            }

            _xt_clear_ints(1<<INUM_UART);


        }
    } else {
        //printf("Error: unexpected uart irq, INT_STATUS 0x%02x\n", UART(UART0).INT_STATUS);
    }
}


int IRAM uart0_getchar() {
	uint8_t c = 0;
	int ret;
	ret = xQueueReceive(rxqueue, (void*)&c, 1000);
	return ret == pdFALSE ? -1 : c;
}
void uart0_rxqueue_reset() {
	xQueueReset(rxqueue);
}

uint32_t uart0_rxcount() {
	return uxQueueMessagesWaiting(rxqueue);
}

uint32_t uart0_rxavail() {
	return  QUEUE_SIZE - uart0_rxcount();
}
void uart0_ovr_reset(void) {
	overruns = 0;
}
void uart0_rx_init(void)
{
    int trig_lvl = 80;
    uint8_t timeout = 20;
    //uart0_sem = xSemaphoreCreateCounting(UART0_RX_SIZE, 0);

    rxqueue = xQueueCreate(QUEUE_SIZE, 1);

    _xt_isr_attach(INUM_UART, uart0_rx_handler, NULL);
    _xt_isr_unmask(1 << INUM_UART);

    // reset the rx fifo
    uint32_t conf = UART(UART0).CONF0;
    UART(UART0).CONF0 = conf | UART_CONF0_RXFIFO_RESET;
    UART(UART0).CONF0 = conf & ~UART_CONF0_RXFIFO_RESET;

    // set rx fifo trigger
    UART(UART0).CONF1 |= ((trig_lvl & UART_CONF1_RXFIFO_FULL_THRESHOLD_M) << UART_CONF1_RXFIFO_FULL_THRESHOLD_S)
				| UART_CONF1_RX_TIMEOUT_ENABLE
				| (timeout & UART_CONF1_RX_TIMEOUT_THRESHOLD_M)<<UART_CONF1_RX_TIMEOUT_THRESHOLD_S;

    // clear all interrupts
    UART(UART0).INT_CLEAR = 0x1ff;

    // enable rx_interrupt
    UART(UART0).INT_ENABLE = UART_INT_ENABLE_RXFIFO_FULL;

}


uint32_t uart0_overruns() {
	return overruns;
}