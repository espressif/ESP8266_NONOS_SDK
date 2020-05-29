/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef UART_APP_H
#define UART_APP_H

#include "eagle_soc.h"
#include "c_types.h"
#include "driver/uart_register.h"

#define UART_TX_BUFFER_SIZE 256  /* Ring buffer length of tx buffer */
#define UART_RX_BUFFER_SIZE 256  /* Ring buffer length of rx buffer */

#define UART_BUFF_EN  0   /* set 1: use uart buffer, FOR UART0 */
#define UART_SELFTEST 0   /* set 1: enable the loop test demo for uart buffer, FOR UART0 */

#define UART_HW_RTS   0   /* set 1: enable uart hw flow control RTS, PIN MTDO, FOR UART0 */
#define UART_HW_CTS   0   /* set 1: enable uart hw flow control CTS, PIN MTCK, FOR UART0 */

typedef enum {
	UART0=0x0,
	UART1=0x1
} UARTNum;


typedef enum {
    FIVE_BITS  = 0x0,
    SIX_BITS   = 0x1,
    SEVEN_BITS = 0x2,
    EIGHT_BITS = 0x3
} UARTNumBits;

typedef enum {
    ONE_STOP_BIT       = 0x1,
    ONE_HALF_STOP_BIT  = 0x2,
    TWO_STOP_BIT       = 0x3
} UARTStopBits;

typedef enum {
    EVEN_BITS = 0x0,
    ODD_BITS  = 0x1,
    NONE_BITS = 0x2
} UARTParity;

typedef enum {
    STICK_PARITY_DIS   = 0,
    STICK_PARITY_EN    = 1
} UARTExistParity;

typedef enum {
    UART_NONE_INVERSE = 0x0,
    UART_RXD_INVERSE  = UART_RXD_INV,
    UART_CTS_INVERSE  = UART_CTS_INV,
    UART_TXD_INVERSE  = UART_TXD_INV,
    UART_RTS_INVERSE  = UART_RTS_INV
} UARTLineLevelInverse;


typedef enum {
    BIT_RATE_300     = 300,
    BIT_RATE_600     = 600,
    BIT_RATE_1200    = 1200,
    BIT_RATE_2400    = 2400,
    BIT_RATE_4800    = 4800,
    BIT_RATE_9600    = 9600,
    BIT_RATE_19200   = 19200,
    BIT_RATE_38400   = 38400,
    BIT_RATE_57600   = 57600,
    BIT_RATE_74880   = 74880,
    BIT_RATE_115200  = 115200,
    BIT_RATE_230400  = 230400,
    BIT_RATE_460800  = 460800,
    BIT_RATE_921600  = 921600,
    BIT_RATE_1843200 = 1843200,
    BIT_RATE_3686400 = 3686400
} UARTBaudRate;

typedef enum {
    NONE_CTRL,
    HARDWARE_CTRL,
    XON_XOFF_CTRL
} UARTFlowCtrl;

typedef enum {
    UART_HW_FLOWCONTROL_NONE    = 0x0,
    UART_HW_FLOWCONTROL_RTS     = 0x1,
    UART_HW_FLOWCONTROL_CTS     = 0x2,
    UART_HW_FLOWCONTROL_CTS_RTS = 0x3
} UARTHwFlowCtrl;

typedef enum {
    EMPTY,
    UNDER_WRITE,
    WRITE_OVER
} UARTRxMsgBuffState;

typedef enum {
    BAUD_RATE_DET,
    WAIT_SYNC_FRM,
    SRCH_MSG_HEAD,
    RCV_MSG_BODY,
    RCV_ESC_CHAR
} UARTRxMsgState;

typedef enum {
    RUN   = 0,
    BLOCK = 1
} TCPState;

typedef struct {
    uint32_t     		rx_buff_size;
    uint8_t     		*p_rx_buff;
    uint8_t     		*p_write_pos;
    uint8_t     		*p_read_pos;
    uint8_t      		trig_lvl;
    UARTRxMsgBuffState 	buff_state;
} UARTRxMsgBuff_t;

typedef struct {
    uint32_t	tx_buff_size;
    uint8_t   	*p_tx_buff;
} UARTTxMsgBuff_t;

typedef struct {
    UARTBaudRate		baud_rate;
    UARTNumBits			data_bits;
    UARTExistParity		exist_parity;
    UARTParity			parity;
    UARTStopBits		stop_bits;
    UARTFlowCtrl		flow_ctrl;
    UARTRxMsgBuff_t		rx_buff;
    UARTTxMsgBuff_t		tx_buff;
    UARTRxMsgState		rx_state;
    int32_t				received;
    int32_t				buff_uart_no;  /* indicate which uart use tx/rx buffer */
} UARTDevice_t;

#define UART_FIFO_LEN  128  /* define the tx fifo length */
#define UART_TX_EMPTY_THRESH_VAL 0x10

typedef struct UARTBuffer {
    uint32_t			buf_size;
    uint8_t				*p_buf;
    uint8_t				*p_in_pos;
    uint8_t				*p_out_pos;
    STATUS				buf_state;
    uint16_t			space;  			/* remaining buffer space */
    uint8_t				tcp_control;
    struct  UARTBuffer	*next;
}UARTBuffer_t;

typedef struct{
    uint32_t			buf_size;
    uint8_t				*p_buf;
    uint8_t				*p_in_pos;
    uint8_t				*p_out_pos;
    STATUS				buf_state;
    uint16_t			space;  			/* remaining buffer space */
} UARTRxBuff_t;

void uart_rx_intr_enable(UARTNum uart);
void uart_rx_intr_disable(UARTNum uart);

STATUS uart_tx_one_char(UARTNum uart, uint8_t tx_char);
STATUS uart_tx_one_char_no_wait(UARTNum uart, uint8_t tx_char);
void uart_send_str(UARTNum uart, const char *str);
void uart_send_str_no_wait(UARTNum uart, const char *str);
void uart_tx_send_buffer(UARTNum uart, uint8_t *buf, uint16_t len);

void uart_init(UARTBaudRate uart0_br, UARTBaudRate uart1_br);
void uart_reattach(void);
void uart_set_num_bits(UARTNum uart, UARTNumBits len);
void uart_set_stop_bits(UARTNum uart, UARTStopBits bits);
void uart_set_line_inverse(UARTNum uart, UARTLineLevelInverse inverse_mask);
void uart_set_parity(UARTNum uart, UARTParity parity);
void uart_set_rate(UARTNum uart, uint32_t baud_rate);
void uart_set_flow_ctrl(UARTNum uart, UARTHwFlowCtrl flow_ctrl, uint8_t rx_thresh);
void uart_wait_tx_fifo_empty(UARTNum uart, uint32_t time_out_us);  /* do not use if tx flow control enabled */
void uart_reset_fifo(UARTNum uart);
void uart_clear_intr_status(UARTNum uart, uint32_t clr_mask);
void uart_set_intr_enable(UARTNum uart, uint32_t ena_mask);
void uart_set_print_port(UARTNum uart);
bool uart_check_output_finished(UARTNum uart, uint32_t time_out_us);

#endif

