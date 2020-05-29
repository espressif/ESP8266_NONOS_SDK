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

#include "ets_sys.h"
#include "c_types.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "driver/uart_register.h"
#include "driver/uart.h"

#define FUNC_UART0_CTS 4
#define FUNC_U1TXD_BK  2
#define UART_LINE_INV_MASK  (0x3f<<19)

/* UartDev is defined and initialized in ROM */
extern UARTDevice_t UartDev;

/*uart demo with a system task, to output what uart receives*/
/*this is a example to process uart data from task,please change the priority to fit your application task if exists*/
/*it might conflict with your task, if so,please arrange the priority of different task,  or combine it to a different event in the same task. */
#define uart_recvTaskPrio        0
#define uart_recvTaskQueueLen    10
os_event_t    uart_recvTaskQueue[uart_recvTaskQueueLen];

#define DBG0(x, ...)
#define DBG1(x, ...) uart_send_str_no_wait(UART1, (x), ## __VA_ARGS__)

static void uart0_rx_intr_handler(void *para);

/******************************************************************************
 * FunctionName : uart_config
 * Description  : Internal used function
 *                UART0 used for data TX/RX, RX buffer size is 0x100, interrupt enabled
 *                UART1 just used for debug output
 * Parameters   : uart_no, use UART0 or UART1 defined ahead
 * Returns      : NONE
*******************************************************************************/
static void ICACHE_FLASH_ATTR uart_config(UARTNum uart) {
    if (uart == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } else {
        /* rcv_buff size if 0x100 */
        ETS_UART_INTR_ATTACH(uart0_rx_intr_handler,  &(UartDev.rx_buff));
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
#if UART_HW_RTS
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);   /* HW FLOW CONTROL RTS PIN */
#endif
#if UART_HW_CTS
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_U0CTS);   /* HW FLOW CONTROL CTS PIN */
#endif
    }

    uart_div_modify(uart, UART_CLK_FREQ / (UartDev.baud_rate));

    WRITE_PERI_REG(UART_CONF0(uart),
    		  ((UartDev.exist_parity & UART_PARITY_EN_M)  <<  UART_PARITY_EN_S)
			| ((UartDev.parity & UART_PARITY_M)  << UART_PARITY_S)
			| ((UartDev.stop_bits & UART_STOP_BIT_NUM) << UART_STOP_BIT_NUM_S)
			| ((UartDev.data_bits & UART_BIT_NUM) << UART_BIT_NUM_S));

    /* clear rx and tx fifo,not ready */
    SET_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);

    if (uart == UART0) {
        /* set rx fifo trigger */
        WRITE_PERI_REG(UART_CONF1(uart),
        		((100 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
#if UART_HW_RTS
                       ((110 & UART_RX_FLOW_THRHD) << UART_RX_FLOW_THRHD_S) |
                       UART_RX_FLOW_EN |   /* enable rx flow control */
#endif
					   (0x02 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S |
                       UART_RX_TOUT_EN |
                       ((0x10 & UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S));
#if UART_HW_CTS
        SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);  /* add this sentence to add a tx flow control via MTCK( CTS ) */
#endif
        SET_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA);
    } else {
        WRITE_PERI_REG(UART_CONF1(uart), ((UartDev.rx_buff.trig_lvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S));
    }

    /* clear all interrupt */
    WRITE_PERI_REG(UART_INT_CLR(uart), 0xffff);
    /* enable rx_interrupt */
    SET_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_OVF_INT_ENA);
}

#if UART_BUFF_EN

static UARTBuffer_t *p_tx_buffer = NULL;
static UARTBuffer_t *p_rx_buffer = NULL;

static UARTBuffer_t * ICACHE_FLASH_ATTR init_uart_buf(uint32_t buf_size) {
    uint32_t heap_size = system_get_free_heap_size();

    if (heap_size <= buf_size) {
        DBG1("no buf for uart\n\r");
        return NULL;
    } else {
        DBG0("test heap size: %d\n\r", heap_size);
        UARTBuffer_t *p = (UARTBuffer_t *)os_malloc(sizeof(UARTBuffer_t));
        p->buf_size = buf_size;
        p->p_buf = (uint8_t *)os_malloc(p->buf_size);
        p->p_in_pos = p->p_buf;
        p->p_out_pos = p->p_buf;
        p->space = p->buf_size;
        p->buf_state = OK;
        p->next = NULL;
        p->tcp_control = RUN;
        return p;
    }
}

static void copy_uart_buf(UARTBuffer_t *p, char *pdata, uint16_t len) {
    if (len == 0) {
        return ;
    }

    uint16_t tail_len = p->p_buf + p->buf_size - p->p_in_pos;

    if (tail_len >= len) {
        os_memcpy(p->p_in_pos, pdata, len);
        p->p_in_pos += len;
        p->p_in_pos = (p->p_buf + (p->p_in_pos - p->p_buf) % p->buf_size);
        p->space -= len;
    } else {
        os_memcpy(p->p_in_pos, pdata, tail_len);
        p->p_in_pos += (tail_len);
        p->p_in_pos = (p->p_buf + (p->p_in_pos - p->p_buf) % p->buf_size);
        p->space -= tail_len;
        os_memcpy(p->p_in_pos, pdata + tail_len, len - tail_len);
        p->p_in_pos += (len - tail_len);
        p->p_in_pos = (p->p_buf + (p->p_in_pos - p->p_buf) % p->buf_size);
        p->space -= (len - tail_len);
    }

}

static void ICACHE_FLASH_ATTR free_uart_buf(UARTBuffer_t *p_buf) {
    os_free(p_buf->p_buf);
    os_free(p_buf);
}

static uint16_t ICACHE_FLASH_ATTR rx_buf_deq(char *pdata, uint16_t len) {
    uint16_t buf_len = (p_rx_buffer->buf_size - p_rx_buffer->space);
    uint16_t tail_len = p_rx_buffer->p_buf + p_rx_buffer->buf_size - p_rx_buffer->p_out_pos ;
    uint16_t len_tmp = 0;
    len_tmp = ((len > buf_len) ? buf_len : len);

    if (p_rx_buffer->p_out_pos <= p_rx_buffer->p_in_pos) {
        os_memcpy(pdata, p_rx_buffer->p_out_pos, len_tmp);
        p_rx_buffer->p_out_pos += len_tmp;
        p_rx_buffer->space += len_tmp;
    } else {
        if (len_tmp > tail_len) {
            os_memcpy(pdata, p_rx_buffer->p_out_pos, tail_len);
            p_rx_buffer->p_out_pos += tail_len;
            p_rx_buffer->p_out_pos = (p_rx_buffer->p_buf + (p_rx_buffer->p_out_pos - p_rx_buffer->p_buf) % p_rx_buffer->buf_size);
            p_rx_buffer->space += tail_len;

            os_memcpy(pdata + tail_len, p_rx_buffer->p_out_pos, len_tmp - tail_len);
            p_rx_buffer->p_out_pos += (len_tmp - tail_len);
            p_rx_buffer->p_out_pos = (p_rx_buffer->p_buf + (p_rx_buffer->p_out_pos - p_rx_buffer->p_buf) % p_rx_buffer->buf_size);
            p_rx_buffer->space += (len_tmp - tail_len);
        } else {
            /* os_printf("case 3 in rx deq\n\r"); */
            os_memcpy(pdata, p_rx_buffer->p_out_pos, len_tmp);
            p_rx_buffer->p_out_pos += len_tmp;
            p_rx_buffer->p_out_pos = (p_rx_buffer->p_buf + (p_rx_buffer->p_out_pos - p_rx_buffer->p_buf) % p_rx_buffer->buf_size);
            p_rx_buffer->space += len_tmp;
        }
    }

    if (p_rx_buffer->space >= UART_FIFO_LEN) {
        uart_rx_intr_enable(UART0);
    }

    return len_tmp;
}

static void rx_buf_enq(void) {
    uint8_t fifo_len, buf_idx;
    uint8_t fifo_data;
#if 1
    fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;

    if (fifo_len >= p_rx_buffer->space) {
        os_printf("buf full!!!\n\r");
    } else {
        buf_idx = 0;

        while (buf_idx < fifo_len) {
            buf_idx++;
            fifo_data = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
            *(p_rx_buffer->p_in_pos++) = fifo_data;

            if (p_rx_buffer->p_in_pos == (p_rx_buffer->p_buf + p_rx_buffer->buf_size)) {
                p_rx_buffer->p_in_pos = p_rx_buffer->p_buf;
            }
        }

        p_rx_buffer->space -= fifo_len ;

        if (p_rx_buffer->space >= UART_FIFO_LEN) {
            /* os_printf("after rx enq buf enough\n\r"); */
            uart_rx_intr_enable(UART0);
        }
    }

#endif
}

static void ICACHE_FLASH_ATTR tx_buf_enq(char *pdata, uint16_t len)
{
    CLEAR_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);

    if (p_tx_buffer == NULL) {
        DBG1("\n\rnull, create buffer struct\n\r");
        p_tx_buffer = init_uart_buf(UART_TX_BUFFER_SIZE);

        if (p_tx_buffer != NULL) {
            copy_uart_buf(p_tx_buffer,  pdata,  len);
        } else {
            DBG1("uart tx MALLOC no buf \n\r");
        }
    } else {
        if (len <= p_tx_buffer->space) {
        	copy_uart_buf(p_tx_buffer,  pdata,  len);
        } else {
            DBG1("UART TX BUF FULL!!!!\n\r");
        }
    }

#if 0

    if (p_tx_buffer->space <= URAT_TX_LOWER_SIZE) {
        set_tcp_block();
    }

#endif
    SET_PERI_REG_MASK(UART_CONF1(UART0), (UART_TX_EMPTY_THRESH_VAL & UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S);
    SET_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
}

static void tx_fifo_insert(UARTNum uart, UARTBuffer_t *p, uint8_t len)
{
    uint8_t i;

    for (i = 0; i < len; i++) {
        WRITE_PERI_REG(UART_FIFO(uart), *(p->p_out_pos++));

        if (p->p_out_pos == (p->p_buf + p->buf_size)) {
            p->p_out_pos = p->p_buf;
        }
    }

    p->p_out_pos = (p->p_buf + (p->p_out_pos - p->p_buf) % p->buf_size);
    p->space += len;
}

static void tx_buf_start(UARTNum uart) {
    uint8_t tx_fifo_len = (READ_PERI_REG(UART_STATUS(uart)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT;
    uint8_t fifo_remain = UART_FIFO_LEN - tx_fifo_len ;
    uint8_t len_tmp;
    uint16_t tail_ptx_len, head_ptx_len, data_len;
    /* struct UartBuffer* pTxBuff = *get_buff_prt(); */

    if (p_tx_buffer) {
        data_len = (p_tx_buffer->buf_size - p_tx_buffer->space);

        if (data_len > fifo_remain) {
            len_tmp = fifo_remain;
            tx_fifo_insert(uart, p_tx_buffer, len_tmp);
            SET_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
        } else {
            len_tmp = data_len;
            tx_fifo_insert(uart, p_tx_buffer, len_tmp);
        }
    } else {
        DBG1("pTxBuff null \n\r");
    }
}

#endif

static void ICACHE_FLASH_ATTR uart0_send_str(const char *str) { uart_send_str(UART0, str); }
void at_port_print(const char *str) __attribute__((alias("uart0_send_str")));

/******************************************************************************
 * FunctionName : uart0_rx_intr_handler
 * Description  : Internal used function
 *                UART0 interrupt handler, add self handle code inside
 * Parameters   : void *para - point to ETS_UART_INTR_ATTACH's arg
 * Returns      : NONE
*******************************************************************************/
/* ATTENTION:
 * IN NON-OS VERSION SDK, DO NOT USE "ICACHE_FLASH_ATTR" FUNCTIONS IN THE WHOLE HANDLER PROCESS
 * ALL THE FUNCTIONS CALLED IN INTERRUPT HANDLER MUST BE DECLARED IN RAM. IF NOT, POST AN EVENT
 * AND PROCESS IN SYSTEM TASK
 */

static void uart0_rx_intr_handler(void *para) {
    /* uart0 and uart1 intr combine together, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
    * uart1 and uart0 respectively
    */
    uint8_t rx_char;
    uint8_t uart = UART0;
    uint8_t fifo_len = 0;
    uint8_t buf_idx = 0;
    uint8_t temp, cnt;
    /* RcvMsgBuff *pRxBuff = (RcvMsgBuff *)para; */

    if (UART_FRM_ERR_INT_ST == (READ_PERI_REG(UART_INT_ST(uart)) & UART_FRM_ERR_INT_ST)) {
        DBG1("FRM_ERR\r\n");
        WRITE_PERI_REG(UART_INT_CLR(uart), UART_FRM_ERR_INT_CLR);
    } else if (UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(uart)) & UART_RXFIFO_FULL_INT_ST)) {
        DBG0("f");
        uart_rx_intr_disable(UART0);
        WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
        system_os_post(uart_recvTaskPrio, 0, 0);
    } else if (UART_RXFIFO_TOUT_INT_ST == (READ_PERI_REG(UART_INT_ST(uart)) & UART_RXFIFO_TOUT_INT_ST)) {
        DBG0("t");
        uart_rx_intr_disable(UART0);
        WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
        system_os_post(uart_recvTaskPrio, 0, 0);
    } else if (UART_TXFIFO_EMPTY_INT_ST == (READ_PERI_REG(UART_INT_ST(uart)) & UART_TXFIFO_EMPTY_INT_ST)) {
        DBG0("e");
        /* to output uart data from uart buffer directly in empty interrupt handler*/
        /*instead of processing in system event, in order not to wait for current task/function to quit */

        CLEAR_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
#if UART_BUFF_EN
        tx_buf_start(UART0);
#endif
        /* system_os_post(uart_recvTaskPrio, 1, 0); */
        WRITE_PERI_REG(UART_INT_CLR(uart), UART_TXFIFO_EMPTY_INT_CLR);

    } else if (UART_RXFIFO_OVF_INT_ST  == (READ_PERI_REG(UART_INT_ST(uart)) & UART_RXFIFO_OVF_INT_ST)) {
        WRITE_PERI_REG(UART_INT_CLR(uart), UART_RXFIFO_OVF_INT_CLR);
        DBG1("RX OVF!!\r\n");
    }

}

static void ICACHE_FLASH_ATTR uart_recv_task(os_event_t *events) {
    if (events->sig == 0) {
#if  UART_BUFF_EN
        rx_buf_enq();
#else
        uint8_t fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
        uint8_t d_tmp = 0;
        uint8_t idx = 0;

        for (idx = 0; idx < fifo_len; idx++) {
            d_tmp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
            uart_tx_one_char(UART0, d_tmp);
        }

        WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
        uart_rx_intr_enable(UART0);
#endif
    } else if (events->sig == 1) {
#if UART_BUFF_EN
        /* already move uart buffer output to uart empty interrupt */
    	tx_buf_start(UART0);
#else

#endif
    }
}

void ICACHE_FLASH_ATTR uart_init(UARTBaudRate uart0_br, UARTBaudRate uart1_br) {
    /* this is an example to process uart data from a system task
     * please change the priority to fit your application task if exists */
    system_os_task(uart_recv_task, uart_recvTaskPrio, uart_recvTaskQueue, uart_recvTaskQueueLen);

    UartDev.baud_rate = uart0_br;
    uart_config(UART0);
    UartDev.baud_rate = uart1_br;
    uart_config(UART1);
    ETS_UART_INTR_ENABLE();

#if UART_BUFF_EN
    p_tx_buffer = init_uart_buf(UART_TX_BUFFER_SIZE);
    p_rx_buffer = init_uart_buf(UART_RX_BUFFER_SIZE);
#endif


    /*option 1: use default print, output from uart0 , will wait some time if fifo is full */
    /* do nothing... */

    /*option 2: output from uart1,uart1 output will not wait , just for output debug info */
    /*os_printf output uart data via uart1(GPIO2)*/
    /* os_install_putc1((void *)uart1_write_char); */   /* use this one to output debug information via uart1 */

    /*option 3: output from uart0 will skip current byte if fifo is full now... */
    /*see uart0_write_char_no_wait:you can output via a buffer or output directly */
    /*os_printf output uart data via uart0 or uart buffer*/
    /* os_install_putc1((void *)uart0_write_char_no_wait); */ /* use this to print via uart0 */

#if UART_SELFTEST&UART_BUFF_EN
    os_timer_disarm(&buff_timer_t);
    os_timer_setfn(&buff_timer_t, uart_test_rx, NULL);    /* a demo to process the data in uart rx buffer */
    os_timer_arm(&buff_timer_t, 10, 1);
#endif
}

void ICACHE_FLASH_ATTR uart_reattach(void) {
    uart_init(BIT_RATE_115200, BIT_RATE_115200);
}

/******************************************************************************
 * FunctionName : uart_tx_one_char
 * Description  : Internal used function
 *                Use uart1 interface to transfer one char
 * Parameters   : uint8_t TxChar - character to tx
 * Returns      : OK
*******************************************************************************/
STATUS uart_tx_one_char(UARTNum uart, uint8_t tx_har) {
    while (true) {
        uint32_t fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart), tx_har);
    return OK;
}

/******************************************************************************
 * FunctionName : uart_tx_one_char_no_wait
 * Description  : uart tx a single char without waiting for fifo
 * Parameters   : uint8_t uart - uart port
 *                uint8_t TxChar - char to tx
 * Returns      : STATUS
*******************************************************************************/
STATUS uart_tx_one_char_no_wait(UARTNum uart, uint8_t tx_char) {
    uint8_t fifo_cnt = ((READ_PERI_REG(UART_STATUS(uart)) >> UART_TXFIFO_CNT_S)& UART_TXFIFO_CNT);

    if (fifo_cnt < 126) {
        WRITE_PERI_REG(UART_FIFO(uart), tx_char);
    }

    return OK;
}

/******************************************************************************
 * FunctionName : uart_send_str_no_wait
 * Description  : uart tx a string without waiting for every char, used for print debug info which can be lost
 * Parameters   : const char *str - string to be sent
 * Returns      : NONE
*******************************************************************************/
void uart_send_str_no_wait(UARTNum uart, const char *str)
{
    while (*str) {
        uart_tx_one_char_no_wait(uart, *str++);
    }
}

void ICACHE_FLASH_ATTR uart_tx_send_buffer(UARTNum uart, uint8_t *buf, uint16_t len) {
    uint16_t i;

    for (i = 0; i < len; i++) {
        uart_tx_one_char(uart, buf[i]);
    }
}

void uart_rx_intr_disable(UARTNum uart)
{
#if 1
    CLEAR_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA);
#else
    ETS_UART_INTR_DISABLE();
#endif
}

void uart_rx_intr_enable(UARTNum uart) {
#if 1
    SET_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA);
#else
    ETS_UART_INTR_ENABLE();
#endif
}

void ICACHE_FLASH_ATTR uart_set_num_bits(UARTNum uart, UARTNumBits len) {
    SET_PERI_REG_BITS(UART_CONF0(uart), UART_BIT_NUM, len, UART_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR uart_set_stop_bits(UARTNum uart, UARTStopBits bits) {
    SET_PERI_REG_BITS(UART_CONF0(uart), UART_STOP_BIT_NUM, bits, UART_STOP_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR uart_set_line_inverse(UARTNum uart, UARTLineLevelInverse inverse_mask) {
    CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0(uart), inverse_mask);
}

void ICACHE_FLASH_ATTR uart_set_parity(UARTNum uart, UARTParity parity) {
    CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_PARITY | UART_PARITY_EN);

    if (parity == NONE_BITS) {
    } else {
        SET_PERI_REG_MASK(UART_CONF0(uart), parity | UART_PARITY_EN);
    }
}

void ICACHE_FLASH_ATTR uart_set_rate(UARTNum uart, uint32_t rate) {
    uart_div_modify(uart, UART_CLK_FREQ / rate);
}

void ICACHE_FLASH_ATTR uart_set_flow_ctrl(UARTNum uart, UARTHwFlowCtrl flow_ctrl, uint8_t rx_thresh)
{
    if (flow_ctrl & UART_HW_FLOWCONTROL_RTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
        SET_PERI_REG_BITS(UART_CONF1(uart), UART_RX_FLOW_THRHD, rx_thresh, UART_RX_FLOW_THRHD_S);
        SET_PERI_REG_MASK(UART_CONF1(uart), UART_RX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF1(uart), UART_RX_FLOW_EN);
    }

    if (flow_ctrl & UART_HW_FLOWCONTROL_CTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_UART0_CTS);
        SET_PERI_REG_MASK(UART_CONF0(uart), UART_TX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_TX_FLOW_EN);
    }
}

/*  do not use if tx flow control enabled  */
void ICACHE_FLASH_ATTR uart_wait_tx_fifo_empty(UARTNum uart, uint32_t time_out_us) {
    uint32_t t_s = system_get_time();

    while (READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S)) {

        if ((system_get_time() - t_s) > time_out_us) {
            break;
        }
        WRITE_PERI_REG(0X60000914, 0X73); /* WTD */
    }
}

bool ICACHE_FLASH_ATTR uart_check_output_finished(UARTNum uart, uint32_t time_out_us) {
    uint32_t t_start = system_get_time();
    uint8_t tx_fifo_len;
    uint32_t tx_buff_len = 0;

    while (1) {
        tx_fifo_len = ((READ_PERI_REG(UART_STATUS(uart)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT);

#if UART_BUFF_EN
        if (p_tx_buffer) {
            tx_buff_len = ((p_tx_buffer->buf_size) - (p_tx_buffer->space));
        }
#endif
        if (tx_fifo_len == 0 && tx_buff_len == 0) {
            return true;
        }

        if (system_get_time() - t_start > time_out_us) {
            return false;
        }
        WRITE_PERI_REG(0X60000914, 0X73); /* WTD */
    }
}

void ICACHE_FLASH_ATTR uart_reset_fifo(UARTNum uart) {
    SET_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);
}

void ICACHE_FLASH_ATTR uart_clear_intr_status(UARTNum uart, uint32_t clr_mask) {
    WRITE_PERI_REG(UART_INT_CLR(uart), clr_mask);
}

void ICACHE_FLASH_ATTR uart_set_intr_enable(UARTNum uart, uint32_t ena_mask) {
    SET_PERI_REG_MASK(UART_INT_ENA(uart), ena_mask);
}

static void ICACHE_FLASH_ATTR uart0_write_char_no_wait(char c)
{
#if UART_BUFF_EN
    uint8_t chr;

    if (c == '\n') {
        chr = '\r';
        tx_buf_enq(&chr, 1);
        chr = '\n';
        tx_buf_enq(&chr, 1);
    } else if (c == '\r') {

    } else {
        tx_buf_enq(&c, 1);
    }

#else
    if (c == '\n') {
        uart_tx_one_char_no_wait(UART0, '\r');
        uart_tx_one_char_no_wait(UART0, '\n');
    } else if (c == '\r') {

    } else {
        uart_tx_one_char_no_wait(UART0, c);
    }
#endif
}

static void uart_write_char(UARTNum uart, char c) {
    if (c == '\n') {
        uart_tx_one_char(UART0, '\r');
        uart_tx_one_char(UART0, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_one_char(UART0, c);
    }
}

static void ICACHE_FLASH_ATTR uart0_write_char(char c) { uart_write_char(UART1, c); }
static void ICACHE_FLASH_ATTR uart1_write_char(char c) { uart_write_char(UART0, c); }

void ICACHE_FLASH_ATTR uart_set_print_port(UARTNum uart)
{
    if (uart == UART1) {
        os_install_putc1(uart1_write_char);
    } else {
        /* option 1: do not wait if uart fifo is full, drop current character */
        os_install_putc1(uart0_write_char_no_wait);
        /* option 2: wait for a while if uart fifo is full */
        os_install_putc1(uart0_write_char);
    }
}

#if UART_SELFTEST

#if UART_BUFF_EN
os_timer_t buff_timer_t;

void ICACHE_FLASH_ATTR uart_test_rx()
{
    uint8_t uart_buf[128] = {0};
    uint16_t len = 0;
    len = rx_buff_deq(uart_buf, 128);
    tx_buff_enq(uart_buf, len);
}
#endif

void ICACHE_FLASH_ATTR uart_init_2(UARTBaudRate uart0_br, UARTBaudRate uart1_br) {
    /* rom use 74880 baut_rate, here reinitialize */
    UartDev.baud_rate = uart0_br;
    UartDev.exist_parity = STICK_PARITY_EN;
    UartDev.parity = EVEN_BITS;
    UartDev.stop_bits = ONE_STOP_BIT;
    UartDev.data_bits = EIGHT_BITS;

    uart_config(UART0);
    UartDev.baud_rate = uart1_br;
    uart_config(UART1);
    ETS_UART_INTR_ENABLE();

    /* install uart1 putc callback */
    os_install_putc1((void *)uart1_write_char); /* print output at UART1 */
}

#endif

