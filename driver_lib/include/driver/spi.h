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

#ifndef _SPI_H_
#define _SPI_H_

#include "spi_register.h"
#include "ets_sys.h"
#include "osapi.h"
#include "uart.h"
#include "os_type.h"
#include "spi_flash.h"

#define SPI_FLASH_BYTES_LEN   24
#define IODATA_START_ADDR     BIT0
#define SPI_BUFF_BYTE_NUM     32

/*SPI number define*/
#define SPI             0
#define HSPI            1

void cache_flush(void);
void spi_master_init(uint8_t spi_no);
void spi_lcd_9bit_write(uint8_t spi_no, uint8_t high_bit, uint8_t low_8bit);
void spi_mast_byte_write(uint8_t spi_no, uint8_t data);

/* read and write data to esp8266 slave buffer,which needs 16bit transmission ,
   first byte is master command 0x04, second byte is master data */
void spi_byte_write_espslave(uint8_t spi_no, uint8_t data);
void spi_byte_read_espslave(uint8_t spi_no, uint8_t *data);

void spi_slave_init(uint8_t spi_no, uint8_t data_len);
void spi_slave_isr_handler(void *para);

/* hspi test function, used to test esp8266 spi slave */
void hspi_master_readwrite_repeat(void);
void spi_test_init(void);

#endif /* _SPI_H_ */

