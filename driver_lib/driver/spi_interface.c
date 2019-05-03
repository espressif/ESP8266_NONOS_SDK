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
#include "osapi.h"
#include "driver/spi_interface.h"

void ICACHE_FLASH_ATTR spi_init(SpiNum spiNum, SpiAttr *pAttr) {
    if ((spiNum > SpiNum_HSPI)
            || (NULL == pAttr)) {
        return;
    }

    /* SPI_CPOL & SPI_CPHA */
    switch (pAttr->subMode) {
        case SpiSubMode_1:
            CLEAR_PERI_REG_MASK(SPI_PIN(spiNum), SPI_IDLE_EDGE);
            SET_PERI_REG_MASK(SPI_USER(spiNum),  SPI_CK_OUT_EDGE); /* CHPA_FALLING_EDGE_SAMPLE */
            break;

        case SpiSubMode_2:
            SET_PERI_REG_MASK(SPI_PIN(spiNum), SPI_IDLE_EDGE);
            SET_PERI_REG_MASK(SPI_USER(spiNum),  SPI_CK_OUT_EDGE); /* CHPA_FALLING_EDGE_SAMPLE */
            break;

        case SpiSubMode_3:
            SET_PERI_REG_MASK(SPI_PIN(spiNum), SPI_IDLE_EDGE);
            CLEAR_PERI_REG_MASK(SPI_USER(spiNum),  SPI_CK_OUT_EDGE);
            break;

        case SpiSubMode_0:
        default:
            CLEAR_PERI_REG_MASK(SPI_PIN(spiNum), SPI_IDLE_EDGE);
            CLEAR_PERI_REG_MASK(SPI_USER(spiNum),  SPI_CK_OUT_EDGE);
            break;
    }

    if (SpiBitOrder_MSBFirst == pAttr->bitOrder) {
        CLEAR_PERI_REG_MASK(SPI_CTRL(spiNum), SPI_WR_BIT_ORDER);
        CLEAR_PERI_REG_MASK(SPI_CTRL(spiNum), SPI_RD_BIT_ORDER);
    } else if (SpiBitOrder_LSBFirst == pAttr->bitOrder) {
        SET_PERI_REG_MASK(SPI_CTRL(spiNum), SPI_WR_BIT_ORDER);
        SET_PERI_REG_MASK(SPI_CTRL(spiNum), SPI_RD_BIT_ORDER);
    } else {
    }

    /* Disable flash operation mode
       As earlier as better, if not SPI_CTRL2 can not to be set delay cycles. */
    CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_FLASH_MODE);

    if (SpiMode_Master == pAttr->mode) {
        /* SPI mode type */
        CLEAR_PERI_REG_MASK(SPI_SLAVE(spiNum), SPI_SLAVE_MODE);
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO_HIGHPART); /* By default slave send buffer C0-C7 */

        /* SPI Speed */
        if (1 < (pAttr->speed)) {
            uint8_t i, k;
            i = (pAttr->speed / 40) ? (pAttr->speed / 40) : 1;
            k = pAttr->speed / i;
            CLEAR_PERI_REG_MASK(SPI_CLOCK(spiNum), SPI_CLK_EQU_SYSCLK);
            WRITE_PERI_REG(SPI_CLOCK(spiNum),
                           (((i - 1) & SPI_CLKDIV_PRE) << SPI_CLKDIV_PRE_S) |
                           (((k - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) |
                           ((((k + 1) / 2 - 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) |
                           (((k - 1) & SPI_CLKCNT_L) << SPI_CLKCNT_L_S)); /* clear bit 31,set SPI clock div */
        } else {
            WRITE_PERI_REG(SPI_CLOCK(spiNum), SPI_CLK_EQU_SYSCLK); /* 80Mhz speed */
        }

        /* By default format:CMD+ADDR+DATA */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
        SET_PERI_REG_MASK(SPI_CTRL2(spiNum), ((0x1 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
    } else if (SpiMode_Slave == pAttr->mode) {
        /* BIT19 must do */
        SET_PERI_REG_MASK(SPI_PIN(spiNum), BIT19);

        /* SPI mode type */
        SET_PERI_REG_MASK(SPI_SLAVE(spiNum), SPI_SLAVE_MODE);
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO_HIGHPART); /* By default slave send buffer C8-C15 */

        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI);

        /* If do not set delay cycles, slave not working,master cann't get the data. */
        SET_PERI_REG_MASK(SPI_CTRL2(spiNum), ((0x1 & SPI_MOSI_DELAY_NUM) << SPI_MOSI_DELAY_NUM_S));
        /* SPI Speed */
        WRITE_PERI_REG(SPI_CLOCK(spiNum), 0);

        /* By default format::CMD(8bits)+ADDR(8bits)+DATA(32bytes) */
        SET_PERI_REG_BITS(SPI_USER2(spiNum), SPI_USR_COMMAND_BITLEN,
                          7, SPI_USR_COMMAND_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLAVE1(spiNum), SPI_SLV_WR_ADDR_BITLEN,
                          7, SPI_SLV_WR_ADDR_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLAVE1(spiNum), SPI_SLV_RD_ADDR_BITLEN,
                          7, SPI_SLV_RD_ADDR_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLAVE1(spiNum), SPI_SLV_BUF_BITLEN,
                          (32 * 8 - 1), SPI_SLV_BUF_BITLEN_S);
        /* For 8266 work on slave mode */
        SET_PERI_REG_BITS(SPI_SLAVE1(spiNum), SPI_SLV_STATUS_BITLEN,
                          7, SPI_SLV_STATUS_BITLEN_S);
    } else {
    }

    /* clear Dual or Quad lines transmission mode */
    CLEAR_PERI_REG_MASK(SPI_CTRL(spiNum), SPI_QIO_MODE | SPI_DIO_MODE | SPI_DOUT_MODE | SPI_QOUT_MODE);
    /* Clear the data buffer */
    uint8_t i;
    uint32_t regAddr = REG_SPI_BASE(spiNum) + 0x40;

    for (i = 0; i < 16; ++i) {
        WRITE_PERI_REG(regAddr, 0);
        regAddr += 4;
    }
}

void ICACHE_FLASH_ATTR spi_master_config_addr(SpiNum spiNum, uint32_t addr) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }
    WRITE_PERI_REG(SPI_ADDR(spiNum), addr);
}

void ICACHE_FLASH_ATTR spi_master_config_cmd(SpiNum spiNum, uint32_t cmd) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }

    /* SPI_USER2 bit28-31 is cmd length,cmd bit length is value(0-15)+1,
       bit15-0 is cmd value */
    SET_PERI_REG_BITS(SPI_USER2(spiNum), SPI_USR_COMMAND_VALUE, cmd, SPI_USR_COMMAND_VALUE_S);
}

int32_t ICACHE_FLASH_ATTR spi_master_send_data(SpiNum spiNum, SpiData *pInData) {
    char idx = 0;

    if ((spiNum > SpiNum_HSPI)
            || (NULL == pInData)
            || (64 < pInData->dataLen)) {
        return -1;
    }

    uint32_t *value = pInData->data;

    while (READ_PERI_REG(SPI_CMD(spiNum))&SPI_USR);

    /* Set command */
    if (pInData->cmdLen != 0) {
        /* Max command length 16 bits */
        SET_PERI_REG_BITS(SPI_USER2(spiNum), SPI_USR_COMMAND_BITLEN,
                          ((pInData->cmdLen << 3) - 1), SPI_USR_COMMAND_BITLEN_S);
        /* Enable and load command */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_COMMAND);
        spi_master_config_cmd(spiNum, pInData->cmd);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_COMMAND);
        SET_PERI_REG_BITS(SPI_USER2(spiNum), SPI_USR_COMMAND_BITLEN,
                          0, SPI_USR_COMMAND_BITLEN_S);
    }

    /* Set address */
    if (pInData->addrLen == 0) {
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_ADDR);
        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_ADDR_BITLEN,
                          0, SPI_USR_ADDR_BITLEN_S);
    } else {
        if (NULL == pInData->addr) {
            return -1;
        }

        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_ADDR_BITLEN,
                          ((pInData->addrLen << 3) - 1), SPI_USR_ADDR_BITLEN_S);
        /* Enable and load address */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_ADDR);
        spi_master_config_addr(spiNum, *pInData->addr);
    }

    /* Set data */
    if (pInData->dataLen != 0) {
        if (NULL == value) {
            return -1;
        }

        /* Enable MOSI */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI);
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO);

        /* Load send buffer */
        do {
            WRITE_PERI_REG((SPI_W0(spiNum) + (idx << 2)), *value++);
        } while (++idx < ((pInData->dataLen / 4) + ((pInData->dataLen % 4) ? 1 : 0)));

        /* Set data send buffer length.Max data length 64 bytes. */
        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_MOSI_BITLEN, ((pInData->dataLen << 3) - 1), SPI_USR_MOSI_BITLEN_S);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI);
        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_MOSI_BITLEN,
                          0, SPI_USR_MOSI_BITLEN_S);
    }

    /* Start send */
    SET_PERI_REG_MASK(SPI_CMD(spiNum), SPI_USR);

    /* Wait for transmit done */
    while (!(READ_PERI_REG(SPI_SLAVE(spiNum))&SPI_TRANS_DONE));

    CLEAR_PERI_REG_MASK(SPI_SLAVE(spiNum), SPI_TRANS_DONE);
    return 0;
}

/**
 * @brief Receive data from slave.
 *
 */
int32_t ICACHE_FLASH_ATTR spi_master_recv_data(SpiNum spiNum, SpiData *pOutData) {
    char idx = 0;

    if ((spiNum > SpiNum_HSPI)
            || (NULL == pOutData)) {
        return -1;
    }

    uint32_t *value = pOutData->data;

    while (READ_PERI_REG(SPI_CMD(spiNum))&SPI_USR);

    /* Set command */
    if (pOutData->cmdLen != 0) {
        /* Max command length 16 bits. */
        SET_PERI_REG_BITS(SPI_USER2(spiNum), SPI_USR_COMMAND_BITLEN,
                          ((pOutData->cmdLen << 3) - 1), SPI_USR_COMMAND_BITLEN_S);
        /* Enable and load command */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_COMMAND);
        spi_master_config_cmd(spiNum, pOutData->cmd);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_COMMAND);
        SET_PERI_REG_BITS(SPI_USER2(spiNum), SPI_USR_COMMAND_BITLEN,
                          0, SPI_USR_COMMAND_BITLEN_S);
    }

    /* Set Address */
    if (pOutData->addrLen == 0) {
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_ADDR);
        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_ADDR_BITLEN,
                          0, SPI_USR_ADDR_BITLEN_S);
    } else {
        if (NULL == pOutData->addr) {
            return -1;
        }

        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_ADDR_BITLEN,
                          ((pOutData->addrLen << 3) - 1), SPI_USR_ADDR_BITLEN_S);
        /* Enable and load  address */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_ADDR);
        spi_master_config_addr(spiNum, *pOutData->addr);
    }

    /* Set data */
    if (pOutData->dataLen != 0) {
        if (NULL == value) {
            return -1;
        }

        /* Clear MOSI enable */
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI);
        /* Enable MOSI */
        SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO);
        /* Set data send buffer length.Max data length 64 bytes. */
        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_MISO_BITLEN, ((pOutData->dataLen << 3) - 1), SPI_USR_MISO_BITLEN_S);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI);
        CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO);
        SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_MISO_BITLEN,
                          0, SPI_USR_MISO_BITLEN_S);
    }

    /* Start send */
    SET_PERI_REG_MASK(SPI_CMD(spiNum), SPI_USR);

    while (READ_PERI_REG(SPI_CMD(spiNum))&SPI_USR);

    /* Read data out */
    do {
        *value++ = READ_PERI_REG(SPI_W0(spiNum) + (idx << 2));
    } while (++idx < ((pOutData->dataLen / 4) + ((pOutData->dataLen % 4) ? 1 : 0)));

    return 0;
}

/**
 * @brief Load data to send buffer by slave mode.
 *
 */
int32_t ICACHE_FLASH_ATTR spi_slave_send_data(SpiNum spiNum, uint32_t *pInData, uint8_t inLen) {
    if (NULL == pInData) {
        return -1;
    }

    uint32_t *value = pInData;
    char i;

    for (i = 0; i < inLen; ++i) {
        WRITE_PERI_REG((SPI_W8(spiNum) + (i << 2)), *value++);
    }

    /* Enable slave transmission listen */
    SET_PERI_REG_MASK(SPI_CMD(spiNum), SPI_USR);
    return 0;
}

/**
 * @brief Configure slave prepare for receive data.
 *
 */
int32_t ICACHE_FLASH_ATTR spi_slave_recv_data(SpiNum spiNum) {
    if ((spiNum > SpiNum_HSPI)) {
        return -1;
    }

    /* Enable slave transmission listen */
    SET_PERI_REG_MASK(SPI_CMD(spiNum), SPI_USR);

    return 0;
}

/**
 * @brief Send data to slave(ESP8266 register of RD_STATUS or WR_STATUS).
 *
 */
void ICACHE_FLASH_ATTR spi_master_send_status(SpiNum spiNum, uint8_t data) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }

    while (READ_PERI_REG(SPI_CMD(spiNum))&SPI_USR);

    /* Enable MOSI */
    SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI);
    CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO | SPI_USR_DUMMY | SPI_USR_ADDR);

    /* 8bits cmd, 0x04 is eps8266 slave write cmd value */
    WRITE_PERI_REG(SPI_USER2(spiNum),
                   ((7 & SPI_USR_COMMAND_BITLEN) << SPI_USR_COMMAND_BITLEN_S)
                   | MASTER_WRITE_STATUS_TO_SLAVE_CMD);
    /* Set data send buffer length. */
    SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_MOSI_BITLEN,
                      ((sizeof(data) << 3) - 1), SPI_USR_MOSI_BITLEN_S);

    WRITE_PERI_REG(SPI_W0(spiNum), (uint32_t)(data));
    /* Start SPI */
    SET_PERI_REG_MASK(SPI_CMD(spiNum), SPI_USR);

}

/**
 * @brief Receive status register from slave(ESP8266).
 *
 */
int32_t ICACHE_FLASH_ATTR spi_master_recv_status(SpiNum spiNum) {
    if (spiNum > SpiNum_HSPI) {
        return -1;
    }

    while (READ_PERI_REG(SPI_CMD(spiNum))&SPI_USR);

    /* Enable MISO */
    SET_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MISO);
    CLEAR_PERI_REG_MASK(SPI_USER(spiNum), SPI_USR_MOSI | SPI_USR_DUMMY | SPI_USR_ADDR);

    /* 8bits cmd, 0x06 is eps8266 slave read status cmd value */
    WRITE_PERI_REG(SPI_USER2(spiNum),
                   ((7 & SPI_USR_COMMAND_BITLEN) << SPI_USR_COMMAND_BITLEN_S)
                   | MASTER_READ_STATUS_FROM_SLAVE_CMD);
    /* Set receive buffer length. */
    SET_PERI_REG_BITS(SPI_USER1(spiNum), SPI_USR_MISO_BITLEN,
                      7, SPI_USR_MISO_BITLEN_S);

    /* start SPI */
    SET_PERI_REG_MASK(SPI_CMD(spiNum), SPI_USR);

    while (READ_PERI_REG(SPI_CMD(spiNum))&SPI_USR);

    uint8_t data = (uint8_t)(READ_PERI_REG(SPI_W0(spiNum)) & 0xff);

    return (uint8_t)(READ_PERI_REG(SPI_W0(spiNum)) & 0xff);
}

/**
 * @brief Select SPI CS pin.
 *
 */
void ICACHE_FLASH_ATTR spi_cs_pin_select(SpiNum spiNum, SpiPinCS pinCs) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }

    SET_PERI_REG_BITS(SPI_PIN(spiNum), 3, 0, 0);
    SET_PERI_REG_MASK(SPI_PIN(spiNum), pinCs);
}

void spi_int_cfg(SpiNum spiNum, SpiIntInfo *pIntInfo) {
    if ((spiNum > SpiNum_HSPI)
            || (NULL == pIntInfo)) {
        return;
    }

    /* Clear the interrupt source and disable all of the interrupt. */
    CLEAR_PERI_REG_MASK(SPI_SLAVE(spiNum), 0x3FF);
    spi_int_enable(spiNum, pIntInfo->src);
    os_printf("src=%x\r\n,isrFunc=%x", (pIntInfo->src << 5), pIntInfo->isrFunc);

    /* set and enable ISR */
    ETS_SPI_INTR_ATTACH(pIntInfo->isrFunc, NULL);
    ETS_SPI_INTR_ENABLE();
}

/**
 * @brief Enable SPI interrupt source.
 *
 */
void ICACHE_FLASH_ATTR spi_int_enable(SpiNum spiNum, SpiIntSrc intSrc) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }

    SET_PERI_REG_MASK(SPI_SLAVE(spiNum), (intSrc << 5));
}

/**
 * @brief Disable SPI interrupt source.
 *
 */
void ICACHE_FLASH_ATTR spi_int_disable(SpiNum spiNum, SpiIntSrc intSrc) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }

    CLEAR_PERI_REG_MASK(SPI_SLAVE(spiNum), intSrc);
}

/**
 * @brief Clear all of SPI interrupt source.
 *
 */
void ICACHE_FLASH_ATTR spi_int_clear(SpiNum spiNum) {
    if (spiNum > SpiNum_HSPI) {
        return;
    }

    CLEAR_PERI_REG_MASK(SPI_SLAVE(spiNum), SpiIntSrc_TransDone
                        | SpiIntSrc_WrStaDone
                        | SpiIntSrc_RdStaDone
                        | SpiIntSrc_WrBufDone
                        | SpiIntSrc_RdBufDone);
}
