// Copyright 2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/errno.h>

#include "osapi.h"
#include "mem.h"

#include "spi_flash.h"

#include "eagle_soc.h"
#include "driver/spi_register.h"

typedef volatile struct {
    union {
        struct {
            uint32_t reserved0: 18;                         /*reserved*/
            uint32_t usr:        1;                         /*User define command enable.  An operation will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_hpm:  1;                         /*Drive Flash into high performance mode.  The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_res:  1;                         /*This bit combined with reg_resandres bit releases Flash from the power-down state or high performance mode and obtains the devices ID. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_dp:   1;                         /*Drive Flash into power down.  An operation will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_ce:   1;                         /*Chip erase enable. Chip erase operation will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_be:   1;                         /*Block erase enable(32KB) .  Block erase operation will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_se:   1;                         /*Sector erase enable(4KB). Sector erase operation will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_pp:   1;                         /*Page program enable(1 byte ~256 bytes data to be programmed). Page program operation  will be triggered when the bit is set. The bit will be cleared once the operation done .1: enable 0: disable.*/
            uint32_t flash_wrsr: 1;                         /*Write status register enable.   Write status operation  will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_rdsr: 1;                         /*Read status register-1.  Read status operation will be triggered when the bit is set. The bit will be cleared once the operation done.1: enable 0: disable.*/
            uint32_t flash_rdid: 1;                         /*Read JEDEC ID . Read ID command will be sent when the bit is set. The bit will be cleared once the operation done. 1: enable 0: disable.*/
            uint32_t flash_wrdi: 1;                         /*Write flash disable. Write disable command will be sent when the bit is set. The bit will be cleared once the operation done. 1: enable 0: disable.*/
            uint32_t flash_wren: 1;                         /*Write flash enable.  Write enable command will be sent when the bit is set. The bit will be cleared once the operation done. 1: enable 0: disable.*/
            uint32_t flash_read: 1;                         /*Read flash enable. Read flash operation will be triggered when the bit is set. The bit will be cleared once the operation done. 1: enable 0: disable.*/
        };
        uint32_t val;
    } cmd;
    uint32_t addr;                                          /*addr to slave / from master. SPI transfer from the MSB to the LSB. If length > 32 bits, then address continues from MSB of wr_status.*/
    union {
        struct {
            uint32_t clkcnt_l:           4;
            uint32_t clkcnt_h:           4;
            uint32_t clkcnt_n:           4;
            uint32_t clk_equ_sysclk:     1;
            uint32_t fastrd_mode:        1;                 /*This bit enable the bits: spi_fread_qio  spi_fread_dio  spi_fread_qout and spi_fread_dout. 1: enable 0: disable.*/
            uint32_t fread_dual:         1;                 /*In the read operations  read-data phase apply 2 signals. 1: enable 0: disable.*/
            uint32_t reserved15:         5;                 /*reserved*/
            uint32_t fread_quad:         1;                 /*In the read operations read-data phase apply 4 signals. 1: enable 0: disable.*/
            uint32_t reserved21:         2;                 /*reserved*/
            uint32_t fread_dio:          1;                 /*In the read operations address phase and read-data phase apply 2 signals. 1: enable 0: disable.*/
            uint32_t fread_qio:          1;                 /*In the read operations address phase and read-data phase apply 4 signals. 1: enable 0: disable.*/
            uint32_t rd_bit_order:       1;                 /*In read-data (MISO) phase 1: LSB first 0: MSB first*/
            uint32_t wr_bit_order:       1;                 /*In command address write-data (MOSI) phases 1: LSB firs 0: MSB first*/
            uint32_t reserved27:         5;                 /*reserved*/
        };
        uint32_t val;
    } ctrl;
    union {
        struct {
            uint32_t reserved0:         16;                 /*reserved*/
            uint32_t cs_hold_delay_res: 12;                 /*Delay cycles of resume Flash when resume Flash is enable by spi clock.*/
            uint32_t cs_hold_delay:      4;                 /*SPI cs signal is delayed by spi clock cycles*/
        };
        uint32_t val;
    } ctrl1;
    union {
        struct {
            uint32_t status:    16;                         /*In the slave mode, it is the status for master to read out.*/
            uint32_t wb_mode:    8;                         /*Mode bits in the flash fast read mode, it is combined with spi_fastrd_mode bit.*/
            uint32_t status_ext: 8;                         /*In the slave mode,it is the status for master to read out.*/
        };
        uint32_t val;
    } rd_status;                                     /*In the slave mode, this register are the status register for the master to read out.*/
    union {
        struct {
            uint32_t reserved0:       16;                   /*reserved*/
            uint32_t miso_delay_mode:  2;                   /*MISO signals are delayed by spi_clk. 0: zero  1: if spi_ck_out_edge or spi_ck_i_edge is set 1  delayed by half cycle    else delayed by one cycle  2: if spi_ck_out_edge or spi_ck_i_edge is set 1  delayed by one cycle  else delayed by half cycle  3: delayed one cycle*/
            uint32_t miso_delay_num:   3;                   /*MISO signals are delayed by system clock cycles*/
            uint32_t mosi_delay_mode:  2;                   /*MOSI signals are delayed by spi_clk. 0: zero  1: if spi_ck_out_edge or spi_ck_i_edge is set 1  delayed by half cycle    else delayed by one cycle  2: if spi_ck_out_edge or spi_ck_i_edge is set 1  delayed by one cycle  else delayed by half cycle  3: delayed one cycle*/
            uint32_t mosi_delay_num:   3;                   /*MOSI signals are delayed by system clock cycles*/
            uint32_t cs_delay_mode:    2;                   /*spi_cs signal is delayed by spi_clk . 0: zero  1: if spi_ck_out_edge or spi_ck_i_edge is set 1  delayed by half cycle    else delayed by one cycle  2: if spi_ck_out_edge or spi_ck_i_edge is set 1  delayed by one cycle   else delayed by half cycle  3: delayed one cycle*/
            uint32_t cs_delay_num:     4;                   /*spi_cs signal is delayed by system clock cycles*/
        };
        uint32_t val;
    } ctrl2;
    union {
        struct {
            uint32_t clkcnt_l:       6;                     /*In the master mode it must be equal to spi_clkcnt_n. In the slave mode it must be 0.*/
            uint32_t clkcnt_h:       6;                     /*In the master mode it must be floor((spi_clkcnt_n+1)/2-1). In the slave mode it must be 0.*/
            uint32_t clkcnt_n:       6;                     /*In the master mode it is the divider of spi_clk. So spi_clk frequency is system/(spi_clkdiv_pre+1)/(spi_clkcnt_n+1)*/
            uint32_t clkdiv_pre:    13;                     /*In the master mode it is pre-divider of spi_clk.*/
            uint32_t clk_equ_sysclk: 1;                     /*In the master mode 1: spi_clk is eqaul to system 0: spi_clk is divided from system clock.*/
        };
        uint32_t val;
    } clock;
    union {
        struct {
            uint32_t duplex:            1;                  
            uint32_t reserved1:         1;                  /*reserved*/
            uint32_t flash_mode:        1;
            uint32_t reserved3:         1;                  /*reserved*/
            uint32_t cs_hold:           1;                  /*spi cs keep low when spi is in done phase. 1: enable 0: disable.*/
            uint32_t cs_setup:          1;                  /*spi cs is enable when spi is in prepare phase. 1: enable 0: disable.*/
            uint32_t ck_i_edge:         1;                  /*In the slave mode  the bit is same as spi_ck_out_edge in master mode. It is combined with  spi_miso_delay_mode bits.*/
            uint32_t ck_out_edge:       1;                  /*the bit combined with spi_mosi_delay_mode bits to set mosi signal delay mode.*/
            uint32_t reserved8:         2;                  /*reserved*/
            uint32_t rd_byte_order:     1;                  /*In read-data (MISO) phase 1: big-endian 0: little_endian*/
            uint32_t wr_byte_order:     1;                  /*In command address write-data (MOSI) phases 1: big-endian 0: litte_endian*/
            uint32_t fwrite_dual:       1;                  /*In the write operations write-data phase apply 2 signals*/
            uint32_t fwrite_quad:       1;                  /*In the write operations write-data phase apply 4 signals*/
            uint32_t fwrite_dio:        1;                  /*In the write operations address phase and write-data phase apply 2 signals.*/
            uint32_t fwrite_qio:        1;                  /*In the write operations address phase and write-data phase apply 4 signals.*/
            uint32_t sio:               1;                  /*Set the bit to enable 3-line half duplex communication  mosi and miso signals share the same pin. 1: enable 0: disable.*/
            uint32_t reserved17:        7;                  /*reserved*/
            uint32_t usr_miso_highpart: 1;                  /*read-data phase only access to high-part of the buffer spi_w8~spi_w15. 1: enable 0: disable.*/
            uint32_t usr_mosi_highpart: 1;                  /*write-data phase only access to high-part of the buffer spi_w8~spi_w15. 1: enable 0: disable.*/
            uint32_t reserved26:        1;                  /*reserved*/
            uint32_t usr_mosi:          1;                  /*This bit enable the write-data phase of an operation.*/
            uint32_t usr_miso:          1;                  /*This bit enable the read-data phase of an operation.*/
            uint32_t usr_dummy:         1;                  /*This bit enable the dummy phase of an operation.*/
            uint32_t usr_addr:          1;                  /*This bit enable the address phase of an operation.*/
            uint32_t usr_command:       1;                  /*This bit enable the command phase of an operation.*/
        };
        uint32_t val;
    } user;
    union {
        struct {
            uint32_t usr_dummy_cyclelen: 8;                 /*The length in spi_clk cycles of dummy phase. The register value shall be (cycle_num-1).*/
            uint32_t usr_miso_bitlen:    9;                 /*The length in bits of  read-data. The register value shall be (bit_num-1).*/
            uint32_t usr_mosi_bitlen:    9;                 /*The length in bits of write-data. The register value shall be (bit_num-1).*/
            uint32_t usr_addr_bitlen:    6;                 /*The length in bits of address phase. The register value shall be (bit_num-1).*/
        };
        uint32_t val;
    } user1;
    union {
        struct {
            uint32_t usr_command_value: 16;                 /*The value of  command. Output sequence: bit 7-0 and then 15-8.*/
            uint32_t reserved16:        12;                 /*reserved*/
            uint32_t usr_command_bitlen: 4;                 /*The length in bits of command phase. The register value shall be (bit_num-1)*/
        };
        uint32_t val;
    } user2;
    uint32_t wr_status;                                 /*In the slave mode this register are the status register for the master to write into. In the master mode this register are the higher 32bits in the 64 bits address condition.*/
    union {
        struct {
            uint32_t cs0_dis:        1;                     /*SPI CS0 pin enable, 1: disable CS0, 0: spi_cs0 signal is from/to CS0 pin*/
            uint32_t cs1_dis:        1;                     /*SPI CS1 pin enable, 1: disable CS1, 0: spi_cs1 signal is from/to CS1 pin*/
            uint32_t cs2_dis:        1;                     /*SPI CS2 pin enable, 1: disable CS2, 0: spi_cs2 signal is from/to CS2 pin*/
            uint32_t reserved3:     16;                     /*reserved*/
            uint32_t slave_mode:     1;                     /*1: Both CLK and CS are input, 0: Both CLK and CS are output*/
            uint32_t reserved20:     9;                     /*reserved*/
            uint32_t ck_idle_edge:   1;                     /*1: spi clk line is high when idle     0: spi clk line is low when idle*/
            uint32_t reserved30:     2;                     /*reserved*/
        };
        uint32_t val;
    } pin;
    union {
        struct {
            uint32_t rd_buf_done:  1;                       /*The interrupt raw bit for the completion of read-buffer operation in the slave mode.*/
            uint32_t wr_buf_done:  1;                       /*The interrupt raw bit for the completion of write-buffer operation in the slave mode.*/
            uint32_t rd_sta_done:  1;                       /*The interrupt raw bit for the completion of read-status operation in the slave mode.*/
            uint32_t wr_sta_done:  1;                       /*The interrupt raw bit for the completion of write-status operation in the slave mode.*/
            uint32_t trans_done:   1;                       /*The interrupt raw bit for the completion of any operation in both the master mode and the slave mode.*/
            uint32_t rd_buf_inten: 1;                       /*The interrupt enable bit for the completion of read-buffer operation in the slave mode.*/
            uint32_t wr_buf_inten: 1;                       /*The interrupt enable bit for the completion of write-buffer operation in the slave mode.*/
            uint32_t rd_sta_inten: 1;                       /*The interrupt enable bit for the completion of read-status operation in the slave mode.*/
            uint32_t wr_sta_inten: 1;                       /*The interrupt enable bit for the completion of write-status operation in the slave mode.*/
            uint32_t trans_inten:  1;                       /*The interrupt enable bit for the completion of any operation in both the master mode and the slave mode.*/
            uint32_t reserved10:  13;                       /*reserved*/
            uint32_t trans_cnt:    4;                       /*The operations counter in both the master mode and the slave mode.*/
            uint32_t cmd_define:   1;                       /*1: slave mode commands are defined in SPI_SLAVE3.  0: slave mode commands are fixed as: 1: write-status 2: write-buffer and 3: read-buffer 4: read-status*/
            uint32_t wr_rd_sta_en: 1;                       /*write and read status enable  in the slave mode*/
            uint32_t wr_rd_buf_en: 1;                       /*write and read buffer enable in the slave mode*/
            uint32_t slave_mode:   1;                       /*1: slave mode 0: master mode.*/
            uint32_t sync_reset:   1;                       /*Software reset enable, reset the spi clock line cs line and data lines.*/
        };
        uint32_t val;
    } slave;
    union {
        struct {
            uint32_t rdbuf_dummy_en:  1;                    /*In the slave mode it is the enable bit of dummy phase for read-buffer operations.*/
            uint32_t wrbuf_dummy_en:  1;                    /*In the slave mode it is the enable bit of dummy phase for write-buffer operations.*/
            uint32_t rdsta_dummy_en:  1;                    /*In the slave mode it is the enable bit of dummy phase for read-status operations.*/
            uint32_t wrsta_dummy_en:  1;                    /*In the slave mode it is the enable bit of dummy phase for write-status operations.*/
            uint32_t wr_addr_bitlen:  6;                    /*In the slave mode it is the address length in bits for write-buffer operation. The register value shall be (bit_num-1).*/
            uint32_t rd_addr_bitlen:  6;                    /*In the slave mode it is the address length in bits for read-buffer operation. The register value shall be (bit_num-1).*/
            uint32_t buf_bitlen:      9;                    /*In the slave mode it is the length of buffer bit.*/
            uint32_t status_readback: 1;                    /*In the slave mode it is the bit decide whether master reads rd_status register or wr_status register data. 0: rd_status: 1: wr_status*/
            uint32_t reserved25:      1;                    /*reserved*/
            uint32_t status_bitlen:   5;                    /*In the slave mode it is the length of status bit.*/
        };
        uint32_t val;
    } slave1;
    union {
        struct {
            uint32_t rdsta_dummy_cyclelen: 8;               /*In the slave mode it is the length in spi_clk cycles of dummy phase for read-status operations. The register value shall be (cycle_num-1).*/
            uint32_t wrsta_dummy_cyclelen: 8;               /*In the slave mode it is the length in spi_clk cycles of dummy phase for write-status operations. The register value shall be (cycle_num-1).*/
            uint32_t rdbuf_dummy_cyclelen: 8;               /*In the slave mode it is the length in spi_clk cycles of dummy phase for read-buffer operations. The register value shall be (cycle_num-1).*/
            uint32_t wrbuf_dummy_cyclelen: 8;               /*In the slave mode it is the length in spi_clk cycles of dummy phase for write-buffer operations. The register value shall be (cycle_num-1).*/
        };
        uint32_t val;
    } slave2;
    union {
        struct {
            uint32_t rdbuf_cmd_value: 8;                    /*In the slave mode it is the value of read-buffer command.*/
            uint32_t wrbuf_cmd_value: 8;                    /*In the slave mode it is the value of write-buffer command.*/
            uint32_t rdsta_cmd_value: 8;                    /*In the slave mode it is the value of read-status command.*/
            uint32_t wrsta_cmd_value: 8;                    /*In the slave mode it is the value of write-status command.*/
        };
        uint32_t val;
    } slave3;
    uint32_t data_buf[16];                                  /*data buffer*/
    uint32_t reserved_80[30];
    uint32_t ext2;
    union {
        struct {
            uint32_t int_hold_ena: 2;                       /*This register is for two SPI masters to share the same cs clock and data signals. The bits of one SPI are set  if the other SPI is busy  the SPI will be hold. 1(3): hold at ,idle, phase 2: hold at ,prepare, phase.*/
            uint32_t reserved2:   30;                       /*reserved*/
        };
        uint32_t val;
    } ext3;
} spi_dev_t;

extern volatile spi_dev_t SPI0;

#define SPI_FLASH       SPI0
#define SPI_BLOCK_SIZE  32
#define ADDR_SHIFT_BITS 8

#if 1
typedef int (*__ets_printf_t)(const char *fmt, ...); 
#define ROM_PRINTF(_fmt, ...)   ((__ets_printf_t)(0x400024cc))(_fmt, ##__VA_ARGS__)
#else
#define ROM_PRINTF(_fmt, ...)
#endif

#define TOCHAR(_v)              #_v
#define PRINT_STEP(_s)          ROM_PRINTF("Step %d\n", (_s));
#define JUMP_TO_STEP(_s)        { ROM_PRINTF("Jump to " TOCHAR(_s) "\n"); goto _s; }
#define GOTO_FAILED(_s)         { ROM_PRINTF("ERROR: " TOCHAR(_s) " failed\n"); ret = -EIO; JUMP_TO_STEP(step17); }

#define write_u8_dummy(_c, _a, _d8,_d)          {uint32_t __data = _d8; spi_trans(1, (_c), 8, (_a), 24, (uint8_t *)&__data, 1, (_d));}
#define write_u8(_c, _a, _d8)                   write_u8_dummy((_c), (_a), (_d8), 0)

extern void Cache_Read_Disable_2(void);
extern void Cache_Read_Enable_2();
extern uint32_t spi_flash_get_id(void);

extern SpiFlashChip* flashchip;

#define ICACHE_PATCH_ATTR __attribute__((section(".patch.text")))

static void ICACHE_PATCH_ATTR delay(int ms)
{
    for (volatile int i = 0; i < ms; i++) {
        for (volatile int j = 0; j < 7800; j++) {
        }
    }
}

#if 0
static void dump_hex(const uint8_t *ptr, int n)
{
  const uint8_t *s1 = ptr;
  const int line_bytes = 16;

  ROM_PRINTF("\nHex:\n");
  for (int i = 0; i < n ; i += line_bytes)
    {
       int m = MIN(n - i, line_bytes);

      ROM_PRINTF("\t");
      for (int j = 0; j < m; j++)
        {
          ROM_PRINTF("%02x ", s1[i + j]);
        }
      
      ROM_PRINTF("\n");
    }

  ROM_PRINTF("\n");
}

static void dump_hex_compare(const uint8_t *s1, const uint8_t *s2, int n)
{
  const int line_bytes = 16;

  ROM_PRINTF("\nHex:\n");
  for (int i = 0; i < n ; i += line_bytes)
    {
       int m = MIN(n - i, line_bytes);

      ROM_PRINTF("\t");
      for (int j = 0; j < m; j++)
        {
          ROM_PRINTF("%02x:%02x ", s1[i + j], s2[i + j]);
        }
      
      ROM_PRINTF("\n");
    }

  ROM_PRINTF("\n");
}
#endif

typedef struct spi_state {
    uint32_t io_mux_reg;
    uint32_t spi_clk_reg;
    uint32_t spi_ctrl_reg;
    uint32_t spi_user_reg;
} spi_state_t;

static void ICACHE_PATCH_ATTR spi_enter(spi_state_t *state)
{
    // vPortEnterCritical();
    ets_intr_lock();
    // Cache_Read_Disable_2();

    Wait_SPI_Idle(flashchip);

    state->io_mux_reg = READ_PERI_REG(PERIPHS_IO_MUX_CONF_U);
    state->spi_clk_reg = SPI_FLASH.clock.val;
    state->spi_ctrl_reg = SPI_FLASH.ctrl.val;
    state->spi_user_reg = SPI_FLASH.user.val;

    SPI_FLASH.user.usr_command = 1;
    SPI_FLASH.user.flash_mode = 0;
    SPI_FLASH.user.usr_miso_highpart = 0;

    CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI0_CLK_EQU_SYS_CLK);

    SPI_FLASH.user.cs_setup = 1;
    SPI_FLASH.user.cs_hold = 1;
    SPI_FLASH.user.usr_mosi = 1;

    SPI_FLASH.user.usr_command = 1;
    SPI_FLASH.user.flash_mode = 0;

    SPI_FLASH.ctrl.fread_qio = 0;
    SPI_FLASH.ctrl.fread_dio = 0;
    SPI_FLASH.ctrl.fread_quad = 0;
    SPI_FLASH.ctrl.fread_dual = 0;

    SPI_FLASH.clock.val = 0;
    SPI_FLASH.clock.clkcnt_l = 3;
    SPI_FLASH.clock.clkcnt_h = 1;
    SPI_FLASH.clock.clkcnt_n = 3;

    SPI_FLASH.ctrl.fastrd_mode = 1;

    while (SPI_FLASH.cmd.usr) {
        ;
    }
}

static void ICACHE_PATCH_ATTR spi_exit(spi_state_t *state)
{
    while (SPI_FLASH.cmd.usr) {
        ;
    }

    WRITE_PERI_REG(PERIPHS_IO_MUX_CONF_U, state->io_mux_reg);

    SPI_FLASH.ctrl.val = state->spi_ctrl_reg;
    SPI_FLASH.clock.val = state->spi_clk_reg;
    SPI_FLASH.user.val = state->spi_user_reg;

    // Cache_Read_Enable_2();
    // vPortExitCritical();
    ets_intr_unlock();
}

static void ICACHE_PATCH_ATTR spi_trans_block(bool write_mode,
                            uint32_t cmd,
                            uint32_t cmd_bits,
                            uint32_t addr,
                            uint32_t addr_bits,
                            uint8_t *data,
                            uint32_t data_bytes,
                            uint32_t dummy_bits)
{
    if ((uint32_t)data & 0x3) {
        ROM_PRINTF("ERROR: data=%p\n", data);
        return;
    }

    if (cmd_bits) {
        SPI_FLASH.user.usr_command = 1;
        SPI_FLASH.user2.usr_command_value = cmd;
        SPI_FLASH.user2.usr_command_bitlen = cmd_bits - 1;
    } else {
        SPI_FLASH.user.usr_command = 0;
    }

    if (dummy_bits) {
        SPI_FLASH.user.usr_dummy = 1;
        SPI_FLASH.user1.usr_dummy_cyclelen = dummy_bits - 1;
    } else {
        SPI_FLASH.user.usr_dummy = 0;
    }
    // SPI_FLASH.user.usr_dummy = 0;

    if (addr_bits) {
        SPI_FLASH.user.usr_addr = 1;
        SPI_FLASH.addr = addr << ADDR_SHIFT_BITS;
        SPI_FLASH.user1.usr_addr_bitlen = addr_bits - 1;
    } else {
        SPI_FLASH.user.usr_addr = 0;
    }

    if (write_mode && data_bytes) {
        int words = (data_bytes + 3) / 4;
        uint32_t *p = (uint32_t *)data;

        SPI_FLASH.user.usr_mosi = 1;
        SPI_FLASH.user.usr_miso = 0;
        SPI_FLASH.user1.usr_mosi_bitlen = data_bytes * 8 - 1;

        for (int i = 0; i < words; i++) {
            SPI_FLASH.data_buf[i] = p[i];
        }
    } else if (!write_mode && data_bytes) {
        int words = (data_bytes + 3) / 4;

        SPI_FLASH.user.usr_mosi = 0;
        SPI_FLASH.user.usr_miso = 1;
        SPI_FLASH.user1.usr_miso_bitlen = data_bytes * 8 - 1;

        for (int i = 0; i < words; i++) {
            SPI_FLASH.data_buf[i] = 0;
        }
    } else {
        SPI_FLASH.user.usr_mosi = 0;
        SPI_FLASH.user.usr_miso = 0;
    }

    SPI_FLASH.cmd.usr = 1;
    while (SPI_FLASH.cmd.usr) {
        ;
    }

    if (!write_mode && data_bytes) {
        int words = (data_bytes + 3) / 4;
        uint32_t *p = (uint32_t *)data;

        for (int i = 0; i < words; i++) {
            p[i] = SPI_FLASH.data_buf[i];
        }
    }
}

static void ICACHE_PATCH_ATTR spi_trans(bool write_mode,
                      uint32_t cmd,
                      uint32_t cmd_bits,
                      uint32_t addr,
                      uint32_t addr_bits,
                      uint8_t *data,
                      uint32_t data_bytes,
                      uint32_t dummy_bits)

{
    if (!data_bytes || data_bytes <= SPI_BLOCK_SIZE) {
        return spi_trans_block(write_mode, cmd, cmd_bits, addr,
                               addr_bits, data, data_bytes, dummy_bits);
    }

    for (int i = 0; i < data_bytes; i += SPI_BLOCK_SIZE) {
        uint32_t n = MIN(SPI_BLOCK_SIZE, data_bytes - i);

        spi_trans_block(write_mode, cmd, cmd_bits, addr + i,
                        addr_bits, data + i, n, dummy_bits);
    }
}

static void ICACHE_PATCH_ATTR write_cmd(uint32_t cmd)
{
    spi_trans(1, cmd, 8, 0, 0, NULL, 0, 0);
}

static void ICACHE_PATCH_ATTR write_buffer(uint32_t addr, uint8_t *buffer, int size)
{
    for (int i = 0; i < size; i += SPI_BLOCK_SIZE) {
        int n = MIN(size - i, SPI_BLOCK_SIZE);

        write_cmd(0x6);
        spi_trans(1, 0x42, 8, addr + i, 24, buffer + i, n, 0);
        delay(3);
    }
}

static void ICACHE_PATCH_ATTR read_buffer(uint32_t addr, uint8_t *buffer, int n)
{
    spi_trans(0, 0x48, 8, addr, 24, buffer, n, 8);
}

static void ICACHE_PATCH_ATTR erase_sector(uint32_t addr)
{
    write_cmd(0x6);
    spi_trans(1, 0x44, 8, addr, 24, NULL, 0, 0);
    delay(8);
}

int ICACHE_PATCH_ATTR th25q16hb_apply_patch_0(uint8_t *buffer1024)
{
    int ret = 0;
    uint32_t flash_id;
    int count;
    spi_state_t state;
    uint8_t *buffer256_0;
    uint8_t *buffer256_1;
    uint8_t *buffer256_2;
    // uint8_t *buffer1024;

    // flash_id = spi_flash_get_id();
    // if (flash_id != 0x1560eb) {
    //     ROM_PRINTF("WARN: id=0x%x, is not TH25Q16HB\n", flash_id);
    //     return 0;
    // }

    // buffer1024 = os_malloc(1024);
    // if (!buffer1024) {
    //     return -ENOMEM;
    // }

    buffer256_0 = buffer1024;
    buffer256_1 = buffer1024 + 256;
    buffer256_2 = buffer1024 + 512;
    spi_enter(&state);

    // Step 1
    PRINT_STEP(1);
    write_cmd(0x33);

    // Step 2
    PRINT_STEP(2);
    write_cmd(0xcc);
    
    // Step 3
    PRINT_STEP(3);
    write_cmd(0xaa);

    // Step 4
    PRINT_STEP(4);
    write_u8(0xfa, 0x1200d, 0x3);
    write_u8_dummy(0xfa, 0x2200d, 0x3, 1);

    // Step 5-1
    PRINT_STEP(5);
    read_buffer(0xbed, buffer256_0, 3);

    if (buffer256_0[0] == 0xff &&
        buffer256_0[1] == 0xff &&
        buffer256_0[2] == 0xff) {
        ROM_PRINTF("INFO: check done 0\n");
    } else if (buffer256_0[0] == 0x55 &&
               buffer256_0[1] == 0xff &&
               buffer256_0[2] == 0xff) {
        JUMP_TO_STEP(step10);
    } else if (buffer256_0[0] == 0x55 &&
               buffer256_0[1] == 0x55 &&
               buffer256_0[2] == 0xff) {
        JUMP_TO_STEP(step14);
    } else if (buffer256_0[0] == 0x55 &&
               buffer256_0[1] == 0x55 &&
               buffer256_0[2] == 0x55) {
        JUMP_TO_STEP(step17);
    } else {
        ROM_PRINTF("ERROR: 0xbed=0x%x 0xbee=0x%x 0xbef=0x%x\n",
                    buffer256_0[0], buffer256_0[1], buffer256_0[2]);
        GOTO_FAILED(5-1);
    }
#if 1
    // Step 5-2
    read_buffer(0x50d, buffer256_0, 1);
    buffer256_0[0] &= 0x7f;
    if (buffer256_0[0] == 0x7c) {
        JUMP_TO_STEP(step17);
    } else if (buffer256_0[0] == 0x3c) {
        ROM_PRINTF("INFO: check done 1\n");
    } else {
        ROM_PRINTF("ERROR: 0x50d=0x%x\n", buffer256_0[0]);
        GOTO_FAILED(5-2);
    }

    // Step 6
    PRINT_STEP(6);
    for (count = 0; count < 3; count++) {
        erase_sector(0);

        bool check_done = true;
        read_buffer(0x0, buffer1024, 1024);
        for (int i = 0; i < 1024; i++) {
            if (buffer1024[i] != 0xff) {
                check_done = false;
                ROM_PRINTF("ERROR: buffer1024[%d]=0x%x\n", i, buffer1024[i]);
                break;
            }
        }

        if (check_done) {
            break;
        }
    }
    if (count >= 3) {
        GOTO_FAILED(6)
    }

    // Step 7-1.1
    PRINT_STEP(7);
    read_buffer(0x400, buffer256_0, 256);
    read_buffer(0x400, buffer256_1, 256);
    read_buffer(0x400, buffer256_2, 256);
    if (os_memcmp(buffer256_0, buffer256_1, 256) ||
        os_memcmp(buffer256_0, buffer256_2, 256)) {
        GOTO_FAILED(7-1.1);
    }

    write_buffer(0, buffer256_0, 256);
    write_buffer(0x200, buffer256_0, 256);

    // Step 7-1.2
    for (count = 0; count < 3; count++) {
        read_buffer(0, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }

        read_buffer(0x200, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(7-1.2);
    }

    // Step 7-2.1
    read_buffer(0x500, buffer256_0, 256);
    read_buffer(0x500, buffer256_1, 256);
    read_buffer(0x500, buffer256_2, 256);
    if (os_memcmp(buffer256_0, buffer256_1, 256) ||
        os_memcmp(buffer256_0, buffer256_2, 256)) {
        GOTO_FAILED(7-2.1);
    }
    write_buffer(0x100, buffer256_0, 256);
    write_buffer(0x300, buffer256_0, 256);

    // Step 7-2.2
    for (count = 0; count < 3; count++) {
        read_buffer(0x100, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }
        read_buffer(0x300, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(7-2.2);
    }

    // Step 8
    PRINT_STEP(8);
    read_buffer(0x0, buffer256_0, 1);
    read_buffer(0x23, buffer256_1, 1);
    if (buffer256_0[0] != 0x13 || buffer256_1[0] != 0x14) {
        ROM_PRINTF("ERROR: 0x0=0x%x 0x23=0x%x\n", buffer256_0[0], buffer256_1[0]);
        GOTO_FAILED(8);
    }

    // Step 9-1
    PRINT_STEP(9);
    read_buffer(0x140, buffer256_0, 2);
    if (buffer256_0[0] != 0 || buffer256_0[1] != 0xff) {
        ROM_PRINTF("ERROR: 0x140=0x%x 0x141=0x%x\n", buffer256_0[0], buffer256_0[1]);
        GOTO_FAILED(9-1);
    }

    // Step 9-2
    buffer256_0[0] = 0x55;
    write_buffer(0xaed, buffer256_0, 1);
    write_buffer(0xbed, buffer256_0, 1);
    for (count = 0; count < 3; count++) {
        read_buffer(0xaed, buffer256_0, 1);
        if (buffer256_0[0] != 0x55) {
            break;
        }

        read_buffer(0xbed, buffer256_0, 1);
        if (buffer256_0[0] != 0x55) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(9-2);
    }
#endif
step10:
    // Step 10-1
    PRINT_STEP(10);
    for (count = 0; count < 3; count++) {
        erase_sector(0x400);

        bool check_done = true;
        read_buffer(0x400, buffer1024, 1024);
        for (int i = 0; i < 1024; i++) {
            if (buffer1024[i] != 0xff) {
                check_done = false;
                ROM_PRINTF("ERROR: buffer1024[%d]=0x%x\n", i, buffer1024[i]);
                break;
            }
        }

        if (check_done) {
            break;
        }
    }
    if (count >= 3) {
        GOTO_FAILED(10-1);
    }

    // Step 10-3.1
    read_buffer(0, buffer256_0, 256);
    read_buffer(0, buffer256_1, 256);
    read_buffer(0, buffer256_2, 256);
    if (os_memcmp(buffer256_0, buffer256_1, 256) ||
        os_memcmp(buffer256_0, buffer256_2, 256)) {
        GOTO_FAILED(10-3.1);
    }
    write_buffer(0x400, buffer256_0, 256);
    write_buffer(0x600, buffer256_0, 256);

    // Step 10-3.2
    for (count = 0; count < 3; count++) {
        read_buffer(0x400, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }

        read_buffer(0x600, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(10-3.2);
    }    

    // Step 10-3.3
    read_buffer(0x100, buffer256_0, 256);
    read_buffer(0x100, buffer256_1, 256);
    read_buffer(0x100, buffer256_2, 256);
    if (os_memcmp(buffer256_0, buffer256_1, 256) ||
        os_memcmp(buffer256_0, buffer256_2, 256)) {
        GOTO_FAILED(10-3.3);
    }
    buffer256_0[9] = 0x79;
    buffer256_0[13] = (buffer256_0[13] & 0x3f) |
                       ((~buffer256_0[13]) & 0x80) |
                       0x40;
    write_buffer(0x500, buffer256_0, 256);
    write_buffer(0x700, buffer256_0, 256);

    // Step 10-3.4
    for (count = 0; count < 3; count++) {
        read_buffer(0x500, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }

        read_buffer(0x700, buffer256_1, 256);
        if (os_memcmp(buffer256_0, buffer256_1, 256)) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(10-3.4);
    } 

    // Step11-1
    PRINT_STEP(11);

    for (count = 0; count < 3; count++) {
        read_buffer(0x400, buffer256_0, 1);
        read_buffer(0x423, buffer256_1, 1);
        if (buffer256_0[0] != 0x13 || buffer256_1[0] != 0x14) {
            ROM_PRINTF("ERROR: 0x400=0x%x 0x423=0x%x\n", buffer256_0[0], buffer256_1[0]);
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(11-1);
    }

    // Step11-2.1
    for (count = 0; count < 3; count++) {
        read_buffer(0x540, buffer256_0, 2);
        if (buffer256_0[0] != 0 || buffer256_0[1] != 0xff) {
            ROM_PRINTF("ERROR: 0x540=0x%x 0x541=0x%x\n", buffer256_0[0], buffer256_0[1]);
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(11-2);
    }

    // Step11-2.2
    for (count = 0; count < 3; count++) {
        read_buffer(0x50d, buffer256_0, 1);
        buffer256_0[0] &= 0x7f;
        if (buffer256_0[0] != 0x7c) {
            ROM_PRINTF("ERROR: 0x50d=0x%x\n", buffer256_0[0]);
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(11-2);
    }

    // Step 12
    PRINT_STEP(12);
    buffer256_0[0] = 0x55;
    write_buffer(0xaee, buffer256_0, 1);
    write_buffer(0xbee, buffer256_0, 1);

    // Step 13
    PRINT_STEP(13);
    for (count = 0; count < 3; count++) {
        read_buffer(0xaee, buffer256_0, 1);
        if (buffer256_0[0] != 0x55) {
            break;
        }

        read_buffer(0xbee, buffer256_0, 1);
        if (buffer256_0[0] != 0x55) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(13);
    }

step14:
    // Step 14
    PRINT_STEP(14);
    for (count = 0; count < 3; count++) {
        erase_sector(0);

        bool check_done = true;
        read_buffer(0x0, buffer1024, 1024);
        for (int i = 0; i < 1024; i++) {
            if (buffer1024[i] != 0xff) {
                check_done = false;
                ROM_PRINTF("ERROR: buffer1024[%d]=0x%x\n", i, buffer1024[i]);
                break;
            }
        }

        if (check_done) {
            break;
        }
    }
    if (count >= 3) {
        GOTO_FAILED(14);
    }    

    // Step 15
    PRINT_STEP(15);
    buffer256_0[0] = 0x55;
    write_buffer(0xaef, buffer256_0, 1);
    write_buffer(0xbef, buffer256_0, 1);

    // Step 16
    PRINT_STEP(16);
    for (count = 0; count < 3; count++) {
        read_buffer(0xaef, buffer256_0, 1);
        if (buffer256_0[0] != 0x55) {
            break;
        }

        read_buffer(0xbef, buffer256_0, 1);
        if (buffer256_0[0] != 0x55) {
            break;
        }
    }
    if (count < 3) {
        GOTO_FAILED(16);
    }

step17:
    // Step 17
    PRINT_STEP(17);
    write_cmd(0x55);

    // Step 18
    PRINT_STEP(18);
    write_cmd(0x88);

    spi_exit(&state);

    // os_free(buffer1024);

    if (!ret) {
        ROM_PRINTF("INFO: Patch for TH25Q16HB is done\n");
    }

    return ret;
}

int __attribute__((section(".PatchEnter.text"))) call_user_start1(uint8_t *buffer1024)
{
    // ets_printf("call_user_start\r\n");
    return th25q16hb_apply_patch_0(buffer1024);
    // return 0;
}