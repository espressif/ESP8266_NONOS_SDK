#ifndef ESPMISSINGINCLUDES_H
#define ESPMISSINGINCLUDES_H

/*
Copied from https://github.com/homebots/esp-open-sdk/commits/master/user_rf_cal_sector_set.c :

Suddenly introduced in SDK 1.5.4+patch/SDK 2.0.0, another callback most
users won't care about. Vendor's description is: "Set the target flash
sector to store RF_CAL parameters. The system parameter area (4 flash
sectors) has already been used, so the RF_CAL parameters will be stored
in the target sector set by user_rf_cal_sector_set." It's a mystery how
all the previous SDK version worked without a need to store this info
in flash (actually, that was one of the "selling points" of ESP8266,
setting it aside from all other chips which had RF calibration in flash),
and 2.0.0 suddenly decided to do it.

So, make this default user_rf_cal_sector_set() implementation return
flash_sectors - 5, so the last 20KB/5 sectors of flash should be
reserved for systems needs (vs 16KB/4 sectors in previous SDKs).
*/

#include <c_types.h>
#include <spi_flash.h>

uint32 user_rf_cal_sector_set(void)
{
  extern char flashchip;
  SpiFlashChip *flash = (SpiFlashChip *)(&flashchip + 4);
  uint32_t sec_num = flash->chip_size >> 12;
  return sec_num - 5;
}

// https://tech.scargill.net/home-control-2018/

void ets_isr_mask(unsigned intr);
void ets_isr_unmask(unsigned intr);

void user_rf_pre_init(void)
{
}

#endif
