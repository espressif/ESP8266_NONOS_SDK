#include "osapi.h"
#include "patch_array.h"
#include "mem.h"
#include "eagle_soc.h"

#if 0
typedef int (*__ets_printf_t)(const char *fmt, ...); 
#define ROM_PRINTF(_fmt, ...)   ((__ets_printf_t)(0x400024cc))(_fmt, ##__VA_ARGS__)
#else
#define ROM_PRINTF(_fmt, ...)   ets_printf(_fmt, ##__VA_ARGS__)
#endif

extern void call_user_start(void);
extern char _heap_start;

int patch_apply(void)
{
    uint32_t flash_id;
    int ret = __INT_MAX__;

    WRITE_PERI_REG(0x60000240, 0);
    WRITE_PERI_REG(0x60000200, 0x01 << 28);
    while(READ_PERI_REG(0x60000200) != 0);
    flash_id = READ_PERI_REG(0x60000240) & 0xffffff;
    ROM_PRINTF("flash_id=0x%x\r\n", flash_id);
    if (flash_id == 0x1560eb) {
        ROM_PRINTF("apply...\r\n");
        uint32 len = 0;
        uint8* data = patch_data + 8;
        uint32* addr = 0;

        ROM_PRINTF("data %p %d...\r\n", data, patch_data[1]);
        for (uint8 loop = 0; loop < patch_data[1] && (data - patch_data < sizeof(patch_data) - 8); loop++) {
          addr = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3]<<24);
          len = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7]<<24);
          data += 8;
          ets_memcpy(addr, data, len);
          data += len;
          ROM_PRINTF("addr=0x%x, len=%d\r\n",addr, len);
        }
        int (*apply_patch)(uint8_t *buffer1024);
        apply_patch = patch_data[4] | (patch_data[5] << 8) | (patch_data[6] << 16) | (patch_data[7]<<24);
        ROM_PRINTF("apply_patch=0x%08x...\r\n", *(uint32*)apply_patch);
        ret = apply_patch(&_heap_start);
        ROM_PRINTF("apply done...\r\n");
    }
    return ret;
}

void call_user_start1(void)
{
  int32 ret = patch_apply();
  ROM_PRINTF("ret=%d\r\n", ret);
  call_user_start();
}
