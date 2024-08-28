/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rtthread.h>
#include <rthw.h>

#include <dfs_fs.h>
#include <ioremap.h>

#include <msh.h>

#ifdef ENABLE_CHERRY_USB

#ifdef ENABLE_CHERRY_USB_DEVICE
#include "usbd_core.h"
#endif // ENABLE_CHERRY_USB_DEVICE

#ifdef ENABLE_CHERRY_USB_HOST
#include "usbh_core.h"
#endif // ENABLE_CHERRY_USB_HOST

#if defined(ENABLE_CHERRY_USB_DEVICE) && defined (ENABLE_CHERRY_USB_HOST)
  #if CHERRY_USB_DEVICE_USING_DEV + CHERRY_USB_HOST_USING_DEV != 1
    #error "Can not set same usb device as device and host"
  #endif
#endif

#endif // ENABLE_CHERRY_USB

#ifdef RT_USING_SDIO

#define TO_STRING(x) #x
#define CONCAT(x, y, z) x y z

#define SD_DEV_PART(s, d, p) CONCAT(s, TO_STRING(d), p)

static const struct dfs_mount_tbl custom_mount_table[] = {
  {SD_DEV_PART("sd", SDCARD_ON_SDIO_DEV, "0"), "/bin", "elm", 0, 0},
  {SD_DEV_PART("sd", SDCARD_ON_SDIO_DEV, "1"), "/sdcard", "elm", 0, 0},
  {SD_DEV_PART("sd", SDCARD_ON_SDIO_DEV, "2"), "/data", "elm", 0, 0},
  {0}
};

static bool s_fs_mount_data_succ = false;

static void mnt_mount_table(void)
{
    int ret;
    int index = 0;

    while (1)
    {
        if (custom_mount_table[index].path == NULL) break;

        if (0x00 != (ret = dfs_mount(custom_mount_table[index].device_name,
                      custom_mount_table[index].path,
                      custom_mount_table[index].filesystemtype,
                      custom_mount_table[index].rwflag,
                      custom_mount_table[index].data)))
        {
            rt_kprintf("mount fs[%s] on %s failed(%d), error %d.\n", custom_mount_table[index].filesystemtype,
                       custom_mount_table[index].path, ret, errno);

          if(0x00 == strcmp("/data", custom_mount_table[index].path)) {
              s_fs_mount_data_succ = false;

              if((-19) == errno) {
                rt_kprintf("Please format the partition[2] to FAT32.\nRefer to https://support.microsoft.com/zh-cn/windows/%E5%88%9B%E5%BB%BA%E5%92%8C%E6%A0%BC%E5%BC%8F%E5%8C%96%E7%A1%AC%E7%9B%98%E5%88%86%E5%8C%BA-bbb8e185-1bda-ecd1-3465-c9728f7d7d2e\n");
              }
            }
        } else {
          if(0x00 == strcmp("/data", custom_mount_table[index].path)) {
            s_fs_mount_data_succ = true;
          }
        }

        index ++;
    }
}
#endif

int main(void) {
#ifdef CONFIG_SDK_ENABLE_CANMV
  printf("CanMV Start\n");
#endif //CONFIG_SDK_ENABLE_CANMV

#ifdef RT_USING_SDIO
  while (mmcsd_wait_cd_changed(100) != MMCSD_HOST_PLUGED) {
  }
  mnt_mount_table();
#endif //RT_USING_SDIO

#ifdef ENABLE_CHERRY_USB
  void *usb_base;
  const void *usb_dev_addr[2] = {(void *)0x91500000UL, (void *)0x91540000UL};

#ifdef ENABLE_CHERRY_USB_DEVICE
  usb_base = (void *)rt_ioremap((void *)usb_dev_addr[CHERRY_USB_DEVICE_USING_DEV], 0x10000);

  extern void cdc_acm_mtp_init(void *usb_base, bool fs_data_mount_succ);
  cdc_acm_mtp_init(usb_base, s_fs_mount_data_succ);
#endif // ENABLE_CHERRY_USB_DEVICE

#ifdef ENABLE_CHERRY_USB_HOST
  usb_base = (void *)rt_ioremap((void *)usb_dev_addr[CHERRY_USB_HOST_USING_DEV], 0x10000);
  usbh_initialize(0, (uint32_t)usb_base);
#endif // ENABLE_CHERRY_USB_HOST

  rt_thread_delay(10);
#endif //ENABLE_CHERRY_USB

#ifdef CONFIG_SDK_ENABLE_CANMV
  msh_exec("/sdcard/micropython", 32);
#endif //CONFIG_SDK_ENABLE_CANMV

  return 0;
}
