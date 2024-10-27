#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <rtdef.h>
#include <rthw.h>
#include <rtthread.h>

#include <dfs.h>
#include <dfs_fs.h>
#include <dfs_posix.h>

#include "./config.h"

struct config_handler {
  uint32_t command; // const char *command;
  void (*handler)(char *value);
};

/* 0: success, -1: failed */
/* if successed, user should close(fd), and rt_free(buffer) to release resources
 */
static int read_config_file(int *fd, char **buffer, size_t *buffer_size) {
  int _fd = 0;
  size_t _buffer_size = 0;
  char *_buffer = NULL;

  _fd = open(RT_SMART_CONFIG_FILE_PATH, O_RDWR);
  if (0 > _fd) {
    goto _failed;
  }

  _buffer_size = lseek(_fd, 0, SEEK_END);
  lseek(_fd, 0, SEEK_SET);

  if (0x00 == _buffer_size) {
    close(_fd);
    goto _failed;
  }

  if (NULL == (_buffer = rt_malloc(_buffer_size))) {
    close(_fd);
    goto _failed;
  }

  if (_buffer_size != read(_fd, _buffer, _buffer_size)) {
    rt_kprintf("read config.txt failed\n");
    close(_fd);
    rt_free(buffer);
    goto _failed;
  }

  *fd = _fd;
  *buffer = _buffer;
  *buffer_size = _buffer_size;

  return 0;

_failed:
  *fd = -1;
  *buffer = NULL;
  *buffer_size = 0;
  return -1;
}

#if defined(CONFIG_RT_AUTO_RESIZE_PARTITION) && defined(RT_USING_SDIO)
// @see src/rtsmart/rtsmart/kernel/rt-thread/components/drivers/sdio/block_dev.c
struct mmcsd_blk_device {
  struct rt_mmcsd_card *card;
  rt_list_t list;
  struct rt_device dev;
  struct dfs_partition part;
  struct rt_device_blk_geometry geometry;
  rt_size_t max_req_size;
};

// MBR ////////////////////////////////////////////////////////////////////////
#define DPT_ADDRESS 0x1be /* device partition offset in Boot Sector */
#define DPT_ITEM_SIZE 16  /* partition item size */

struct partition_entry {
  u_int8_t active;
  u_int8_t start_head;
  u_int16_t start_cylsec;
  u_int8_t type;
  u_int8_t end_head;
  u_int16_t end_cylsec;
  u_int32_t lba_sector;
  u_int32_t lba_len;
};

_Static_assert(sizeof(struct partition_entry) == DPT_ITEM_SIZE,
               "Error partition entry size error");

_Static_assert(4 > CONFIG_RT_AUTO_RESIZE_PARTITION_NR,
               "MBR max partition is 4");

static int mmc_mbr_part_create_new(struct mmcsd_blk_device *blk_dev,
                                   rt_device_t dev_sd) {
  uint8_t *buffer = NULL;

  struct dfs_partition part, prev_part;
  struct partition_entry part_entry;

  const size_t sector_count = blk_dev->card->card_capacity * (1024 / 512) - 1;

  if (NULL == (buffer = rt_malloc_align(SECTOR_SIZE, RT_CPU_CACHE_LINE_SZ))) {
    rt_kprintf("%s malloc failed.\n", __func__);
    return -1;
  }

  if (0x01 != rt_device_read(dev_sd, 0, buffer, 1)) {
    rt_kprintf("%s read sector failed.\n", __func__);
    goto _exit;
  }

  if (RT_EOK != dfs_filesystem_get_partition(
                    &part, buffer, CONFIG_RT_AUTO_RESIZE_PARTITION_NR)) {
    if (RT_EOK !=
        dfs_filesystem_get_partition(&prev_part, buffer,
                                     CONFIG_RT_AUTO_RESIZE_PARTITION_NR - 1)) {
      rt_kprintf("%s get last partition info failed.\n", __func__);
      goto _exit;
    }

    rt_memset(&part_entry, 0, sizeof(part_entry));

    part_entry.active = 0x00; // not bootable
    part_entry.start_head = 0xFE;
    part_entry.start_cylsec = 0xFFFF;
    part_entry.end_head = 0xFE;
    part_entry.end_cylsec = 0xFFFF;
    part_entry.type = 0x0C; // FAT32 (LBA)
    part_entry.lba_sector = ((prev_part.offset + prev_part.size) + 0x200000) &
                            ~(0x200000 - 1); // align to 1GiB

    /* we suppose user not use sdcard bigger than 128GiB, so no need to limit
     * the lba_len  */
    part_entry.lba_len = sector_count  - part_entry.lba_sector;

    rt_memcpy(buffer + DPT_ADDRESS +
                  CONFIG_RT_AUTO_RESIZE_PARTITION_NR * DPT_ITEM_SIZE,
              &part_entry, sizeof(part_entry));

    if (0x01 == rt_device_write(dev_sd, 0, buffer, 1)) {
      rt_kprintf("Create new MBR partiton 0x%x - 0x%x\n",
                 part_entry.lba_sector * SECTOR_SIZE,
                 (part_entry.lba_sector + part_entry.lba_len) * SECTOR_SIZE);

      rt_free_align(buffer);

      return 0;
    } else {
      rt_kprintf("!!!FATAL ERROR, Update MBR FAILED.\n");
    }
  } else {
    rt_free_align(buffer);

    return 1;
  }

_exit:
  if (buffer) {
    rt_free_align(buffer);
  }

  return -1;
}

// GPT ////////////////////////////////////////////////////////////////////////
# define ENABLE_GPT_PART_RESIZE   0

#if defined (ENABLE_GPT_PART_RESIZE) && ENABLE_GPT_PART_RESIZE

#include <drivers/gpt.h>

#define GPT_ENTRIES 		128
#define GPT_SECTORS		(1 + GPT_ENTRIES * sizeof(gpt_entry) / 512)

_Static_assert(sizeof(gpt_entry) == 128, "gpt_entry size error.");

uint32_t gpt_crc32_next(const void *data, size_t len, uint32_t last_crc)
{
    uint32_t crc = ~last_crc;
    const unsigned char *p = (const unsigned char *)data;

    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)p[i]; // XOR the byte with the CRC

        for (int j = 0; j < 8; ++j) { // Process each bit
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320; // Polynomial for CRC-32
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc; // Final CRC value
}

uint32_t gpt_crc32(const void *data, size_t len)
{
	return gpt_crc32_next(data, len, 0);
}

static int mmc_gpt_part_create_new(struct mmcsd_blk_device *blk_dev,
                                   rt_device_t dev_sd) {
  uint8_t *buffer = NULL;

  const uint8_t uuid[] = {
    0x9D, 0x85, 0x6F, 0x14, 0x47, 0xE6, 0xC0, 0x4C, 0xB7, 0x1B, 0xF4, 0x44, 0x72, 0xD2, 0x5F, 0x4D
  };

  struct dfs_partition part, prev_part;

  gpt_header _gpt_header;
  gpt_entry *_gpt_entry = NULL, *new_entry, *prev_entry;
  size_t _gpt_entry_size = 0;

  const size_t sector_count = blk_dev->card->card_capacity * (1024 / 512) - 1;

  int label_offet = 0;

  if(0x00 != gpt_get_partition_param(blk_dev->card, &part, CONFIG_RT_AUTO_RESIZE_PARTITION_NR)) {
    /* Can't find wanted part */

    // get prev part info.
    if(0x00 != gpt_get_partition_param(blk_dev->card, &prev_part, (CONFIG_RT_AUTO_RESIZE_PARTITION_NR - 1))) {
      rt_kprintf("%s get last partition info failed.\n", __func__);
      goto _exit;
    }

    if (NULL == (buffer = rt_malloc_align(SECTOR_SIZE, RT_CPU_CACHE_LINE_SZ))) {
      rt_kprintf("%s malloc failed 1.\n", __func__);
      return -1;
    }

    // read gpt header
    if (0x01 != rt_device_read(dev_sd, 1, buffer, 1)) {
      rt_kprintf("%s read sector failed 1.\n", __func__);
      goto _exit;
    }
    rt_memcpy(&_gpt_header, buffer, sizeof(gpt_header));

    if(GPT_ENTRIES != _gpt_header.num_partition_entries) {
      rt_kprintf("invalid gpt header.\n");
      goto _exit;
    }

    _gpt_entry_size = _gpt_header.num_partition_entries * _gpt_header.sizeof_partition_entry;
    if(NULL == (_gpt_entry = rt_malloc_align(_gpt_entry_size, RT_CPU_CACHE_LINE_SZ))) {
      rt_kprintf("%s malloc failed 2.\n", __func__);
      goto _exit;
    }

    if ((_gpt_entry_size / SECTOR_SIZE) != rt_device_read(dev_sd, _gpt_header.partition_entry_lba, _gpt_entry, _gpt_entry_size / SECTOR_SIZE)) {
      rt_kprintf("%s read sector failed 2.\n", __func__);
      goto _exit;
    }

    prev_entry = &_gpt_entry[CONFIG_RT_AUTO_RESIZE_PARTITION_NR - 1];
    new_entry = &_gpt_entry[CONFIG_RT_AUTO_RESIZE_PARTITION_NR];

    // create a new partition
    new_entry->partition_type_guid = PARTITION_BASIC_DATA_GUID; // FAT32
    memcpy(new_entry->unique_partition_guid.b, uuid, sizeof(uuid));

    new_entry->starting_lba = (prev_entry->ending_lba + 0x200000) & ~(0x200000 - 1); // align to 1GiB
    new_entry->ending_lba = sector_count;

    new_entry->attributes.required_to_function = 0;
    new_entry->attributes.type_guid_specific = 0;

    // label
    label_offet = 0;
    new_entry->partition_name[label_offet++] = 'd';
    new_entry->partition_name[label_offet++] = 'a';
    new_entry->partition_name[label_offet++] = 't';
    new_entry->partition_name[label_offet++] = 'a';
    new_entry->partition_name[label_offet++] = '\0';

    // update gpt header
    _gpt_header.last_usable_lba = sector_count;

    _gpt_header.partition_entry_array_crc32 = gpt_crc32(_gpt_entry, _gpt_entry_size);

    _gpt_header.header_crc32 = 0;
    _gpt_header.header_crc32 = gpt_crc32(&_gpt_header, sizeof(gpt_header));

    if ((_gpt_entry_size / SECTOR_SIZE) != rt_device_write(dev_sd, _gpt_header.partition_entry_lba, _gpt_entry, _gpt_entry_size / SECTOR_SIZE)) {
      rt_kprintf("%s write sector failed 1.\n", __func__);

      rt_free_align(_gpt_entry);
      goto _exit;
    }

    rt_memcpy(buffer, &_gpt_header, sizeof(_gpt_header));
    if (0x01 != rt_device_write(dev_sd, 1, buffer, 1)) {
      rt_kprintf("%s write sector failed 2.\n", __func__);

      rt_free_align(_gpt_entry);
      goto _exit;
    }

    rt_kprintf("Create new GPT partiton 0x%x - 0x%x\n",
                new_entry->starting_lba * SECTOR_SIZE, new_entry->ending_lba * SECTOR_SIZE);

    rt_free_align(buffer);
    rt_free_align(_gpt_entry);

    return 0;
  } else {
    rt_free_align(buffer);
    return 1;
  }

_exit:
  if (buffer) {
    rt_free_align(buffer);
  }

  return -1;
}
#endif

static inline __attribute__((always_inline)) void disable_auto_resize(void) {
  int fd = -1;
  char *file_buffer = NULL;
  size_t file_buffer_size = 0;

  if ((0x00 == read_config_file(&fd, &file_buffer, &file_buffer_size)) &&
      (0 <= fd) && (NULL != file_buffer)) {
    /* change auto_resize to n */
    int offset_auto_resize = 0;
    char *p_auto_resize = rt_strstr(file_buffer, "auto_resize=");
    if (p_auto_resize) {
      lseek(fd, p_auto_resize - file_buffer + sizeof("auto_resize=") - 1,
            SEEK_SET);
      write(fd, "n", 1);
      close(fd);

    } else {
      close(fd);
    }

    if (file_buffer) {
      rt_free(file_buffer);
    }
  } else {
    rt_kprintf("%s, read config file failed\n", __func__);
  }
}

static void execute_auto_resize(char *value) {
  int ret = 0;
  int fd = -1;
  char enable = 0;

  char dev_name[16];
  rt_device_t dev_sd = NULL;

  struct mmcsd_blk_device *blk_dev = NULL;

  if (0x01 != sscanf(value, "%c", &enable)) {
    rt_kprintf("%s parse value failed.\n", __func__);
    return;
  }

  if ('y' != enable) {
    if(0 <= (fd = open("/bin/auto_mkfs_data", O_RDONLY))) {
      close(fd);
      unlink("/bin/auto_mkfs_data");

      fd = -1;
    }

    return;
  }

  disable_auto_resize();

  rt_snprintf(dev_name, sizeof(dev_name), "sd%d", SDCARD_ON_SDIO_DEV);
  if (NULL == (dev_sd = rt_device_find(dev_name))) {
    rt_kprintf("%s find /dev/sd failed.\n", __func__);
    goto _exit;
  }

  if (RT_EOK != (ret = rt_device_open(dev_sd, RT_DEVICE_OFLAG_RDWR))) {
    rt_kprintf("%s open /dev/sd failed.\n", __func__);
    goto _exit;
  }

  blk_dev = rt_container_of(dev_sd, struct mmcsd_blk_device, dev);
  if (NULL == blk_dev) {
    goto _exit;
  }

#if defined (ENABLE_GPT_PART_RESIZE) && ENABLE_GPT_PART_RESIZE
  if (0x00 == check_gpt(blk_dev->card)) {
    /* MBR */
    ret = mmc_mbr_part_create_new(blk_dev, dev_sd);
  } else {
    /* GPT */
    ret = mmc_gpt_part_create_new(blk_dev, dev_sd);
    gpt_free();
  }
#else
    ret = mmc_mbr_part_create_new(blk_dev, dev_sd);
#endif

  if (0x00 == ret) {
    rt_device_close(dev_sd);

    /* create /bin/auto_mkfs */
    if (0 > (fd = open("/bin/auto_mkfs_data", O_CREAT | O_WRONLY))) {
      rt_kprintf("%s, create /bin/auto_mkfs_data failed\n", __func__);
    } else {
      close(fd);
    }

    /* reboot */
    rt_kprintf("reboot...\n");
    rt_hw_cpu_reset();
  } else if(0 > ret) {
    rt_kprintf("Create new partition failed.\n");
  }

_exit:
  if (dev_sd) {
    rt_device_close(dev_sd);
  }
}
#else
static void execute_auto_resize(char *value) {
  rt_kprintf("not support reisze partition.\n");
}
#endif

static const struct config_handler hanlders[] = {
    {
        0x277A782F /* auto_resize */,
        execute_auto_resize,
    },
};

static inline __attribute__((always_inline)) void excete_line(char *line) {
  uint32_t hash_command;

  char *p_command = line;
  char *p_value = NULL;

  if (NULL == (p_value = rt_strstr(p_command, "="))) {
    rt_kprintf("Invaild line '%s'\n", line);
    return;
  }
  p_value[0] = '\0';
  p_value++;

  hash_command = shash(p_command);

  for (size_t i = 0; i < sizeof(hanlders) / sizeof(hanlders[0]); i++) {
    if (hash_command == hanlders[i].command) {
      hanlders[i].handler(p_value);
      break;
    }
  }
}

void excute_sdcard_config(void) {
  int fd = -1;
  size_t length = 0;
  int line_length = 0;

  char *buffer = NULL;
  char *buffer_end = NULL;
  char *p_line_start = NULL;
  char *p_line_end = NULL;

  char line[LINE_MAX_SIZE];

  if (0x00 == read_config_file(&fd, &buffer, &length)) {
    close(fd);

    buffer_end = buffer + length;
    p_line_start = buffer;

    while (p_line_start < buffer_end) {
      p_line_end = rt_strstr(p_line_start, "\n");
      if (NULL == p_line_end) {
        break;
      }
      line_length = p_line_end - p_line_start;
      if (0x00 == line_length) {
        break;
      }
      if (LINE_MAX_SIZE <= line_length) {
        rt_kprintf("parse config, line data too long\n");
        p_line_start = p_line_end;
        continue;
      }
      memcpy(line, p_line_start, line_length);

      excete_line(line);

      p_line_start = p_line_end;
    }
  } else {
    rt_kprintf("open %s failed\n", RT_SMART_CONFIG_FILE_PATH);
  }

  if (buffer) {
    rt_free(buffer);
  }
}
