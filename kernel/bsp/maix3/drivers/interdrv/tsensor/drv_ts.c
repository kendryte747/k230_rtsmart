/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "drv_ts.h"

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_POSIX
#include <dfs_posix.h>
#include <dfs_poll.h>
#include <posix_termios.h>
#endif

#include "math.h"

#include "board.h"
#include <riscv_io.h>
#include "ioremap.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Register offsets
#define REG_TSENW_OFFSET   0x000
#define REG_TSENR_OFFSET   0x004

// Bit positions for REG_TSENW
#define TSENW_TS_TEST_EN_POS      6
#define TSENW_TS_TRIM_POS         2
#define TSENW_TS_CONV_MODE_POS    1
#define TSENW_TS_EN_POS           0

// Bit positions for REG_TSENR
#define TSENR_TS_DOUT_VALID_POS   12
#define TSENR_TS_DOUT_MASK        0xFFF

static struct rt_mutex ts_mutex;

static uint8_t ts_trim = 8;
static uint8_t ts_mode = RT_DEVICE_TS_CTRL_MODE_CONTINUUOS;

static void *ts_base_addr = RT_NULL;

static void tsensor_start(void) {
    if (ts_base_addr == RT_NULL) return; // Ensure base address is set

    // Ensure single output mode is selected and enable the TSensor
    uint32_t reg_val = readl(ts_base_addr + REG_TSENW_OFFSET);
    if(RT_DEVICE_TS_CTRL_MODE_CONTINUUOS == ts_mode) {
        reg_val |= (1 << TSENW_TS_CONV_MODE_POS); // Set continuous mode bit
    } else {
        reg_val &= ~(1 << TSENW_TS_CONV_MODE_POS); // Clear continuous mode bit
    }
    reg_val |= (1 << TSENW_TS_EN_POS);         // Set enable bit
    writel(reg_val, ts_base_addr + REG_TSENW_OFFSET);
}

static void tsensor_stop(void) {
    if (ts_base_addr == RT_NULL) return; // Ensure base address is set

    // Disable the TSensor
    uint32_t reg_val = readl(ts_base_addr + REG_TSENW_OFFSET);
    reg_val &= ~((1 << TSENW_TS_EN_POS));
    writel(reg_val, ts_base_addr + REG_TSENW_OFFSET);
}

static int tsensor_read_data(uint16_t *data, uint32_t timeout_ms) {
    if (ts_base_addr == RT_NULL || data == RT_NULL) return -1; // Ensure base address is set

    uint32_t max_attempts = timeout_ms; // Max attempts for the given timeout in ms

    for (uint32_t attempt = 0; attempt < max_attempts; attempt++) {
        // Check if the data is valid
        if ((readl(ts_base_addr + REG_TSENR_OFFSET) >> TSENR_TS_DOUT_VALID_POS) & 0x1) {
            // Read the 12-bit temperature data
            *data = readl(ts_base_addr + REG_TSENR_OFFSET) & TSENR_TS_DOUT_MASK;
            return 0; // Success
        }

        // Delay before next polling attempt
        rt_thread_mdelay(1); // Delay in microseconds
    }

    return -2; // Timeout error
}

static double tsensor_calculate_temperature(uint16_t data) {
    return (1e-10 * pow(data, 4) * 1.01472
            - 1e-6 * pow(data, 3) * 1.10063
            + 4.36150 * 1e-3 * pow(data, 2)
            - 7.10128 * data
            + 3565.87);
}

void tsensor_init(void) {
    if (ts_base_addr == RT_NULL) return; // Ensure base address is set

    if(RT_EOK != rt_mutex_take(&ts_mutex, rt_tick_from_millisecond(500))) {
        rt_kprintf("%s mutex take timeout.\n");

        return;
    }

    // Configure the TSensor with the default settings
    uint32_t reg_val = readl(ts_base_addr + REG_TSENW_OFFSET);
    reg_val &= ~(0xF << TSENW_TS_TRIM_POS);       // Clear previous trim value
    reg_val |= (ts_trim << TSENW_TS_TRIM_POS); // Set new trim value
    writel(reg_val, ts_base_addr + REG_TSENW_OFFSET);

    rt_mutex_release(&ts_mutex);
}

void tsensor_set_trim(uint8_t trim) {
    if(RT_EOK != rt_mutex_take(&ts_mutex, rt_tick_from_millisecond(500))) {
        rt_kprintf("%s mutex take timeout.\n");

        return;
    }

    // Ensure the trim_value is within range (4 bits)
    ts_trim = trim & 0xF;

    rt_mutex_release(&ts_mutex);

    tsensor_init();
}

uint8_t tsensor_get_trim(void) {
    uint8_t temp;

    if(RT_EOK != rt_mutex_take(&ts_mutex, rt_tick_from_millisecond(500))) {
        rt_kprintf("%s mutex take timeout.\n");
        return -1;
    }

    temp = ts_trim;

    rt_mutex_release(&ts_mutex);

    return temp;
}

void tsensor_set_mode(uint8_t mode) {
    if(RT_EOK != rt_mutex_take(&ts_mutex, rt_tick_from_millisecond(500))) {
        rt_kprintf("%s mutex take timeout.\n");

        return;
    }

    ts_mode = mode;

    rt_mutex_release(&ts_mutex);
}

uint8_t tsensor_get_mode(void) {
    uint8_t temp;

    if(RT_EOK != rt_mutex_take(&ts_mutex, rt_tick_from_millisecond(500))) {
        rt_kprintf("%s mutex take timeout.\n");
        return -1;
    }

    temp = ts_mode;

    rt_mutex_release(&ts_mutex);

    return temp;
}

int tsensor_read_temp(double *temp) {
    uint16_t data;

    if(RT_EOK != rt_mutex_take(&ts_mutex, rt_tick_from_millisecond(500))) {
        rt_kprintf("%s mutex take timeout.\n");
        return -1;
    }

    tsensor_start();

    rt_thread_mdelay(10);

    if(0x00 == tsensor_read_data(&data, 100)) {
        goto _succ;
    }
    tsensor_stop();

    rt_mutex_release(&ts_mutex);

    return -1;
_succ:
    tsensor_stop();

    *((double *)temp) = tsensor_calculate_temperature(data);

    rt_mutex_release(&ts_mutex);

    return 0;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined (DRV_TS_USE_RT_DEVICE)

static rt_err_t ts_deivce_open(rt_device_t dev, rt_uint16_t oflag)
{
    tsensor_init();

    return RT_EOK;
}

static rt_err_t ts_device_close(rt_device_t dev)
{
    tsensor_stop();

    return RT_EOK;
}

static rt_size_t ts_device_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if(sizeof(double) != size) {
        rt_kprintf("%s invalid buffer size %u\n", __func__, size);
        return 0;
    }

    if(0x00 != tsensor_read_temp(buffer)) {
        return 0; // read failed
    }

    return sizeof(double);
}

static rt_err_t ts_device_control(rt_device_t dev, int cmd, void *args)
{
    uint8_t trim_val = tsensor_get_trim();
    uint8_t work_mode = tsensor_get_mode();

    switch(cmd) {
        case RT_DEVICE_TS_CTRL_SET_MODE: {
            work_mode = (rt_uint8_t)(*(rt_uint8_t *)args);

            tsensor_set_mode(work_mode);
        } break;
        case RT_DEVICE_TS_CTRL_GET_MODE: {
            *((rt_uint8_t *)args) = (rt_uint8_t)work_mode;
        } break;
        case RT_DEVICE_TS_CTRL_SET_TRIM: {
            trim_val = (rt_uint8_t)(*(rt_uint8_t *)args);

            tsensor_set_trim(trim_val);
        } break;
        case RT_DEVICE_TS_CTRL_GET_TRIM: {
            *((rt_uint8_t *)args) = (rt_uint8_t)trim_val;
        } break;
        default: {
            rt_kprintf("%s unsupport cmd 0x%x\n", __func__, cmd);
        } break;
    }

    return RT_EOK;
}

static struct rt_device ts_device;

static const struct rt_device_ops ts_ops =
{
    RT_NULL,
    ts_deivce_open,
    ts_device_close,
    ts_device_read,
    RT_NULL,
    ts_device_control
};

static int register_ts_device(void)
{
    rt_device_t device;
    rt_err_t ret = RT_EOK;

    device = &ts_device;

#ifdef RT_USING_DEVICE_OPS
    device->ops = &ts_ops;
#else
    device->init       =     RT_NULL;
    device->open       =     ts_deivce_open;
    device->close      =     ts_device_close;
    device->read       =     ts_device_read;
    device->write      =     RT_NULL;
    device->control    =     ts_device_control;
#endif

    if(RT_EOK != (ret = rt_device_register(device, "ts", RT_DEVICE_FLAG_RDWR))) {
        rt_kprintf("ts device register fail\n");
        return -1;
    }

    return 0;
}
#endif // DRV_TS_USE_RT_DEVICE
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
static int rt_hw_ts_init(void)
{
    if(RT_NULL == (ts_base_addr = rt_ioremap((void*)TS_BASE_ADDR, TS_IO_SIZE))) {
        rt_kprintf("ts ioremap error\n");
        return -1;
    }

#if defined (DRV_TS_USE_RT_DEVICE)
    if(0x00 != register_ts_device()) {
        return -2;
    }
#endif // DRV_TS_USE_RT_DEVICE

    if(RT_EOK != rt_mutex_init(&ts_mutex, "dev_ts", RT_IPC_FLAG_PRIO)) {
        rt_kprintf("ts init mutex error\n");
        return -3;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_ts_init);
