/* Copyright 2020 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <stdlib.h>
#include "utest.h"
#include <math.h>

#include "drv_ts.h"

#define TS_DEV_NAME               "ts"
rt_device_t ts_dev = RT_NULL;

// #define _debug_print

#ifdef  _debug_print
#include <stdio.h>
#define DBG_BUFF_MAX_LEN          256

int float_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[DBG_BUFF_MAX_LEN] = { 0 };
    va_start(args, fmt);
    int length = vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    return length;
}
#endif

static void ts_open(void)
{
    ts_dev = (rt_device_t)rt_device_find(TS_DEV_NAME);
    if (ts_dev == RT_NULL)
    {
        uassert_false(1);
        return;
    }

    if(rt_device_open(ts_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", TS_DEV_NAME);
        return;
    }

    return;
}

static void ts_close(void)
{
    rt_device_close(ts_dev);
}

static void ts_device_single_mode(void)
{
    rt_uint8_t mode = RT_DEVICE_TS_CTRL_MODE_SINGLE;
    rt_device_control(ts_dev, RT_DEVICE_TS_CTRL_SET_MODE, &mode);
}

static void ts_device_continuos_mode(void)
{
    rt_uint8_t mode = RT_DEVICE_TS_CTRL_MODE_CONTINUUOS;
    rt_device_control(ts_dev, RT_DEVICE_TS_CTRL_SET_MODE, &mode);
}

static void test_ts_read(void)
{
    double temp = 0.0f;

    ts_open();

    ///////////////////////////////////////////////////////////////////////////
    rt_kprintf("enter continuos mode\n");

    ts_device_continuos_mode();
    for(int i = 0; i < 5; i++) {
        if(sizeof(double) != rt_device_read(ts_dev, 0, &temp, sizeof(double))) {
            uassert_false(1);
            goto _failed;
        }
        rt_kprintf("Continuos mode Temperature = %d.%d C\n", ((int)(temp * 1000000) / 1000000), ((int)(temp * 1000000) % 1000000));

        rt_thread_delay(rt_tick_from_millisecond(500));
    }

    ///////////////////////////////////////////////////////////////////////////
    rt_kprintf("enter single mode\n");

    ts_device_single_mode();
    for(int i = 0; i < 5; i++) {
        if(sizeof(double) != rt_device_read(ts_dev, 0, &temp, sizeof(double))) {
            uassert_false(1);
            goto _failed;
        }
        rt_kprintf("Single mode Temperature = %d.%d C\n", ((int)(temp * 1000000) / 1000000), ((int)(temp * 1000000) % 1000000));
    
        rt_thread_delay(rt_tick_from_millisecond(500));
    }

    ///////////////////////////////////////////////////////////////////////////
    rt_kprintf("enter continuos mode\n");

    ts_device_continuos_mode();
    for(int i = 0; i < 5; i++) {
        if(sizeof(double) != rt_device_read(ts_dev, 0, &temp, sizeof(double))) {
            uassert_false(1);
            goto _failed;
        }
        rt_kprintf("Continuos mode Temperature = %d.%d C\n", ((int)(temp * 1000000) / 1000000), ((int)(temp * 1000000) % 1000000));

        rt_thread_delay(rt_tick_from_millisecond(500));
    }

    ///////////////////////////////////////////////////////////////////////////
    rt_kprintf("enter single mode\n");

    ts_device_single_mode();
    for(int i = 0; i < 5; i++) {
        if(sizeof(double) != rt_device_read(ts_dev, 0, &temp, sizeof(double))) {
            uassert_false(1);
            goto _failed;
        }
        rt_kprintf("Single mode Temperature = %d.%d C\n", ((int)(temp * 1000000) / 1000000), ((int)(temp * 1000000) % 1000000));
    
        rt_thread_delay(rt_tick_from_millisecond(500));
    }
    ///////////////////////////////////////////////////////////////////////////

_failed:
    ts_close();

    return;
}

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void testcase(void)
{
    UTEST_UNIT_RUN(test_ts_read);
}
UTEST_TC_EXPORT(testcase, "testcases.kernel.ts_tc", utest_tc_init, utest_tc_cleanup, 100);

/********************* end of file ************************/
