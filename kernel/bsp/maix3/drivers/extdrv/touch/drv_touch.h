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

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "rtdef.h"
#include <rtthread.h>
#include <touch.h>

#include "drv_gpio.h"

#define TOUCH_MAX_POINT_NUMBER      5
#define TOUCH_READ_REG_MAX_SIZE     32
#define TOUCH_READ_MQ_MSG_COUNT     3
#define TOUCH_TIMEOUT_MS            1000

struct touch_register {
    rt_tick_t time;
    uint32_t reg[TOUCH_READ_REG_MAX_SIZE / sizeof(uint32_t)];
};
typedef struct touch_register touch_read_mq_msg_type;

struct touch_point {
    int point_num;
    struct rt_touch_data point[TOUCH_MAX_POINT_NUMBER];
};

struct drv_touch_dev {
    const struct {
        int intr;
        gpio_pin_edge_t intr_edge;

        int rst;
        int rst_valid;
    } pin;

    struct {
        struct rt_i2c_bus_device *bus;
        const char *name;
        const  uint32_t speed;
        const  uint16_t addr;
    } i2c;

    struct {
        struct rt_touch_device touch;

        void *priv;

        int (*read_register)(struct drv_touch_dev *dev, struct touch_register *reg);
        int (*parse_register)(struct drv_touch_dev *dev, struct touch_register *reg, struct touch_point *result);

        int (*reset)(struct drv_touch_dev *dev);
        int (*get_default_rotate)(struct drv_touch_dev *dev);
    } dev;

    const struct {
        int range_x;
        int range_y;
        int point_num;
    } touch;

#ifdef TOUCH_DRV_MODEL_INT_WITH_THREAD
    struct {
        struct rt_thread thr;
        uint32_t thread_stack[RT_ALIGN(TOUCH_DRV_THREAD_STACK_SIZE, RT_ALIGN_SIZE) / sizeof(uint32_t)];

        struct rt_semaphore ctrl_sem;

        struct rt_messagequeue ctrl_mq;
        char ctrl_mq_pool[(sizeof(void *) + RT_ALIGN(sizeof(uint32_t), RT_ALIGN_SIZE)) * 1];

        struct rt_messagequeue read_mq;
        char read_mq_pool[(sizeof(void *) + RT_ALIGN(sizeof(touch_read_mq_msg_type), RT_ALIGN_SIZE)) * TOUCH_READ_MQ_MSG_COUNT];
    } thr;
#endif
};

/**
 * write slave, first byte should be register address.
 */
int touch_dev_write_reg(struct drv_touch_dev *dev, rt_uint8_t *buffer, rt_size_t length);

int touch_dev_read_reg(struct drv_touch_dev *dev, rt_uint8_t addr,
    rt_uint8_t *buffer, rt_size_t length);

/**
 * For some touch device, which have no hardware event generate, such as CST128-A.
 */
void touch_dev_update_event(int finger_num, struct rt_touch_data *point);

int drv_touch_init_ft5x16(struct drv_touch_dev *dev);
int drv_touch_init_cst128(struct drv_touch_dev *dev);
