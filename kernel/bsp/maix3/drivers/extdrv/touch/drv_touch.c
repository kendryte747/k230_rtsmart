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

#include "drv_touch.h"
#include "rtdef.h"
#include "rtthread.h"

#include <ioremap.h>
#include <lwp_user_mm.h>

#define DBG_TAG          "drv_touch"
#define DBG_LVL          DBG_WARNING
#define DBG_COLOR
#include <rtdbg.h>

#define TOUCH_THREAD_MQ_INT_FLAG    0x01
#define TOUCH_THREAD_MQ_RST_FLAG    0x02

static struct drv_touch_dev touch_dev = {
    .touch = {
        .range_x = TOUCH_RANGE_X,
        .range_y = TOUCH_RANGE_Y,
        .point_num = TOUCH_POINT_NUM,
    },

    .pin = {
        .intr = TOUCH_DEV_INT_PIN,
        .intr_edge = TOUCH_DEV_INT_EDGE,

        .rst = TOUCH_DEV_RST_PIN,
        .rst_valid = TOUCH_DEV_RST_PIN_VALID_VALUE,
    },

    .i2c = {
        .name = TOUCH_DEV_I2C_BUS,
        .addr = TOUCH_DEV_I2C_ADDR,
#if defined (TOUCH_DEV_I2C_SET_SPEED)
        .speed = TOUCH_DEV_I2C_BUS_SPEED,
#endif
        .bus = NULL,
    },
};

int touch_dev_write_reg(struct drv_touch_dev *dev, rt_uint8_t *buffer, rt_size_t length)
{
    struct rt_i2c_msg msg =
    {
        .addr   = dev->i2c.addr,
        .flags  = RT_I2C_WR,
        .buf    = buffer,
        .len    = length,
    };

    if(0x01 == rt_i2c_transfer(dev->i2c.bus, &msg, 1)) {
        return 0;
    } else {
        return -1;
    }
}

int touch_dev_read_reg(struct drv_touch_dev *dev, rt_uint8_t addr,
    rt_uint8_t *buffer, rt_size_t length)
{
    struct rt_i2c_msg msgs[2] = {
        {
            .addr   = dev->i2c.addr,
            .flags  = RT_I2C_WR,
            .buf    = &addr,
            .len    = 1,
        },
        {
            .addr   = dev->i2c.addr,
            .flags  = RT_I2C_RD,
            .buf    = buffer,
            .len    = length,
        },
    };

    if(0x02 == rt_i2c_transfer(dev->i2c.bus, msgs, 2)) {
        return 0;
    } else {
        return -1;
    }
}

void touch_dev_update_event(int finger_num, struct rt_touch_data *point) {
    static int last_finger_num = 0;
    static struct rt_touch_data last_point[TOUCH_MAX_POINT_NUMBER];
    static rt_tick_t last_timestamp = 0;  // Track the timestamp of the last touch event

    rt_bool_t new_session = RT_FALSE;

    // Check if too much time has passed since the last touch event
    if (finger_num > 0 && (point[0].timestamp - last_timestamp > TOUCH_TIMEOUT_MS)) {
        // A new touch event is considered after a timeout
        new_session = RT_TRUE;
        LOG_D("New touch session detected due to timeout.\n");

        // If needed, mark all previous touches as lifted
        for (int i = 0; i < last_finger_num; i++) {
            last_point[i].event = RT_TOUCH_EVENT_UP;
            LOG_D("Finger %d: Timeout - Finger lifted\n", last_point[i].track_id);
        }

        // Reset the last finger number since it's a new session
        last_finger_num = 0;
    }

    // Update the timestamp of the last touch event
    last_timestamp = point[0].timestamp;

    // Process current touch data
    for (int i = 0; i < finger_num; i++) {
        struct rt_touch_data *current_point = &point[i];
        struct rt_touch_data *last_touch = &last_point[i];

        if (i < last_finger_num && !new_session) {
            // Check if the touch point has moved
            if (current_point->x_coordinate != last_touch->x_coordinate ||
                current_point->y_coordinate != last_touch->y_coordinate) {
                // Update event to move
                current_point->event = RT_TOUCH_EVENT_MOVE;
            }
        } else {
            // New finger detected or new session
            current_point->event = RT_TOUCH_EVENT_DOWN;
        }

        // Print touch info for debugging (optional)
        LOG_D("Finger %d: X = %d, Y = %d, Event = %d, Timestamp = %ld\n",
               current_point->track_id,
               current_point->x_coordinate,
               current_point->y_coordinate,
               current_point->event,
               current_point->timestamp);

        // Update the last point data
        rt_memcpy(last_touch, current_point, sizeof(struct rt_touch_data));
    }

    // Check if fingers have been lifted
    if (finger_num < last_finger_num && !new_session) {
        for (int i = finger_num; i < last_finger_num; i++) {
            struct rt_touch_data *last_touch = &last_point[i];
            // Finger lifted
            last_touch->event = RT_TOUCH_EVENT_UP;
            LOG_D("Finger %d: Lifted, Event = %d, Timestamp = %ld\n",
                   last_touch->track_id, last_touch->event, last_touch->timestamp);
        }
    }

    // Update last finger number
    last_finger_num = finger_num;
}

#ifdef TOUCH_DRV_MODEL_INT_WITH_THREAD
#if (0 > TOUCH_DEV_INT_PIN) || (63 < TOUCH_DEV_INT_PIN)
    #error "Touch INT Pin config invalid."
#endif

static void touch_int_irq(void *args) {
    struct drv_touch_dev *dev = args;

    uint32_t flag = TOUCH_THREAD_MQ_INT_FLAG;

    rt_mq_send(&dev->thr.ctrl_mq, &flag, sizeof(uint32_t));
}

static void touch_read_thread(void *args) {
    struct drv_touch_dev *dev = args;

    uint32_t recv;
    struct touch_register reg, dump[2];

    while(1) {
        if(RT_EOK == rt_mq_recv(&dev->thr.ctrl_mq, &recv, sizeof(uint32_t), RT_WAITING_FOREVER)) {
            if(TOUCH_THREAD_MQ_INT_FLAG == recv) {
                if(dev->dev.read_register) {
                    if(0x00 != dev->dev.read_register(dev, &reg)) {
                        LOG_E("Touch device read register faild.");

                        continue;
                    }

                    if(RT_EOK != rt_mq_send(&dev->thr.read_mq, &reg, sizeof(reg))) {
                        // drop oldest message
                        rt_mq_recv(&dev->thr.read_mq, &dump[0], sizeof(dump), RT_WAITING_NO);

                        rt_mq_send(&dev->thr.read_mq, &reg, sizeof(reg));
                    }
                } else {
                    LOG_W("Touch device not impl read_register");
                }
            } else if(TOUCH_THREAD_MQ_RST_FLAG == recv) {
                if(dev->dev.reset) {
                    if(0x00 != dev->dev.reset(dev)) {
                        LOG_E("reset touch failed");
                    }

                    rt_thread_delay(rt_tick_from_millisecond(30));

                    rt_sem_release(&dev->thr.ctrl_sem);
                }
            } else {
                LOG_W("Unknown message %d\n", recv);
            }
        }
    }
}

static rt_size_t drv_touch_read(struct rt_touch_device *touch, void *buf, rt_size_t len) {
    rt_tick_t time;

    struct touch_register reg;
    struct touch_point point;

    struct drv_touch_dev *dev = touch->config.user_data;

    const rt_tick_t one_sec_tick = rt_tick_from_millisecond(1000);

    rt_size_t req_buffer_size = 0;
    rt_size_t req_finger_num = len / sizeof(struct rt_touch_data);
    struct rt_touch_data *req_point = (struct rt_touch_data *)buf;

    while(1) {
        // if we can't recv reg buffer from mq, return 0 as no touch data
        if(RT_EOK != rt_mq_recv(&dev->thr.read_mq, &reg, sizeof(reg), rt_tick_from_millisecond(3))) {
            return 0;
        }

        // if the reg data is push to queue 1s before, we use it
        time = rt_tick_get() - reg.time;
        if(one_sec_tick > time) {
            break;
        }
    }

    if(dev->dev.parse_register) {
        if(0x00 != dev->dev.parse_register(dev, &reg, &point)) {
            LOG_E("Touch parse register failed.");
            return 0;
        }
        req_finger_num = (req_finger_num > point.point_num) ? point.point_num : req_finger_num;
        req_buffer_size = req_finger_num * sizeof(struct rt_touch_data);

        memcpy(req_point, &point.point[0], req_buffer_size);

        return req_buffer_size;
    } else {
        LOG_W("Touch device not impl parse_register.");
    }

    return 0;
}
#else
static volatile int int_flag = 0x00;

static void touch_int_irq(void *args) {
    int_flag = 1;
}

static rt_size_t drv_touch_read(struct rt_touch_device *touch, void *buf, rt_size_t len) {
    struct touch_register reg;
    struct touch_point point;

    struct drv_touch_dev *dev = touch->config.user_data;

    rt_size_t req_buffer_size = 0;
    rt_size_t req_finger_num = len / sizeof(struct rt_touch_data);
    struct rt_touch_data *req_point = (struct rt_touch_data *)buf;

    if((0 <= dev->pin.intr) && (0x00 == int_flag)) {
        return 0;
    }
    int_flag = 0x00;

    if(dev->dev.read_register) {
        if(RT_EOK != dev->dev.read_register(dev, &reg)) {
            LOG_E("Touch device read register faild.");
            return 0;
        }

        if(dev->dev.parse_register) {
            if(0x00 != dev->dev.parse_register(dev, &reg, &point)) {
                LOG_E("Touch parse register failed.");
                return 0;
            }
            req_finger_num = (req_finger_num > point.point_num) ? point.point_num : req_finger_num;
            req_buffer_size = req_finger_num * sizeof(struct rt_touch_data);

            memcpy(req_point, &point.point[0], req_buffer_size);

            return req_buffer_size;
        } else {
            LOG_W("Touch device not impl parse_register.");
        }
    } else {
        LOG_W("Touch device not impl read_register");
    }

    return 0;
}
#endif

static rt_err_t drv_touch_control(struct rt_touch_device *touch, int cmd, void *arg) {
    struct drv_touch_dev *dev = touch->config.user_data;

    switch(cmd) {
        case RT_TOUCH_CTRL_GET_INFO: {
            lwp_put_to_user(arg, &touch->info, sizeof(struct rt_touch_info));

            return RT_EOK;
        } break;
        case RT_TOUCH_CTRL_RESET: {
            if(dev->dev.reset) {
#ifdef TOUCH_DRV_MODEL_INT_WITH_THREAD
                // push command to thread
                uint32_t flag = TOUCH_THREAD_MQ_RST_FLAG;
                rt_mq_send_wait(&dev->thr.ctrl_mq, &flag, sizeof(uint32_t), RT_WAITING_FOREVER);

                if(RT_EOK != rt_sem_take(&dev->thr.ctrl_sem, rt_tick_from_millisecond(200))) {
                    LOG_W("Wait Thread reset touch timeout.");
                    return -RT_ETIMEOUT;
                }

                return RT_EOK;
#else
                return dev->dev.reset(dev);
#endif
            } else {
                LOG_W("Touch device not impl get_default_rotate");
                return RT_EOK;
            }
        } break;
        case RT_TOUCH_CTRL_GET_DFT_ROTATE: {
            int rotate = RT_TOUCH_ROTATE_DEGREE_0;

            if(dev->dev.get_default_rotate) {
                rotate = dev->dev.get_default_rotate(dev);
            } else {
                LOG_W("Touch device not impl get_default_rotate");
            }

            lwp_put_to_user(arg, &rotate, sizeof(rotate));

            return RT_EOK;
        } break;
    }

    return -RT_ENOSYS;
}

static struct rt_touch_ops drv_touch_ops = {
    .touch_readpoint = drv_touch_read,
    .touch_control = drv_touch_control,
};

static int drv_touch_init(void) {
    int ret;
    struct drv_touch_dev *dev = &touch_dev;

    if(NULL == (dev->i2c.bus = rt_i2c_bus_device_find(dev->i2c.name))) {
        LOG_E("Can't find Touch Device I2C Bus %s", dev->i2c.name);
        return -1;
    }

    if (RT_EOK != (ret = rt_device_open((rt_device_t)dev->i2c.bus, RT_DEVICE_FLAG_RDWR))) {
        LOG_E("Open Touch Device I2C dev %s failed: %d", dev->i2c.name, ret);
        return -2;
    }

#if defined (TOUCH_DEV_I2C_SET_SPEED)
    if(RT_EOK != (ret = rt_device_control((rt_device_t)dev->i2c.bus, RT_I2C_DEV_CTRL_CLK, (uint32_t *)&dev->i2c.speed))) {
        LOG_E("Set Touch Device I2C dev clock %s failed: %d", dev->i2c.name, ret);
        return -3;
    }
#endif

    if(0x00 != 
#if defined TOUCH_TYPE_FT5316
    drv_touch_init_ft5x16(dev)
#elif defined TOUCH_TYPE_CST128
    drv_touch_init_cst128(dev)
#endif
    )
    {
        LOG_E("Touch init device failed.");
        return -4;
    }

#ifdef TOUCH_DRV_MODEL_INT_WITH_THREAD
    rt_sem_init(&dev->thr.ctrl_sem, "touch_ctrl", 0, RT_IPC_FLAG_FIFO);

    rt_mq_init(&dev->thr.ctrl_mq, "touch_int", &dev->thr.ctrl_mq_pool[0], sizeof(uint32_t), sizeof(dev->thr.ctrl_mq_pool), RT_IPC_FLAG_FIFO);

    rt_mq_init(&dev->thr.read_mq, "touch_read", &dev->thr.read_mq_pool[0], sizeof(touch_read_mq_msg_type), sizeof(dev->thr.read_mq_pool), RT_IPC_FLAG_FIFO);

    rt_thread_init(&dev->thr.thr, "touch_read", touch_read_thread, dev, &dev->thr.thread_stack[0], sizeof(dev->thr.thread_stack), TOUCH_DRV_THREAD_PRIO, 5);
    rt_thread_startup(&dev->thr.thr);
#endif

    if(0 <= dev->pin.intr) {
        kd_pin_mode(dev->pin.intr, GPIO_DM_INPUT);
        kd_pin_attach_irq(dev->pin.intr, GPIO_PE_FALLING, touch_int_irq, dev);
        kd_pin_irq_enable(dev->pin.intr, KD_GPIO_IRQ_ENABLE);
    }

    dev->dev.touch.config.dev_name = "touch0";
    dev->dev.touch.config.user_data = dev;

    dev->dev.touch.info.type = RT_TOUCH_TYPE_CAPACITANCE;
    dev->dev.touch.info.vendor = RT_TOUCH_VENDOR_UNKNOWN;

    dev->dev.touch.info.point_num = dev->touch.point_num;
    dev->dev.touch.info.range_x = dev->touch.range_x;
    dev->dev.touch.info.range_y = dev->touch.range_y;

    dev->dev.touch.ops = &drv_touch_ops;

    if(RT_EOK != (ret= rt_hw_touch_register(&dev->dev.touch, dev->dev.touch.config.dev_name, 0, RT_NULL))) {
        LOG_E("Register Touch Device failed %d.", ret);
        return -5;
    }
    
    return 0;
}
INIT_COMPONENT_EXPORT(drv_touch_init);
