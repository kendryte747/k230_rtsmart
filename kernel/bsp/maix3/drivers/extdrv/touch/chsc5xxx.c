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
#include "drivers/touch.h"
#include "drv_touch.h"
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG          "chsc5xxx"
#define DBG_LVL          DBG_WARNING
#define DBG_COLOR
#include <rtdbg.h>

struct chsc5xxx_reg {
    rt_uint8_t act;
    rt_uint8_t finger_num;
    struct {
        rt_uint8_t xl;
        rt_uint8_t yl;
        rt_uint8_t resv;
        rt_uint8_t xh:4;
        rt_uint8_t yh:4;
        rt_uint8_t id:4;
        rt_uint8_t event:4;
    } pos[TOUCH_CHSC5XXX_MAX_POINTS];
};

#define CHSC5XXX_READ_REG_SIZE ((sizeof(struct chsc5xxx_reg) + 3) & ~3)

_Static_assert(CHSC5XXX_READ_REG_SIZE <= TOUCH_READ_REG_MAX_SIZE, "CHSC5XXX_READ_REG_SIZE > TOUCH_READ_REG_MAX_SIZE");

static int chsc5xxx_read_reg(struct drv_touch_dev *dev, rt_uint32_t addr, rt_uint8_t *buffer, rt_size_t length) {
    rt_uint32_t _addr = ((addr & 0xFF000000) >> 24) | ((addr & 0x00FF0000) >> 8) | ((addr & 0x0000FF00) << 8) | ((addr & 0x000000FF) << 24);
    struct rt_i2c_msg msgs[2] = {
        {
            .addr   = dev->i2c.addr,
            .flags  = RT_I2C_WR,
            .buf    = (rt_uint8_t *)&_addr,
            .len    = sizeof(addr),
        },
        {
            .addr   = dev->i2c.addr,
            .flags  = RT_I2C_RD,
            .buf    = buffer,
            .len    = length,
        },
    };

    if(2 == rt_i2c_transfer(dev->i2c.bus, msgs, 2)) {
        return 0;
    } else {
        return -1;
    }
}

static int read_register(struct drv_touch_dev *dev, struct touch_register *reg) {
    reg->time = rt_tick_get();

    return chsc5xxx_read_reg(dev, 0x2000002C, (uint8_t *)&reg->reg[0], CHSC5XXX_READ_REG_SIZE);
}

static int parse_register(struct drv_touch_dev *dev, struct touch_register *reg, struct touch_point *result) {
    const rt_uint8_t event[4] = {RT_TOUCH_EVENT_DOWN, RT_TOUCH_EVENT_UP, RT_TOUCH_EVENT_MOVE, RT_TOUCH_EVENT_NONE};
    
    rt_uint8_t finger_num = 0;
    rt_uint8_t xh, xl, yh, yl, flg, id;
    rt_uint16_t point_x, point_y;
    rt_uint8_t result_index = 0, point_index = 0;

    rt_tick_t time = reg->time;

    struct rt_touch_data *point = NULL;
    struct chsc5xxx_reg *chsc5xxx_reg = (struct chsc5xxx_reg *)reg->reg;

    if (0xFF != chsc5xxx_reg->act) {
        result->point_num = 0;
        return 0;
    }

    for (result_index = 0; result_index < TOUCH_CHSC5XXX_MAX_POINTS; result_index++) {
        if (chsc5xxx_reg->pos[result_index].id != 0xFF) {
            finger_num++;
        }
    }

    if(finger_num > TOUCH_MAX_POINT_NUMBER) {
        LOG_W("CHSC5xxx touch point %d > max %d", finger_num, TOUCH_MAX_POINT_NUMBER);

        finger_num = TOUCH_MAX_POINT_NUMBER;
    }
    result->point_num = finger_num;

    if (finger_num) {
        for(result_index = 0, point_index = 0; result_index < finger_num; result_index++, point_index++) {
            point = &result->point[point_index];

            xh = chsc5xxx_reg->pos[result_index].xh & 0x0F;
            xl = chsc5xxx_reg->pos[result_index].xl;

            point_x = (xh << 8) | xl;
            if(point_x > dev->touch.range_x) {
                point_index--;
                continue;
            }

            yh = chsc5xxx_reg->pos[result_index].yh & 0x0F;
            yl = chsc5xxx_reg->pos[result_index].yl;
            
            point_y = (yh << 8) | yl;
            if(point_y > dev->touch.range_y) {
                point_index--;
                continue;
            }

            flg = (chsc5xxx_reg->pos[result_index].event & 0x0F) >> 2;
            id = chsc5xxx_reg->pos[result_index].id & 0x0F;

            point->event = event[flg];
            point->track_id = id;
            point->width = 0;
            point->x_coordinate = point_x;
            point->y_coordinate = point_y;
            point->timestamp = time;
        }
    }

    return 0;
}

static int reset(struct drv_touch_dev *dev) {
    if((0 <= dev->pin.rst) && (63 >= dev->pin.rst)) {
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
        rt_thread_mdelay(5);
        kd_pin_write(dev->pin.rst, dev->pin.rst_valid);
        rt_thread_mdelay(5);
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
        rt_thread_mdelay(100);
    }

    return 0;
}

static int get_default_rotate(struct drv_touch_dev *dev) {
    return RT_TOUCH_ROTATE_DEGREE_270;
}

int drv_touch_probe_chsc5xxx(struct drv_touch_dev *dev) {
    rt_uint8_t data[8];

    const char *chip_type[] = {
        /*00H*/ "CHSC5472",
        /*01H*/ "CHSC5448",
        /*02H*/ "CHSC5448A",
        /*03H*/ "CHSC5460",
        /*04H*/ "CHSC5468",
        /*05H*/ "CHSC5432",
        /*10H*/ "CHSC5816",
        /*11H*/ "CHSC1716",
    };

    dev->i2c.addr = 0x2e;

    rt_thread_mdelay(100);

    if (0 != chsc5xxx_read_reg(dev, 0x20000080, data, sizeof(data))) {
        return -1;
    }

    switch(data[0]) {
        case 0 ... 5:
            rt_strncpy(dev->dev.drv_name, chip_type[data[0]], sizeof(dev->dev.drv_name));
        break;
        case 0x10:
            rt_strncpy(dev->dev.drv_name, chip_type[6], sizeof(dev->dev.drv_name));
        break;
        case 0x11:
            rt_strncpy(dev->dev.drv_name, chip_type[7], sizeof(dev->dev.drv_name));
        break;
        default:
            rt_strncpy(dev->dev.drv_name, "CHSC5xxx", sizeof(dev->dev.drv_name));
        break;
    }

    dev->dev.read_register = read_register;
    dev->dev.parse_register = parse_register;
    dev->dev.reset = reset;
    dev->dev.get_default_rotate = get_default_rotate;

    return 0;
}
