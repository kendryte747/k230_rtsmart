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
#include "rtthread.h"
#include <stdint.h>

#define DBG_TAG          "ft5x16"
#define DBG_LVL          DBG_WARNING
#define DBG_COLOR
#include <rtdbg.h>

struct ft5x16_reg {
    uint8_t finger_num; // 0x02

    struct {
        uint8_t xh; // 0x03
        uint8_t xl;     // 0x04
        uint8_t yh; // 0x05
        uint8_t yl;     // 0x06
        uint8_t weight; //0x07
        uint8_t resv;
    } pos[5];
};

_Static_assert(sizeof(struct ft5x16_reg) < TOUCH_READ_REG_MAX_SIZE, "FT5x16 reg data size > TOUCH_READ_REG_MAX_SIZE");

// APIs ///////////////////////////////////////////////////////////////////////
static int read_register(struct drv_touch_dev *dev, struct touch_register *reg) {
    reg->time = rt_tick_get();

    return touch_dev_read_reg(dev, 0x02, (uint8_t *)&reg->reg[0], 0x1F - 0x02);
}

static int parse_register(struct drv_touch_dev *dev, struct touch_register *reg, struct touch_point *result) {
    const uint8_t event[4] = {RT_TOUCH_EVENT_DOWN, RT_TOUCH_EVENT_UP, RT_TOUCH_EVENT_MOVE, RT_TOUCH_EVENT_NONE};

    int finger_num;
    uint8_t xh, xl, yh, yl, flg, id, wight;
    uint16_t point_x, point_y;
    int result_index = 0, point_index = 0;

    rt_tick_t time = reg->time;

    struct rt_touch_data *point = NULL;
    struct ft5x16_reg *ft5x16_reg = (struct ft5x16_reg *)reg->reg;

    finger_num = ft5x16_reg->finger_num & 0x0F;

    if(finger_num > 5) {
        result->point_num = 0;
        return 0;
    }

    if(finger_num > TOUCH_MAX_POINT_NUMBER) {
        LOG_W("FT5x16 touch point %d > max %d", finger_num, TOUCH_MAX_POINT_NUMBER);

        finger_num = TOUCH_MAX_POINT_NUMBER;
    }
    result->point_num = finger_num;

    if(finger_num) {
        for(result_index = 0, point_index = 0; result_index < finger_num; result_index++, point_index++) {
            point = &result->point[point_index];

            xh = ft5x16_reg->pos[result_index].xh & 0x0F;
            xl = ft5x16_reg->pos[result_index].xl;

            point_x = (xh << 8) | xl;
            if(point_x > dev->touch.range_x) {
                point_index--;
                continue;
            }

            yh = ft5x16_reg->pos[result_index].yh & 0x0F;
            yl = ft5x16_reg->pos[result_index].yl;

            point_y = (yh << 8) | yl;
            if(point_y > dev->touch.range_y) {
                point_index--;
                continue;
            }

            flg = ft5x16_reg->pos[result_index].xh >> 6;
            id = ft5x16_reg->pos[result_index].yh >> 4;

            wight = ft5x16_reg->pos[result_index].weight;

            point->event = event[flg];
            point->track_id = id;
            point->width = wight;
            point->x_coordinate = point_x;
            point->y_coordinate = point_y;
            point->timestamp = time;
        }
    }

    return 0;
}

static int reset(struct drv_touch_dev *dev) {
    if((0 <= dev->pin.rst) && (63 >= dev->pin.rst)) {
        kd_pin_write(dev->pin.rst, dev->pin.rst_valid);
        rt_thread_mdelay(10);
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
    }

    return 0;
}

static int get_default_rotate(struct drv_touch_dev *dev) {
    return RT_TOUCH_ROTATE_DEGREE_270;
}

int drv_touch_init_ft5x16(struct drv_touch_dev *dev) {
    dev->dev.read_register = read_register;
    dev->dev.parse_register = parse_register;
    dev->dev.reset = reset;
    dev->dev.get_default_rotate = get_default_rotate;

    return 0;
}
