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
#include "rtthread.h"
#include <stdint.h>

#define DBG_TAG          "cst128"
#define DBG_LVL          DBG_WARNING
#define DBG_COLOR
#include <rtdbg.h>

struct cst128_reg {
    uint8_t finger_num; // 0x02

    struct {
        uint8_t xh;     // 0x03
        uint8_t xl;     // 0x04
        uint8_t yh;     // 0x05
        uint8_t yl;     // 0x06
        uint8_t resv[2];
    } pos[5];
};

_Static_assert(sizeof(struct cst128_reg) < TOUCH_READ_REG_MAX_SIZE, "CST128 reg data size > TOUCH_READ_REG_MAX_SIZE");

// APIs ///////////////////////////////////////////////////////////////////////
static int read_register(struct drv_touch_dev *dev, struct touch_register *reg) {
    reg->time = rt_tick_get();

    return touch_dev_read_reg(dev, 0x02, (uint8_t *)&reg->reg[0], 0x1F - 0x02);
}

static int parse_register(struct drv_touch_dev *dev, struct touch_register *reg, struct touch_point *result) {
    int finger_num;
    uint8_t xh, xl, yh, yl;
    uint16_t point_x, point_y;
    int result_index, point_index;
    rt_tick_t time = reg->time;

    struct rt_touch_data *point = NULL;
    struct cst128_reg *cst128_reg = (struct cst128_reg *)reg->reg;

    finger_num = cst128_reg->finger_num & 0x0F;
    if(finger_num > 5) {
        result->point_num = 0;
        return 0;
    }

    if(finger_num > TOUCH_MAX_POINT_NUMBER) {
        LOG_W("CST128 touch point %d > max %d", finger_num, TOUCH_MAX_POINT_NUMBER);

        finger_num = TOUCH_MAX_POINT_NUMBER;
    }
    result->point_num = finger_num;

    if(finger_num) {
        for(result_index = 0, point_index = 0; result_index < finger_num; result_index++, point_index++) {
            point = &result->point[point_index];

            xh = cst128_reg->pos[result_index].xh;
            xl = cst128_reg->pos[result_index].xl;

            point_x = ((xh & 0x0F) << 8) | xl;
            if(point_x > dev->touch.range_x) {
                point_index--;
                continue;
            }

            yh = cst128_reg->pos[result_index].yh;
            yl = cst128_reg->pos[result_index].yl;

            point_y = ((yh & 0x0F) << 8) | yl;
            if(point_y > dev->touch.range_y) {
                point_index--;
                continue;
            }

            point->event = RT_TOUCH_EVENT_NONE;
            point->track_id = result_index;
            point->width = finger_num;
            point->x_coordinate = point_x;
            point->y_coordinate = point_y;
            point->timestamp = time;
        }
    }

    touch_dev_update_event(finger_num, point);

    return 0;
}

static int reset(struct drv_touch_dev *dev) {
    if((0 <= dev->pin.rst) && (63 >= dev->pin.rst)) {
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
        rt_thread_mdelay(20);
        kd_pin_write(dev->pin.rst, dev->pin.rst_valid);
        rt_thread_mdelay(10);
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
        rt_thread_mdelay(50);
    }

    return 0;
}

static int get_default_rotate(struct drv_touch_dev *dev) {
    return RT_TOUCH_ROTATE_SWAP_XY;
}

int drv_touch_probe_cst128(struct drv_touch_dev *dev) {
    uint8_t chip_id;
    struct touch_register reg_data;

#if defined CONFIG_BOARD_K230_CANMV_LCKFB
    rt_thread_mdelay(50);
#endif

    dev->i2c.addr = 0x38;

    if(0x00 != touch_dev_read_reg(dev, 0xA3, &chip_id, 1)) {
        LOG_E("%s->%d\n", __func__, __LINE__);
        return -2;
    }

#if defined CONFIG_BOARD_K230_CANMV_01STUDIO
    if(0x64 != chip_id) {
#elif defined CONFIG_BOARD_K230_CANMV_LCKFB
    if(0x54 != chip_id) {
#endif
        LOG_E("cst128 id error, %02X", chip_id);
        return -3;
    }

    rt_strncpy(dev->dev.drv_name, "cst128", sizeof(dev->dev.drv_name));

    dev->dev.read_register = read_register;
    dev->dev.parse_register = parse_register;
    dev->dev.reset = reset;
    dev->dev.get_default_rotate = get_default_rotate;

#if defined CONFIG_BOARD_K230_CANMV_01STUDIO
    dev->touch.range_x = 480;
    dev->touch.range_y = 640;
#endif

    return 0;
}
