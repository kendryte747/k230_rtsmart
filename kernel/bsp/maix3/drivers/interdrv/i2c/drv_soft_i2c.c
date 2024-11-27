#include "rtdef.h"
#include <rtthread.h>
#include <rtservice.h>

#include <ioremap.h>
#include <lwp_user_mm.h>
#include <stdint.h>

#include "tick.h"

#include "drv_gpio.h"
#include "drv_fpioa.h"

#define DBG_TAG          "soft_i2c"
#define DBG_LVL          DBG_WARNING
#define DBG_COLOR
#include <rtdbg.h>

struct soft_i2c_priv_t {
    int index;
    uint32_t freq;
    uint32_t timeout;

    int sda_mode, scl_mode; // 0: input, 1: output, default set to -1
    int sda_pin, scl_pin;

    struct rt_i2c_bus_device dev;
    struct rt_i2c_bit_ops dev_ops;

    struct rt_mutex mutex;
    rt_slist_t list;
};

static void config_pin_use_as_soft_i2c(int pin) {
    struct st_iomux_reg_t reg;

    reg.u.value = fpioa_get_pin_cfg(pin);

    reg.u.bit.st = 1;
    reg.u.bit.ds = 15;
    reg.u.bit.pd = 0;
    reg.u.bit.pu = 0;
    reg.u.bit.oe = 1;
    reg.u.bit.ie = 1;
    // don't change reg.u.bit.msc
    reg.u.bit.io_sel = 0;

    fpioa_set_pin_cfg(pin, reg.u.value);
}

static inline uint64_t get_tick(void)
{
    uint64_t cnt;

    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(cnt));

    return cnt;
}

static void soft_i2c_udelay(rt_uint32_t us) {
    uint64_t delay = (TIMER_CLK_FREQ / 1000000) * us;
    volatile uint64_t cur_time = get_tick();

    while(1) {
        if((get_tick() - cur_time ) >= delay)
            break;
    }
}

static void soft_i2c_set_sda(void *data, rt_int32_t state) {
    struct st_iomux_reg_t reg;
    struct soft_i2c_priv_t *priv = (struct soft_i2c_priv_t *)data;

    int sda_pin = priv->sda_pin;

    if(0x01 != priv->sda_mode) {
        priv->sda_mode = 1;

        reg.u.value = fpioa_get_pin_cfg(sda_pin);
        reg.u.bit.oe = 1;
        fpioa_set_pin_cfg(sda_pin, reg.u.value);

        kd_pin_mode(sda_pin, GPIO_DM_OUTPUT);
    }

    kd_pin_write(sda_pin, state);
}

static rt_int32_t soft_i2c_get_sda(void *data) {
    struct st_iomux_reg_t reg;
    struct soft_i2c_priv_t *priv = (struct soft_i2c_priv_t *)data;

    int sda_pin = priv->sda_pin;

    if(0x00 != priv->sda_mode) {
        priv->sda_mode = 0;

        reg.u.value = fpioa_get_pin_cfg(sda_pin);
        reg.u.bit.oe = 0;
        fpioa_set_pin_cfg(sda_pin, reg.u.value);

        kd_pin_mode(sda_pin, GPIO_DM_INPUT);
    }

    return kd_pin_read(sda_pin);
}

static void soft_i2c_set_scl(void *data, rt_int32_t state) {
    struct st_iomux_reg_t reg;
    struct soft_i2c_priv_t *priv = (struct soft_i2c_priv_t *)data;

    int scl_pin = priv->scl_pin;

    if(0x01 != priv->scl_mode) {
        priv->scl_mode = 1;

        reg.u.value = fpioa_get_pin_cfg(scl_pin);
        reg.u.bit.oe = 1;
        fpioa_set_pin_cfg(scl_pin, reg.u.value);

        kd_pin_mode(scl_pin, GPIO_DM_OUTPUT);
    }

    kd_pin_write(scl_pin, state);
}

static rt_int32_t soft_i2c_get_scl(void *data) {
    struct st_iomux_reg_t reg;
    struct soft_i2c_priv_t *priv = (struct soft_i2c_priv_t *)data;

    int scl_pin = priv->scl_pin;

    if(0x00 != priv->scl_mode) {
        priv->scl_mode = 0;

        reg.u.value = fpioa_get_pin_cfg(scl_pin);
        reg.u.bit.oe = 0;
        fpioa_set_pin_cfg(scl_pin, reg.u.value);

        kd_pin_mode(scl_pin, GPIO_DM_INPUT);
    }

    return kd_pin_read(scl_pin);
}

static uint32_t soft_i2c_freq_to_delay(uint32_t freq)
{
    if (freq == 0) {
        return 0;
    }

    uint32_t delay_us = (1000000U + freq - 1) / freq / 2;

    return delay_us;
}

static rt_slist_t _kd_soft_i2c_dev_list = RT_SLIST_OBJECT_INIT(_kd_soft_i2c_dev_list);

int rt_soft_i2c_del_dev(int bus_num)
{
    int result = 0;
    char dev_name[32];
    rt_device_t dev = RT_NULL;
    rt_slist_t *node = RT_NULL;
    struct soft_i2c_priv_t *priv = NULL;

    if(4 >= bus_num) {
        LOG_E("don't support remove i2c%d \n", bus_num);
        return -2;
    }

    rt_kprintf("Delete soft i2c bus %d\n", bus_num);

    rt_slist_for_each(node, &_kd_soft_i2c_dev_list) {
        priv = rt_slist_entry(node, struct soft_i2c_priv_t, list);

        if(priv->index == bus_num) {
            snprintf(dev_name, sizeof(dev_name), "i2c%d", bus_num);

            if(RT_EOK != rt_mutex_take(&priv->mutex, rt_tick_from_millisecond(10))) {
                LOG_E("take mutex failed");
                return -2;
            }

            if(NULL == (dev = rt_device_find(dev_name))) {
                result = -2;

                rt_kprintf("can't find /dev/%s", dev_name);
            } else {
                result = 0;

                rt_mutex_detach(&priv->dev.lock);
                rt_device_unregister(dev);
            }

            rt_mutex_release(&priv->mutex);

            rt_mutex_detach(&priv->mutex);

            rt_slist_remove(&_kd_soft_i2c_dev_list, &priv->list);

            rt_free(priv);

            return result;
        }
    }

    return -1;
}

int rt_soft_i2c_add_dev(int bus_num, int scl, int sda, uint32_t freq, uint32_t timeout)
{
    rt_slist_t *node = RT_NULL;
    struct soft_i2c_priv_t *priv = NULL;

    char name[RT_NAME_MAX];

    if(4 >= bus_num) {
        LOG_E("don't support add i2c%d\n", bus_num);
        return -2;
    }

    rt_slist_for_each(node, &_kd_soft_i2c_dev_list) {
        priv = rt_slist_entry(node, struct soft_i2c_priv_t, list);

        if(priv->index == bus_num) {
            if((priv->scl_pin != scl) || (priv->sda_pin != sda) || (priv->freq != freq) || (priv->timeout != timeout)) {
                LOG_I("update soft_i2c%d configure.", bus_num);

                if(RT_EOK != rt_mutex_take(&priv->mutex, rt_tick_from_millisecond(10))) {
                    LOG_E("take mutex failed");
                    return -1;
                }

                priv->scl_pin = scl;
                priv->sda_pin = sda;
                priv->freq = freq;
                priv->timeout = timeout;

                priv->dev_ops.timeout = timeout;
                priv->dev_ops.delay_us = soft_i2c_freq_to_delay(freq);

                rt_mutex_release(&priv->mutex);

                rt_kprintf("Update soft i2c bus %d, scl %d, sda %d, freq %u, timeout %u, delay %d\n", bus_num, scl, sda, freq, timeout, priv->dev_ops.delay_us);
            }

            return 0;
        }
    }

    if(NULL == (priv = rt_malloc(sizeof(*priv)))) {
        LOG_E("malloc failed.");
        return -2;
    }

    priv->index = bus_num;
    priv->scl_pin = scl;
    priv->sda_pin = sda;
    priv->freq = freq;
    priv->timeout = timeout;

    config_pin_use_as_soft_i2c(scl);
    config_pin_use_as_soft_i2c(sda);

    // ops
    memset(&priv->dev_ops, 0, sizeof(priv->dev_ops));

    priv->dev_ops.set_sda = soft_i2c_set_sda,
    priv->dev_ops.get_sda = soft_i2c_get_sda,

    priv->dev_ops.set_scl = soft_i2c_set_scl,
    priv->dev_ops.get_scl = soft_i2c_get_scl,

    priv->dev_ops.udelay = soft_i2c_udelay,

    priv->dev_ops.timeout = timeout;
    priv->dev_ops.delay_us = soft_i2c_freq_to_delay(freq);

    priv->dev_ops.data = priv;

    // dev
    memset(&priv->dev, 0, sizeof(priv->dev));

    priv->dev.priv = &priv->dev_ops;
    priv->dev.retries = 3;

    // rt_snprintf(name, sizeof(name), "soft_i2c%d", bus_num);
    rt_snprintf(name, sizeof(name), "i2c%d", bus_num);

    if(RT_EOK != rt_i2c_bit_add_bus(&priv->dev, name)) {
        LOG_E("Add bus failed");

        rt_free(priv);

        return -3;
    }

    rt_mutex_init(&priv->mutex, name, RT_IPC_FLAG_PRIO);

    rt_slist_init(&priv->list);
    rt_slist_insert(&_kd_soft_i2c_dev_list, &priv->list);

    rt_kprintf("Add new soft i2c bus %d, scl %d, sda %d, freq %u, timeout %u, delay %d\n", bus_num, scl, sda, freq, timeout, priv->dev_ops.delay_us);

    return 0;
}
