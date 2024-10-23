#ifndef __USBH_EC200M_H__
#define __USBH_EC200M_H__

#include <rthw.h>
#include <rtthread.h>
#include "ipc/completion.h"
#include "ipc/ringbuffer.h"
#include "usb_osal.h"
#include "usbh_core.h"

struct serial_rx_fifo
{
    struct rt_ringbuffer rb;
    struct rt_semaphore r_sem;
    rt_uint8_t buffer[];
};

struct serial_tx_fifo
{
    struct rt_semaphore w_sem;
};

struct usbh_ec200m {
    struct rt_device device;
    struct usbh_hubport *hport;
    struct usb_endpoint_descriptor *intin;
    struct usb_endpoint_descriptor *bulkin;
    struct usb_endpoint_descriptor *bulkout;
    struct usbh_urb bulkout_urb;
    rt_uint8_t *bulkout_buffer;
    struct usbh_urb bulkin_urb;
    rt_uint8_t *bulkin_buffer;
    struct usbh_urb intin_urb; /* not surpport now */

    rt_uint16_t bufsz;
    struct serial_rx_fifo *rx_fifo;
    struct serial_tx_fifo *tx_fifo;
    struct rt_mutex lock;
#ifndef RT_USING_POSIX
    struct rt_semaphore rx_ind_sem;
#endif

    uint8_t intf;
    uint8_t minor;
    struct rt_semaphore wait_close;
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif
