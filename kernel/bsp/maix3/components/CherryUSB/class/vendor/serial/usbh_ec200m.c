#include "usbh_ec200m.h"

#define DEV_FORMAT "ttyUSB%d"

#define EC200M_MAX_TTY_INTF (3)
#define EC200M_FIFO_SIZE (512)
#define ALIGN_SIZE (64)
#define INTF_DISCONNECT (0xffff)

#define TTY_LOCK(serial)   rt_mutex_take(&(serial->lock), RT_WAITING_FOREVER)
#define TTY_UNLOCK(serial) rt_mutex_release(&(serial->lock))

static struct usbh_ec200m *g_ec200m[EC200M_MAX_TTY_INTF];
static uint8_t g_dev_inuse = 0;

static struct usbh_ec200m *get_usbh_ec200m(void)
{
    uint8_t devno;

    for (devno = 0; devno < EC200M_MAX_TTY_INTF; devno ++) {
        if ((g_dev_inuse & (1 << devno)) == 0) {
            g_dev_inuse |= (1 << devno);
            g_ec200m[devno] = rt_calloc(1, sizeof(struct usbh_ec200m));
            if (g_ec200m[devno] != NULL) {
                /* alloc ok ,assign devno */
                g_ec200m[devno]->minor = devno;
            }
            break;
        }
    }

    if (devno >= EC200M_MAX_TTY_INTF) {
        return NULL;
    } else {
        return g_ec200m[devno];
    }
}

static void put_usbh_ec200m(struct usbh_ec200m *serial)
{
    uint8_t devno = serial->minor;

    if (devno < EC200M_MAX_TTY_INTF) {
        rt_free(g_ec200m[devno]);
        g_ec200m[devno] = RT_NULL;
        g_dev_inuse &= ~(1 << devno);
    }
}

#ifdef RT_USING_POSIX

#include <dfs_poll.h>
#include <dfs_posix.h>

static int ec200m_fops_open(struct dfs_fd *fd)
{
    rt_err_t ret = 0;
    rt_uint16_t flags = 0;
    rt_device_t device;

    device = (rt_device_t)fd->fnode->data;
    RT_ASSERT(device != RT_NULL);

    switch (fd->flags & O_ACCMODE) {
    case O_RDONLY:
        flags = RT_DEVICE_FLAG_INT_RX;
        break;
    case O_WRONLY:
        flags = RT_DEVICE_FLAG_INT_TX;
        break;
    case O_RDWR:
        flags = RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX;
        break;
    default:
        rt_kprintf("fops open: unknown mode - %d!\n", fd->flags & O_ACCMODE);
        ret = -RT_EINVAL;
        goto out;
    }

    ret = rt_device_open(device, flags);
    if (ret != RT_EOK) {
        rt_kprintf("%s fail,ret = %d\n", __func__, ret);
        goto out;
    }

out:
    return ret;
}

static int ec200m_fops_close(struct dfs_fd *fd)
{
    rt_device_t device;

    device = (rt_device_t)fd->fnode->data;

    rt_device_close(device);

    return 0;
}

static int ec200m_fops_read(struct dfs_fd *fd, void *buf, size_t count)
{
    rt_device_t device;
    int size = 0, ret = RT_EOK;

    device = (rt_device_t)fd->fnode->data;

    do {
        size = rt_device_read(device, -1,  buf, count);
        if (size <= 0) {
            if (fd->flags & O_NONBLOCK) {
                break;
            }

            ret = rt_wqueue_wait_interruptible(&(device->wait_queue),
                                               0, RT_WAITING_FOREVER);
            if (device->user_data == (void *)INTF_DISCONNECT) {
                ret = -RT_EIO;
                break;
            }

            if (ret != RT_EOK) {
                break;
            }
        }
    } while (size <= 0);

    if (ret != RT_EOK) {
        size = ret;
    }

    return size;
}

static int ec200m_fops_write(struct dfs_fd *fd, const void *buf, size_t count)
{
    rt_device_t device;

    device = (rt_device_t)fd->fnode->data;

    return rt_device_write(device, -1, buf, count);

}

const static struct dfs_file_ops _ec200m_ops =
{
    ec200m_fops_open,
    ec200m_fops_close,
    RT_NULL,    /* ioclt */
    ec200m_fops_read,
    ec200m_fops_write,
    RT_NULL,    /* flush */
    RT_NULL,    /* lseek */
    RT_NULL,    /* getdents */
    RT_NULL     /* poll */
};
#endif

#ifdef RT_USING_DEVICE_OPS

void rt_wqueue_wakeup_all(rt_wqueue_t *queue, void *key)
{
    rt_base_t level;
    register int need_schedule = 0;

    rt_list_t *queue_list;
    struct rt_list_node *node;
    struct rt_wqueue_node *entry;

    queue_list = &(queue->waiting_list);

    level = rt_hw_interrupt_disable();
    /* set wakeup flag in the queue */
    queue->flag = RT_WQ_FLAG_WAKEUP;

retry:
    if (!(rt_list_isempty(queue_list)))
    {
        for (node = queue_list->next; node != queue_list; node = node->next)
        {
            entry = rt_list_entry(node, struct rt_wqueue_node, list);
            if (entry->wakeup(entry, key) == 0)
            {
                rt_thread_resume(entry->polling_thread);
                need_schedule = 1;

                rt_wqueue_remove(entry);
                goto retry;
            }
        }
    }
    rt_hw_interrupt_enable(level);

    if (need_schedule)
        rt_schedule();
}

static rt_err_t ec200m_fops_rx_ind(rt_device_t dev, rt_size_t size)
{
#ifdef RT_USING_POSIX
    rt_wqueue_wakeup_all(&(dev->wait_queue), (void*)POLLIN);
#else
    struct usbh_ec200m *serial = (struct usbh_ec200m *)dev;
    rt_sem_release(&(serial->rx_ind_sem));
#endif

    return RT_EOK;
}

static rt_err_t ec200m_init(struct rt_device *dev)
{
    struct usbh_ec200m *serial;

    RT_ASSERT(dev != RT_NULL);

    serial = (struct usbh_ec200m *)dev;
    serial->rx_fifo = RT_NULL;
    serial->tx_fifo = RT_NULL;
    serial->bufsz = EC200M_FIFO_SIZE;

    rt_sem_take(&(serial->wait_close), rt_tick_from_millisecond(1000));

    return RT_EOK;
}

void usbh_ec200m_indat_callback(void *arg, int nbytes)
{
    struct usbh_ec200m *serial = (struct usbh_ec200m *)arg;
    struct serial_rx_fifo *rx_fifo = serial->rx_fifo;
    rt_base_t level;

    if (nbytes > 0) {

        RT_ASSERT(rx_fifo != RT_NULL);

        if (rt_ringbuffer_space_len(&(rx_fifo->rb)) < nbytes) {
            rt_kprintf("have no enough buffer but force save,discard old data. (%d, %d)\n",
                       rt_ringbuffer_space_len(&(rx_fifo->rb)), nbytes);
        }

        level = rt_hw_interrupt_disable();
        rt_ringbuffer_put_force(&(rx_fifo->rb), serial->bulkin_buffer, nbytes);
        rt_hw_interrupt_enable(level);

#if 0
        for (int i = 0; i < nbytes; i++) {
            rt_kprintf("%c", serial->bulkin_buffer[i]);

        }
        rt_kprintf("\n");
#endif

        if (serial->device.rx_indicate != RT_NULL) {
            rt_size_t rx_length;

            level = rt_hw_interrupt_disable();
            rx_length = rt_ringbuffer_data_len(&(rx_fifo->rb));
            rt_hw_interrupt_enable(level);

            if (rx_length)
            {
                serial->device.rx_indicate(&serial->device, rx_length);
            }
        }

        usbh_submit_urb(&serial->bulkin_urb);
    } else if (nbytes == -USB_ERR_NAK) { /* for dwc2 */
        usbh_submit_urb(&serial->bulkin_urb);
    } else {
        USB_LOG_INFO("%s, ret = %d\n", __func__, nbytes);
    }
}

static int usbh_ec200m_bulk_in_transfer(struct usbh_ec200m *serial)
{
    int ret = 0;
    struct usbh_urb *urb = &serial->bulkin_urb;

    usbh_bulk_urb_fill(urb, serial->hport, serial->bulkin, serial->bulkin_buffer,
                       USB_GET_MAXPACKETSIZE(serial->bulkin->wMaxPacketSize),
                       0, (usbh_complete_callback_t)usbh_ec200m_indat_callback, serial);

    ret = usbh_submit_urb(urb);
    if (ret < 0) {
        rt_kprintf("submit bulkin urb fail, ret = %d, %s %d\n", ret, __func__, __LINE__);
    }

    return ret;
}

static rt_err_t ec200m_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_err_t ret = RT_EOK;
    struct usbh_ec200m *serial = (struct usbh_ec200m *)dev;

    RT_ASSERT(dev != RT_NULL);

    TTY_LOCK(serial);

#ifndef RT_USING_POSIX
    if (oflag == RT_DEVICE_OFLAG_RDWR) {
        oflag = RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX;
    }
#endif

    if (serial->rx_fifo == RT_NULL) {
        if (oflag & RT_DEVICE_FLAG_INT_RX) {
            struct serial_rx_fifo *rx_fifo;

            rx_fifo = (struct serial_rx_fifo *)rt_malloc(sizeof(struct serial_rx_fifo) + serial->bufsz);
            RT_ASSERT(rx_fifo != RT_NULL);

            rt_device_set_rx_indicate(dev, ec200m_fops_rx_ind);
            rt_ringbuffer_init(&(rx_fifo->rb), rx_fifo->buffer, serial->bufsz);
            rt_sem_init(&(rx_fifo->r_sem), "ec200m_r_sem", 1, RT_IPC_FLAG_FIFO);
            serial->rx_fifo = rx_fifo;
            usbh_ec200m_bulk_in_transfer(serial);
            dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
        }
    }

    if (serial->tx_fifo == RT_NULL) {
        if (oflag & RT_DEVICE_FLAG_INT_TX) {
            struct serial_tx_fifo *tx_fifo;

            tx_fifo = (struct serial_tx_fifo *)rt_malloc(sizeof(struct serial_tx_fifo));
            RT_ASSERT(tx_fifo != RT_NULL);
            rt_sem_init(&(tx_fifo->w_sem), "ec200m_w_sem", 1, RT_IPC_FLAG_FIFO);
            serial->tx_fifo = tx_fifo;
            dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
        }
    }

    TTY_UNLOCK(serial);
    return ret;
}

static rt_err_t ec200m_close(struct rt_device *dev)
{
    struct usbh_ec200m *serial = (struct usbh_ec200m *)dev;
    rt_err_t ret = RT_EOK;


    RT_ASSERT(dev != RT_NULL);
    if (dev->ref_count > 1) {
        goto out;
    }

    TTY_LOCK(serial);

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX) {
        struct serial_rx_fifo* rx_fifo;

        if (serial->bulkin) {
            usbh_kill_urb(&serial->bulkin_urb);
        }

        rt_device_set_rx_indicate(dev, NULL);
        rx_fifo = (struct serial_rx_fifo*)serial->rx_fifo;
        RT_ASSERT(rx_fifo != RT_NULL);

        rt_sem_detach(&(rx_fifo->r_sem));
        rt_free(rx_fifo);
        serial->rx_fifo = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX) {
        struct serial_tx_fifo* tx_fifo;

        if (serial->bulkout) {
            usbh_kill_urb(&serial->bulkout_urb);
        }

        tx_fifo = (struct serial_tx_fifo*)serial->tx_fifo;
        RT_ASSERT(tx_fifo != RT_NULL);

        rt_sem_detach(&(tx_fifo->w_sem));
        rt_free(tx_fifo);
        serial->tx_fifo = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
    }

    if (serial->intin) {
        usbh_kill_urb(&serial->intin_urb);
    }

    dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;

    TTY_UNLOCK(serial);

    rt_sem_release(&(serial->wait_close));

out:
    return ret;
}

rt_inline rt_size_t _ec200m_int_rx(struct usbh_ec200m *serial, rt_uint8_t *data, int length)
{
    rt_base_t level;
    rt_size_t rcv_len;
    struct serial_rx_fifo *rx_fifo;

    RT_ASSERT(serial != RT_NULL);
    rx_fifo = (struct serial_rx_fifo *)serial->rx_fifo;
    RT_ASSERT(rx_fifo != RT_NULL);

    rt_sem_take(&(rx_fifo->r_sem), RT_WAITING_FOREVER);
    level = rt_hw_interrupt_disable();
    rcv_len = rt_ringbuffer_get(&(rx_fifo->rb), data, length);
    rt_hw_interrupt_enable(level);
    rt_sem_release(&(rx_fifo->r_sem));

    return rcv_len;
}

static rt_size_t _ec200m_read(struct rt_device *dev, rt_off_t pos, void *buffer,
                             rt_size_t count)
{
    rt_size_t rlen = 0;
    struct usbh_ec200m *serial = (struct usbh_ec200m *)dev;

    RT_ASSERT(dev != RT_NULL);

    if (count == 0) {
        rt_kprintf("read zero size and return\n");
        goto out;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX) {
        rlen = _ec200m_int_rx(serial, buffer, count);
    }

out:
    return rlen;
}

static rt_size_t ec200m_read(struct rt_device *dev, rt_off_t pos, void *buffer,
                             rt_size_t count)
{
#ifdef RT_USING_POSIX
    return _ec200m_read(dev, pos, buffer, count);
#else
    int size = 0, ret = RT_EOK;
    struct usbh_ec200m *serial = (struct usbh_ec200m *)dev;

    do {
        size = _ec200m_read(dev, -1, buffer, count);
        if (size <= 0) {

            ret = rt_sem_take_interruptible(&(serial->rx_ind_sem), RT_WAITING_FOREVER);
            if (device->user_data == (void *)INTF_DISCONNECT) {
                ret = -RT_EIO;
                break;
            }

            if (ret != RT_EOK) {
                break;
            }
        }
    } while (size <= 0);

    if (ret != RT_EOK) {
        size = ret;
    }

    return size;
#endif
}

static int usbh_ec200m_bulk_out_transfer(struct usbh_ec200m *serial, uint8_t *buf,
                                         int32_t len, uint32_t timeout)
{
    int ret;
    struct usbh_urb *urb = &serial->bulkout_urb;
    int left, todo;

    left = len;
    do {

        todo = left;

        if (left > serial->bulkout->wMaxPacketSize) {
            todo = serial->bulkout->wMaxPacketSize;
        }

        rt_memcpy(serial->bulkout_buffer, buf, todo);

        usbh_bulk_urb_fill(urb, serial->hport, serial->bulkout, serial->bulkout_buffer,
                           todo, timeout, NULL, NULL);
        ret = usbh_submit_urb(urb);
        if (ret < 0) {
            rt_kprintf("blukout write fail, ret = %d\n", ret);
            goto out;
        } else {
            ret = urb->actual_length;
        }

        buf += todo;
        left -= todo;
    } while (left > 0);

out:
    return ret;
}

rt_inline rt_size_t _ec200m_int_tx(struct usbh_ec200m *serial, const rt_uint8_t *data,
                                   int length)
{
    rt_size_t wlen;
    struct serial_tx_fifo *tx_fifo;

    RT_ASSERT(serial != RT_NULL);
    tx_fifo = (struct serial_tx_fifo *)serial->tx_fifo;
    RT_ASSERT(tx_fifo != RT_NULL);

    rt_sem_take(&(tx_fifo->w_sem), RT_WAITING_FOREVER);
    /* wait write out with few second timeout, it mean we don't surpport nonblock write */
    wlen = usbh_ec200m_bulk_out_transfer(serial, (uint8_t *)data, length, 3000);
    rt_sem_release(&(tx_fifo->w_sem));

    return wlen;
}

static rt_size_t ec200m_write(struct rt_device *dev, rt_off_t pos, const void *buffer,
                              rt_size_t size)
{
    rt_size_t wlen = 0;
    struct usbh_ec200m *serial = (struct usbh_ec200m *)dev;
    RT_ASSERT(dev != RT_NULL);

    if (size == 0) {
        rt_kprintf("write zero size and return\n");
        goto out;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX) {
        wlen = _ec200m_int_tx(serial, (const rt_uint8_t *)buffer, size);
    }

out:
    return wlen;
}

const static struct rt_device_ops ec200m_ops =
{
    ec200m_init,
    ec200m_open,
    ec200m_close,
    ec200m_read,
    ec200m_write,
    RT_NULL
};
#endif

static int usbh_ec200m_connect(struct usbh_hubport *hport, uint8_t intf)
{
    struct usb_endpoint_descriptor *ep_desc;
    struct rt_device *device;
    int ret = 0;

    struct usbh_ec200m *serial = get_usbh_ec200m();
    if (serial == NULL) {
        rt_kprintf("Fail to get ec200m\n");
        ret = -USB_ERR_RANGE;
        goto err_out;
    }

    serial->hport = hport;
    serial->intf = intf;

    hport->config.intf[intf].priv = serial;

    for (uint8_t i = 0;
         i < hport->config.intf[intf].altsetting[0].intf_desc.bNumEndpoints;
         i ++) {

        ep_desc = &hport->config.intf[intf].altsetting[0].ep[i].ep_desc;
        if (USB_GET_ENDPOINT_TYPE(ep_desc->bmAttributes) == USB_ENDPOINT_TYPE_INTERRUPT) {
            USBH_EP_INIT(serial->intin, ep_desc);
        } else {
            if (ep_desc->bEndpointAddress & 0x80) {
                USBH_EP_INIT(serial->bulkin, ep_desc);
            } else {
                USBH_EP_INIT(serial->bulkout, ep_desc);
            }
        }
    }

    serial->bulkin_buffer = rt_malloc_align(serial->bulkin->wMaxPacketSize, ALIGN_SIZE);
    if (serial->bulkin_buffer == NULL) {
        ret = -USB_ERR_NOMEM;
        rt_kprintf("Fail to alloc bulk in buffer\n");
        goto free_class;
    }

    serial->bulkout_buffer = rt_malloc_align(serial->bulkout->wMaxPacketSize, ALIGN_SIZE);
    if (serial->bulkout_buffer == NULL) {
        ret = -USB_ERR_NOMEM;
        rt_kprintf("Fail to alloc bulk out buffer\n");
        goto free_inbuffer;
    }

    snprintf(hport->config.intf[intf].devname, CONFIG_USBHOST_DEV_NAMELEN,
             DEV_FORMAT, serial->minor);

    device = &(serial->device);
    device->type = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops = &ec200m_ops;
#else
    device->init = RT_NULL;
    device->open = ec200m_open;
    device->close = ec200m_close;
    device->read = ec200m_read;
    device->write = ec200m_write;
    device->control = RT_NULL;
#endif
    rt_sem_init(&(serial->wait_close), "ec200m_wait_close", 1, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&(serial->lock), "ec200m_mutex", RT_IPC_FLAG_PRIO);
    ret = rt_device_register(device, hport->config.intf[intf].devname, RT_DEVICE_FLAG_RDWR);
    if (ret) {
        rt_kprintf("connector device register fail\n");
        goto free_outbuffer;
    }

#ifdef RT_USING_POSIX
    device->fops = &_ec200m_ops;
#else
    rt_sem_init(&(serial->rx_ind_sem), "ec200m_wait_sem", 0, RT_IPC_FLAG_FIFO);
#endif

    USB_LOG_INFO("register ec200m class:%s = 0x%x\r\n", hport->config.intf[intf].devname, serial);

    return 0;

free_outbuffer:
    rt_free_align(serial->bulkout_buffer);
free_inbuffer:
    rt_free_align(serial->bulkin_buffer);
free_class:
    put_usbh_ec200m(serial);
err_out:

    return ret;
}

static int usbh_ec200m_disconnect(struct usbh_hubport *hport, uint8_t intf)
{
    int ret = 0;

    struct usbh_ec200m *serial = (struct usbh_ec200m *)hport->config.intf[intf].priv;

    if (serial) {
        /* Double kill urb have no side effect */
        if (serial->bulkin) {
            usbh_kill_urb(&serial->bulkin_urb);
        }

        if (serial->bulkout) {
            usbh_kill_urb(&serial->bulkout_urb);
        }

        if (serial->intin) {
            usbh_kill_urb(&serial->intin_urb);
        }

        if (serial->device.rx_indicate != RT_NULL) {
            serial->device.user_data = (void *)INTF_DISCONNECT;
            serial->device.rx_indicate(&serial->device, 0);
        }

        if (serial->device.ref_count > 0) {
            ret = rt_sem_take(&(serial->wait_close), rt_tick_from_millisecond(1000));
            if (ret != RT_EOK) {
                USB_LOG_ERR("wait ec200m_close fail ret = %d\n", ret);
            }
        }

        rt_device_unregister(&serial->device);

        rt_mutex_detach(&(serial->lock));
        rt_sem_detach(&(serial->wait_close));
#ifndef RT_USING_POSIX
        rt_sem_detach(&(serial->rx_ind_sem));
#endif
        rt_free_align(serial->bulkin_buffer);
        rt_free_align(serial->bulkout_buffer);
        put_usbh_ec200m(serial);

        if (hport->config.intf[intf].devname[0] != '\0') {
            USB_LOG_INFO("unregister ec200m class:%s = 0x%x\r\n", hport->config.intf[intf].devname, serial);
        }
    }

    return ret;
}

const struct usbh_class_driver ec200m_class_driver = {
    .driver_name = "ec200m",
    .connect = usbh_ec200m_connect,
    .disconnect = usbh_ec200m_disconnect
};

CLASS_INFO_DEFINE const struct usbh_class_info ec200m_class_info = {
    .match_flags = USB_CLASS_MATCH_VENDOR | USB_CLASS_MATCH_PRODUCT | USB_CLASS_MATCH_INTF_CLASS,
    .class = 0xff,
    .subclass = 0xff,
    .protocol = 0xff,
    .vid = 0x2c7c,
    .pid = 0x6002,
    .class_driver = &ec200m_class_driver
};

