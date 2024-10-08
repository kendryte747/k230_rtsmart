#include "usbh_core.h"
#include "usbh_hid.h"

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_buffer[128];

void usbh_hid_callback(void *arg, int nbytes)
{
    struct usbh_hid *hid_class = (struct usbh_hid *)arg;

    if (nbytes > 0) {
        for (size_t i = 0; i < nbytes; i++) {
            USB_LOG_RAW("0x%02x ", hid_buffer[i]);
        }
        USB_LOG_RAW("nbytes:%d\r\n", nbytes);
        usbh_submit_urb(&hid_class->intin_urb);
    } else if (nbytes == -USB_ERR_NAK) { /* for dwc2 */
        usbh_submit_urb(&hid_class->intin_urb);
    } else {
    }
}

#define TOTAL_LOG_CNG (20)
static void usbh_hid_thread(void *argument)
{
    int ret;
    struct usbh_hid *hid_class = (struct usbh_hid *)argument;

    // clang-format off
find_class:
    // clang-format on

#if 0
    while ((hid_class = (struct usbh_hid *)usbh_find_class_instance("/dev/input0")) == NULL) {
        goto delete;
        rt_kprintf("class have no found\n") ;
    }
#endif

    rt_kprintf("int urb hid speed = %d\n", hid_class->hport->speed);
    usbh_int_urb_fill(&hid_class->intin_urb, hid_class->hport, hid_class->intin, hid_buffer, hid_class->intin->wMaxPacketSize, 0, usbh_hid_callback, hid_class);
    ret = usbh_submit_urb(&hid_class->intin_urb);
    if (ret < 0) {
        rt_kprintf("urb ret = %d\n", ret);
        goto find_class;
    }
    // clang-format off
delete:
    usb_osal_thread_delete(NULL);
    // clang-format on
}

void usbh_hid_run(struct usbh_hid *hid_class)
{
    if (hid_class == (struct usbh_hid *)usbh_find_class_instance("/dev/input0")) {
            rt_kprintf("%s %d\n", __func__, __LINE__);
            usb_osal_thread_create("usbh_hid", 2048, CONFIG_USBHOST_PSC_PRIO + 1, usbh_hid_thread, hid_class);
    }
}
