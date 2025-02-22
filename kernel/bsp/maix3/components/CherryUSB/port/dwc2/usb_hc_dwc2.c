/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbh_core.h"
#include "usbh_hub.h"
#include "usb_dwc2_reg.h"
#include <rtthread.h>
#include <rthw.h>
#include "board.h"
#include <cache.h>

#define USB_BUF_ALIGN_SIZE (16*1024)
#ifndef CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USBHOST_PIPE_NUM 12
#endif

/* largest non-periodic USB packet used / 4 */
#ifndef CONFIG_USB_DWC2_NPTX_FIFO_SIZE
#define CONFIG_USB_DWC2_NPTX_FIFO_SIZE (512 / 4)
#endif

/* largest periodic USB packet used / 4 */
#ifndef CONFIG_USB_DWC2_PTX_FIFO_SIZE
#define CONFIG_USB_DWC2_PTX_FIFO_SIZE (1024 / 4)
#endif

/*  
(largest USB packet used / 4) + 1 for status information + 1 transfer complete + 
1 location each for Bulk/Control endpoint for handling NAK/NYET scenario 
*/
#ifndef CONFIG_USB_DWC2_RX_FIFO_SIZE
#define CONFIG_USB_DWC2_RX_FIFO_SIZE ((1012 - CONFIG_USB_DWC2_NPTX_FIFO_SIZE - CONFIG_USB_DWC2_PTX_FIFO_SIZE) / 4)
#endif

#define USB_OTG_GLB     ((USB_OTG_GlobalTypeDef *)(bus->hcd.reg_base))
#define USB_OTG_PCGCCTL *(__IO uint32_t *)((uint32_t)bus->hcd.reg_base + USB_OTG_PCGCCTL_BASE)
#define USB_OTG_HPRT    *(__IO uint32_t *)((uint32_t)bus->hcd.reg_base + USB_OTG_HOST_PORT_BASE)
#define USB_OTG_HOST    ((USB_OTG_HostTypeDef *)(bus->hcd.reg_base + USB_OTG_HOST_BASE))
#define USB_OTG_HC(i)   ((USB_OTG_HostChannelTypeDef *)(bus->hcd.reg_base + USB_OTG_HOST_CHANNEL_BASE + ((i)*USB_OTG_HOST_CHANNEL_SIZE)))
#define USB_OTG_FIFO(i) *(__IO uint32_t *)(bus->hcd.reg_base + USB_OTG_FIFO_BASE + ((i)*USB_OTG_FIFO_SIZE))

struct dwc2_chan {
    uint8_t ep0_state;
    uint16_t num_packets;
    uint32_t xferlen;
    uint8_t chidx;
    bool inuse;
    usb_osal_sem_t waitsem;
    struct usbh_urb *urb;
    uint32_t iso_frame_idx;
    uint8_t *buf;
    uint8_t *buf_align;
    uint8_t align_used;
    int32_t do_split;
    int32_t complete_split;
    uint8_t hub_addr;
    uint8_t hub_port;
    uint16_t ssplit_frame;
};

struct dwc2_hcd {
    volatile bool port_csc;
    volatile bool port_pec;
    volatile bool port_occ;
    struct dwc2_chan chan_pool[CONFIG_USBHOST_PIPE_NUM];
} g_dwc2_hcd[CONFIG_USBHOST_MAX_BUS];

#define DWC2_EP0_STATE_SETUP     0
#define DWC2_EP0_STATE_INDATA    1
#define DWC2_EP0_STATE_OUTDATA   2
#define DWC2_EP0_STATE_INSTATUS  3
#define DWC2_EP0_STATE_OUTSTATUS 4

static inline int dwc2_reset(struct usbh_bus *bus)
{
    volatile uint32_t count = 0U;
    uint32_t greset;
    
    USB_OTG_GLB->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    /* Wait for CSftRstDone. */
    do {
        if (++count > 200000U) {
            USB_LOG_INFO("0USB_OTG_GLB->GRSTCTL = 0x%x\n", USB_OTG_GLB->GRSTCTL);
            return -1;
        }
    } while ((USB_OTG_GLB->GRSTCTL & (0x1 << 29)) == 0U);
    
    greset = USB_OTG_GLB->GRSTCTL;
    greset &= ~0x1;
    greset |= (0x1 << 29);
    USB_OTG_GLB->GRSTCTL = greset;

    /* Wait for AHB master IDLE state. */
    do {
        if (++count > 200000U) {
            return -1;
        }
    } while ((USB_OTG_GLB->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

    return 0;
}

static inline int dwc2_core_init(struct usbh_bus *bus)
{
    int ret;
#if defined(CONFIG_USB_HS)
    /* Init The ULPI Interface */
    USB_OTG_GLB->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    /* Select vbus source */
    USB_OTG_GLB->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

    //USB_OTG_GLB->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    /* Reset after a PHY select */
    ret = dwc2_reset(bus);
#else
    /* Select FS Embedded PHY */
    USB_OTG_GLB->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
    /* Reset after a PHY select */
    ret = dwc2_reset(bus);
#endif
    return ret;
}

static inline void dwc2_set_mode(struct usbh_bus *bus, uint8_t mode)
{
    USB_OTG_GLB->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);

    if (mode == USB_OTG_MODE_HOST) {
        USB_OTG_GLB->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;
    } else if (mode == USB_OTG_MODE_DEVICE) {
        USB_OTG_GLB->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    }
}

static inline int dwc2_flush_rxfifo(struct usbh_bus *bus)
{
    volatile uint32_t count = 0;

    USB_OTG_GLB->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

    do {
        if (++count > 200000U) {
            return -1;
        }
    } while ((USB_OTG_GLB->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

    return 0;
}

static inline int dwc2_flush_txfifo(struct usbh_bus *bus, uint32_t num)
{
    volatile uint32_t count = 0U;

    USB_OTG_GLB->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));

    do {
        if (++count > 200000U) {
            return -1;
        }
    } while ((USB_OTG_GLB->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

    return 0;
}

static inline void dwc2_drivebus(struct usbh_bus *bus, uint8_t state)
{
    __IO uint32_t hprt0 = 0U;

    hprt0 = USB_OTG_HPRT;

    hprt0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |
               USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG);

    if (((hprt0 & USB_OTG_HPRT_PPWR) == 0U) && (state == 1U)) {
        USB_OTG_HPRT = (USB_OTG_HPRT_PPWR | hprt0);
    }
    if (((hprt0 & USB_OTG_HPRT_PPWR) == USB_OTG_HPRT_PPWR) && (state == 0U)) {
        USB_OTG_HPRT = ((~USB_OTG_HPRT_PPWR) & hprt0);
    }
}

static inline uint8_t usbh_get_port_speed(struct usbh_bus *bus, const uint8_t port)
{
    __IO uint32_t hprt0 = 0U;
    uint8_t speed;

    hprt0 = USB_OTG_HPRT;

    speed = (hprt0 & USB_OTG_HPRT_PSPD) >> 17;

    if (speed == HPRT0_PRTSPD_HIGH_SPEED) {
        return USB_SPEED_HIGH;
    } else if (speed == HPRT0_PRTSPD_FULL_SPEED) {
        return USB_SPEED_FULL;
    } else if (speed == HPRT0_PRTSPD_LOW_SPEED) {
        return USB_SPEED_LOW;
    } else {
        return USB_SPEED_UNKNOWN;
    }
}

static inline void dwc2_chan_char_init(struct usbh_bus *bus, uint8_t ch_num, uint8_t devaddr, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps, uint8_t speed)
{
    uint32_t regval;

    /* Program the HCCHAR register */
    regval = (((uint32_t)ep_mps << USB_OTG_HCCHAR_MPSIZ_Pos) & USB_OTG_HCCHAR_MPSIZ) |
             ((((uint32_t)ep_addr & 0x7FU) << USB_OTG_HCCHAR_EPNUM_Pos) & USB_OTG_HCCHAR_EPNUM) |
             (((uint32_t)ep_type << USB_OTG_HCCHAR_EPTYP_Pos) & USB_OTG_HCCHAR_EPTYP) |
             (((uint32_t)devaddr << USB_OTG_HCCHAR_DAD_Pos) & USB_OTG_HCCHAR_DAD);

    if ((ep_addr & 0x80U) == 0x80U) {
        regval |= USB_OTG_HCCHAR_EPDIR;
    }

    /* LS device plugged to HUB */
    if ((speed == USB_SPEED_LOW) && (usbh_get_port_speed(bus, 0) != USB_SPEED_LOW)) {
        regval |= USB_OTG_HCCHAR_LSDEV;
    }

    if (ep_type == USB_ENDPOINT_TYPE_INTERRUPT) {
        regval |= USB_OTG_HCCHAR_ODDFRM;
    }

    USB_OTG_HC((uint32_t)ch_num)->HCCHAR = regval;
}

static inline void dwc2_chan_splt_init(struct usbh_bus *bus, uint8_t ch_num)
{
    struct dwc2_chan *chan;
    uint32_t hcsplt;

    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[ch_num];

    if (chan->do_split) {
        hcsplt = USB_OTG_HCSPLT_SPLITEN;
        hcsplt |= (chan->hub_addr << USB_OTG_HCSPLT_HUBADDR_Pos);
        hcsplt |= chan->hub_port;

        if (chan->complete_split) {
            hcsplt |= USB_OTG_HCSPLT_COMPLSPLT;
        } else {
            hcsplt &= ~USB_OTG_HCSPLT_COMPLSPLT;
        }

        USB_OTG_HC((uint32_t)ch_num)->HCSPLT = hcsplt;
    }
}

static void dwc2_chan_init(struct usbh_bus *bus, uint8_t ch_num, uint8_t devaddr, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps, uint8_t speed)
{
    uint32_t regval;

    /* Clear old interrupt conditions for this host channel. */
    USB_OTG_HC((uint32_t)ch_num)->HCINT = 0xFFFFFFFFU;

    /* Enable channel interrupts required for this transfer. */
    regval = USB_OTG_HCINTMSK_CHHM;

    if (ep_type == USB_ENDPOINT_TYPE_INTERRUPT) {
        regval |= USB_OTG_HCINTMSK_NAKM;
    }

    USB_OTG_HC((uint32_t)ch_num)->HCINTMSK = regval;

    /* Enable the top level host channel interrupt. */
    USB_OTG_HOST->HAINTMSK |= 1UL << (ch_num & 0xFU);

    dwc2_chan_char_init(bus, ch_num, devaddr, ep_addr, ep_type, ep_mps, speed);
    dwc2_chan_splt_init(bus, ch_num);
}

/* For IN channel HCTSIZ.XferSize is expected to be an integer multiple of ep_mps size.*/
static inline void dwc2_chan_transfer(struct usbh_bus *bus, uint8_t ch_num, uint8_t ep_addr, uint32_t *buf, uint32_t size, uint8_t num_packets, uint8_t pid)
{
    __IO uint32_t tmpreg;
    uint8_t is_oddframe;
    struct dwc2_chan *chan;
    struct usbh_urb *urb;
    
    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[ch_num];
    urb = chan->urb;

    /* Initialize the HCTSIZn register */
    USB_OTG_HC(ch_num)->HCTSIZ = (size & USB_OTG_HCTSIZ_XFRSIZ) |
                                 (((uint32_t)num_packets << 19) & USB_OTG_HCTSIZ_PKTCNT) |
                                 (((uint32_t)pid << 29) & USB_OTG_HCTSIZ_DPID);

    chan->buf = buf;

    if(((uint64_t)buf & 0x3f != 0) || (size%64 != 0))
    {
        if(size > USB_BUF_ALIGN_SIZE) {
            USB_LOG_ERR("need more buf_align size(%d),please change USB_BUF_ALIGN_SIZE\r\n", size);
            while (1) {
            }
        }
        if ((ep_addr & 0x80) == 0x00U) {
            usb_memcpy(chan->buf_align, chan->buf, size);
        }
        rt_hw_cpu_dcache_clean((void *)chan->buf_align, size);
        /* xfer_buff MUST be 32-bits aligned */
        USB_OTG_HC(ch_num)->HCDMA = (uint32_t)chan->buf_align;
        chan->align_used = 1;
    }
    else {
        rt_hw_cpu_dcache_clean((void *)chan->buf, size);
        /* xfer_buff MUST be 32-bits aligned */
        USB_OTG_HC(ch_num)->HCDMA = (uint32_t)chan->buf;
        chan->align_used = 0;
    }

    is_oddframe = (((uint32_t)USB_OTG_HOST->HFNUM & 0x01U) != 0U) ? 0U : 1U;
    USB_OTG_HC(ch_num)->HCCHAR &= ~USB_OTG_HCCHAR_ODDFRM;
    USB_OTG_HC(ch_num)->HCCHAR |= (uint32_t)is_oddframe << 29;

    /* Set host channel enable */
    tmpreg = USB_OTG_HC(ch_num)->HCCHAR;
    tmpreg &= ~USB_OTG_HCCHAR_CHDIS;
    tmpreg |= USB_OTG_HCCHAR_CHENA;
    USB_OTG_HC(ch_num)->HCCHAR = tmpreg;
}

static void dwc2_halt(struct usbh_bus *bus, uint8_t ch_num)
{
    volatile uint32_t ChannelEna = (USB_OTG_HC(ch_num)->HCCHAR & USB_OTG_HCCHAR_CHENA) >> 31;
    volatile uint32_t count = 0U;
    __IO uint32_t value;

    if (((USB_OTG_GLB->GAHBCFG & USB_OTG_GAHBCFG_DMAEN) == USB_OTG_GAHBCFG_DMAEN) &&
        (ChannelEna == 0U)) {
        return;
    }

    USB_OTG_HC(ch_num)->HCINTMSK = 0;

    value = USB_OTG_HC(ch_num)->HCCHAR;
    value |= USB_OTG_HCCHAR_CHDIS;
    value &= ~USB_OTG_HCCHAR_CHENA;
    /* *
     * High probability cause halt channel fail then we can not reuse 
     * this channel for other transfer, reproduce step just like below :
     * 1. Submit bulk in urb and device have no data, so host NAK always
     * 2. Kill bulk in urb in step 1
     * 3. Submit bulk out urb which will reuse the channel step 1 used before
     * 4. Bulk out urb will fail
     * */
#if 0
    value &= ~USB_OTG_HCCHAR_EPDIR;
#endif
    USB_OTG_HC(ch_num)->HCCHAR = value;
    do {
        if (++count > 200000U) {
            break;
        }
    } while (USB_OTG_HC(ch_num)->HCCHAR & USB_OTG_HCCHAR_CHENA);

    USB_OTG_HC(ch_num)->HCINT = USB_OTG_HC(ch_num)->HCINT;
}

static int usbh_reset_port(struct usbh_bus *bus, const uint8_t port)
{
    __IO uint32_t hprt0 = 0U;

    hprt0 = USB_OTG_HPRT;

    hprt0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |
               USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG);

    USB_OTG_HPRT = (USB_OTG_HPRT_PRST | hprt0);
    usb_osal_msleep(100U); /* See Note #1 */
    USB_OTG_HPRT = ((~USB_OTG_HPRT_PRST) & hprt0);
    usb_osal_msleep(10U);

    while (!(USB_OTG_HPRT & USB_OTG_HPRT_PENA)) {
        usb_osal_msleep(10U);
    }
    return 0;
}

/**
  * @brief  dwc2_get_glb_intstatus: return the global USB interrupt status
  * @retval status
  */
static inline uint32_t dwc2_get_glb_intstatus(struct usbh_bus *bus)
{
    uint32_t tmpreg;

    tmpreg = USB_OTG_GLB->GINTSTS;
    tmpreg &= USB_OTG_GLB->GINTMSK;

    return tmpreg;
}

static int dwc2_chan_alloc(struct usbh_bus *bus)
{
    size_t flags;
    int chidx;

    flags = usb_osal_enter_critical_section();
    for (chidx = 0; chidx < CONFIG_USBHOST_PIPE_NUM; chidx++) {
        if (!g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].inuse) {
            g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].inuse = true;
            /* cann't get usbh_bus in dwc2_chan_free, so deinit here */
            if (g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].do_split == 1) {
                uint32_t hcsplt;

                hcsplt = USB_OTG_HC((uint32_t)chidx)->HCSPLT;
                hcsplt &= ~USB_OTG_HCSPLT_SPLITEN;
                USB_OTG_HC((uint32_t)chidx)->HCSPLT = hcsplt;
                g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].complete_split = 0;
                g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].do_split = 0;
            }
            usb_osal_leave_critical_section(flags);
            return chidx;
        }
    }
    usb_osal_leave_critical_section(flags);
    return -1;
}

static void dwc2_chan_free(struct dwc2_chan *chan)
{
    size_t flags;

    flags = usb_osal_enter_critical_section();
    chan->inuse = false;
    usb_osal_leave_critical_section(flags);
}

static uint8_t dwc2_calculate_packet_num(uint32_t input_size, uint8_t ep_addr, uint16_t ep_mps, uint32_t *output_size)
{
    uint16_t num_packets;
    struct dwc2_chan *chan;

    chan = usb_container_of(output_size, struct dwc2_chan, xferlen);
    if (chan->do_split) {
        chan->xferlen = (input_size < ep_mps ? input_size: ep_mps);
        num_packets = 1;
        goto out;
    }

    num_packets = (uint16_t)((input_size + ep_mps - 1U) / ep_mps);

    if (num_packets > 256) {
        num_packets = 256;
    }

    if (input_size == 0) {
        num_packets = 1;
    }

    if (ep_addr & 0x80) {
        input_size = num_packets * ep_mps;
    } else {
    }

    chan->xferlen = input_size;

out:
    return num_packets;
}

static void dwc2_control_urb_init(struct usbh_bus *bus, uint8_t chidx, struct usbh_urb *urb, struct usb_setup_packet *setup, uint8_t *buffer, uint32_t buflen)
{
    struct dwc2_chan *chan;
    int done_len = 0;

    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx];

    if (chan->do_split) {
        done_len = urb->actual_length - USB_SIZEOF_SETUP_PACKET;
    }

    if (chan->ep0_state == DWC2_EP0_STATE_SETUP) /* fill setup */
    {
        chan->num_packets = dwc2_calculate_packet_num(8, 0x00, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
        dwc2_chan_init(bus, chidx, urb->hport->dev_addr, 0x00, USB_ENDPOINT_TYPE_CONTROL, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
        dwc2_chan_transfer(bus, chidx, 0x00, (uint32_t *)setup, chan->xferlen, chan->num_packets, HC_PID_SETUP);
    } else if (chan->ep0_state == DWC2_EP0_STATE_INDATA) /* fill in data */
    {
        chan->num_packets = dwc2_calculate_packet_num(setup->wLength - done_len, 0x80, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
        dwc2_chan_init(bus, chidx, urb->hport->dev_addr, 0x80, USB_ENDPOINT_TYPE_CONTROL, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
        dwc2_chan_transfer(bus, chidx, 0x80, (uint32_t *)(buffer + done_len), chan->xferlen, chan->num_packets, chan->do_split ? urb->data_toggle : HC_PID_DATA1);
    } else if (chan->ep0_state == DWC2_EP0_STATE_OUTDATA) /* fill out data */
    {
        chan->num_packets = dwc2_calculate_packet_num(setup->wLength - done_len, 0x00, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
        dwc2_chan_init(bus, chidx, urb->hport->dev_addr, 0x00, USB_ENDPOINT_TYPE_CONTROL, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
        dwc2_chan_transfer(bus, chidx, 0x00, (uint32_t *)(buffer + done_len), chan->xferlen, chan->num_packets, chan->do_split ? urb->data_toggle : HC_PID_DATA1);
    } else if (chan->ep0_state == DWC2_EP0_STATE_INSTATUS) /* fill in status */
    {
        chan->num_packets = dwc2_calculate_packet_num(0, 0x80, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
        dwc2_chan_init(bus, chidx, urb->hport->dev_addr, 0x80, USB_ENDPOINT_TYPE_CONTROL, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
        dwc2_chan_transfer(bus, chidx, 0x80, NULL, chan->xferlen, chan->num_packets, HC_PID_DATA1);
    } else if (chan->ep0_state == DWC2_EP0_STATE_OUTSTATUS) /* fill out status */
    {
        chan->num_packets = dwc2_calculate_packet_num(0, 0x00, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
        dwc2_chan_init(bus, chidx, urb->hport->dev_addr, 0x00, USB_ENDPOINT_TYPE_CONTROL, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
        dwc2_chan_transfer(bus, chidx, 0x00, NULL, chan->xferlen, chan->num_packets, HC_PID_DATA1);
    }
}

static void dwc2_bulk_intr_urb_init(struct usbh_bus *bus, uint8_t chidx, struct usbh_urb *urb, uint8_t *buffer, uint32_t buflen)
{
    struct dwc2_chan *chan;
    int done_len = 0;

    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx];
    if (chan->do_split) {
        done_len = urb->actual_length;
    }

    chan->num_packets = dwc2_calculate_packet_num(buflen - done_len, urb->ep->bEndpointAddress, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
    dwc2_chan_init(bus, chidx, urb->hport->dev_addr, urb->ep->bEndpointAddress, USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes), USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
    dwc2_chan_transfer(bus, chidx, urb->ep->bEndpointAddress, (uint32_t *)(buffer + done_len), chan->xferlen, chan->num_packets, urb->data_toggle);
}

static void dwc2_iso_urb_init(struct usbh_bus *bus, uint8_t chidx, struct usbh_urb *urb, struct usbh_iso_frame_packet *iso_packet)
{
    struct dwc2_chan *chan;

    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx];

    chan->num_packets = dwc2_calculate_packet_num(iso_packet->transfer_buffer_length, urb->ep->bEndpointAddress, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), &chan->xferlen);
    dwc2_chan_init(bus, chidx, urb->hport->dev_addr, urb->ep->bEndpointAddress, USB_ENDPOINT_TYPE_ISOCHRONOUS, USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize), urb->hport->speed);
    dwc2_chan_transfer(bus, chidx, urb->ep->bEndpointAddress, (uint32_t *)iso_packet->transfer_buffer, chan->xferlen, chan->num_packets, HC_PID_DATA0);
}

__WEAK void usb_hc_low_level_init(struct usbh_bus *bus)
{
}

__WEAK void usb_hc_low_level_deinit(struct usbh_bus *bus)
{
}

int usb_hc_init(struct usbh_bus *bus)
{
    int ret;

    usb_memset(&g_dwc2_hcd[bus->hcd.hcd_id], 0, sizeof(struct dwc2_hcd));

    for (uint8_t chidx = 0; chidx < CONFIG_USBHOST_PIPE_NUM; chidx++) {
        g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].waitsem = usb_osal_sem_create(0);
        g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].buf_align = rt_malloc_align(USB_BUF_ALIGN_SIZE, 64);
    }

    usb_hc_low_level_init(bus);

    USB_LOG_INFO("========== dwc2 hcd params ==========\r\n");
    USB_LOG_INFO("CID:%08x\r\n", USB_OTG_GLB->CID);
    USB_LOG_INFO("GSNPSID:%08x\r\n", USB_OTG_GLB->GSNPSID);
    USB_LOG_INFO("GHWCFG1:%08x\r\n", USB_OTG_GLB->GHWCFG1);
    USB_LOG_INFO("GHWCFG2:%08x\r\n", USB_OTG_GLB->GHWCFG2);
    USB_LOG_INFO("GHWCFG3:%08x\r\n", USB_OTG_GLB->GHWCFG3);
    USB_LOG_INFO("GHWCFG4:%08x\r\n", USB_OTG_GLB->GHWCFG4);

    USB_LOG_INFO("dwc2 has %d channels and dfifo depth(32-bit words) is %d\r\n", ((USB_OTG_GLB->GHWCFG2 & (0x0f << 14)) >> 14) + 1, (USB_OTG_GLB->GHWCFG3 >> 16));

    if (((USB_OTG_GLB->GHWCFG2 & (0x3U << 3)) >> 3) != 2) {
        USB_LOG_ERR("This dwc2 version does not support dma mode, so stop working\r\n");
        while (1) {
        }
    }

    if ((CONFIG_USB_DWC2_RX_FIFO_SIZE + CONFIG_USB_DWC2_NPTX_FIFO_SIZE + CONFIG_USB_DWC2_PTX_FIFO_SIZE) > (USB_OTG_GLB->GHWCFG3 >> 16)) {
        USB_LOG_ERR("Your fifo config is overflow, please check\r\n");
        while (1) {
        }
    }

    USB_OTG_GLB->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

    /* This is vendor register */
    USB_OTG_GLB->GCCFG = usbh_get_dwc2_gccfg_conf(bus->hcd.reg_base);

    ret = dwc2_core_init(bus);

    /* Force Host Mode*/
    dwc2_set_mode(bus, USB_OTG_MODE_HOST);
    usb_osal_msleep(50);

    /* Restart the Phy Clock */
    USB_OTG_PCGCCTL = 0U;

    dwc2_drivebus(bus, 1);
    usb_osal_msleep(200);

    /* Set default Max speed support */
    USB_OTG_HOST->HCFG &= ~(USB_OTG_HCFG_FSLSS);

    /* Clear all pending HC Interrupts */
    for (uint8_t i = 0U; i < CONFIG_USBHOST_PIPE_NUM; i++) {
        USB_OTG_HC(i)->HCINT = 0xFFFFFFFFU;
        USB_OTG_HC(i)->HCINTMSK = 0U;
    }

    /* Disable all interrupts. */
    USB_OTG_GLB->GINTMSK = 0U;

    /* Clear any pending interrupts */
    USB_OTG_GLB->GINTSTS = 0xFFFFFFFFU;

    /* set Rx FIFO size */
    USB_OTG_GLB->GRXFSIZ = CONFIG_USB_DWC2_RX_FIFO_SIZE;
    USB_OTG_GLB->DIEPTXF0_HNPTXFSIZ = (uint32_t)(((CONFIG_USB_DWC2_NPTX_FIFO_SIZE << 16) & USB_OTG_NPTXFD) | CONFIG_USB_DWC2_RX_FIFO_SIZE);
    USB_OTG_GLB->HPTXFSIZ = (uint32_t)(((CONFIG_USB_DWC2_PTX_FIFO_SIZE << 16) & USB_OTG_HPTXFSIZ_PTXFD) | (CONFIG_USB_DWC2_RX_FIFO_SIZE + CONFIG_USB_DWC2_NPTX_FIFO_SIZE));

    ret = dwc2_flush_txfifo(bus, 0x10U);
    ret = dwc2_flush_rxfifo(bus);

    USB_OTG_GLB->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_1;
    USB_OTG_GLB->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;

    /* Enable interrupts matching to the Host mode ONLY */
    USB_OTG_GLB->GINTMSK |= (USB_OTG_GINTMSK_PRTIM | USB_OTG_GINTMSK_HCIM |
                             USB_OTG_GINTSTS_DISCINT);

    USB_OTG_GLB->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    return ret;
}

int usb_hc_deinit(struct usbh_bus *bus)
{
    volatile uint32_t count = 0U;
    uint32_t value;

    USB_OTG_GLB->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

    dwc2_flush_txfifo(bus, 0x10U);
    dwc2_flush_rxfifo(bus);

    /* Flush out any leftover queued requests. */
    for (uint32_t i = 0U; i <= 15U; i++) {
        value = USB_OTG_HC(i)->HCCHAR;
        value |= USB_OTG_HCCHAR_CHDIS;
        value &= ~USB_OTG_HCCHAR_CHENA;
        value &= ~USB_OTG_HCCHAR_EPDIR;
        USB_OTG_HC(i)->HCCHAR = value;
    }

    /* Halt all channels to put them into a known state. */
    for (uint32_t i = 0U; i <= 15U; i++) {
        value = USB_OTG_HC(i)->HCCHAR;
        value |= USB_OTG_HCCHAR_CHDIS;
        value |= USB_OTG_HCCHAR_CHENA;
        value &= ~USB_OTG_HCCHAR_EPDIR;
        USB_OTG_HC(i)->HCCHAR = value;

        do {
            if (++count > 1000U) {
                return -USB_ERR_TIMEOUT;
            }
        } while ((USB_OTG_HC(i)->HCCHAR & USB_OTG_HCCHAR_CHENA) == USB_OTG_HCCHAR_CHENA);
    }

    /* Disable all interrupts. */
    USB_OTG_GLB->GINTMSK = 0U;

    /* Clear any pending Host interrupts */
    USB_OTG_HOST->HAINT = 0xFFFFFFFFU;
    USB_OTG_GLB->GINTSTS = 0xFFFFFFFFU;

    dwc2_drivebus(bus, 0);
    usb_osal_msleep(200);

    for (uint8_t chidx = 0; chidx < CONFIG_USBHOST_PIPE_NUM; chidx++) {
        usb_osal_sem_delete(g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].waitsem);
        rt_free_align(g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx].buf_align);
    }

    usb_hc_low_level_deinit(bus);
    return 0;
}

uint16_t usbh_get_frame_number(struct usbh_bus *bus)
{
    return (USB_OTG_HOST->HFNUM & USB_OTG_HFNUM_FRNUM);
}

int usbh_roothub_control(struct usbh_bus *bus, struct usb_setup_packet *setup, uint8_t *buf)
{
    __IO uint32_t hprt0;
    uint8_t nports;
    uint8_t port;
    uint32_t status;

    nports = CONFIG_USBHOST_MAX_RHPORTS;
    port = setup->wIndex;
    if (setup->bmRequestType & USB_REQUEST_RECIPIENT_DEVICE) {
        switch (setup->bRequest) {
            case HUB_REQUEST_CLEAR_FEATURE:
                switch (setup->wValue) {
                    case HUB_FEATURE_HUB_C_LOCALPOWER:
                        break;
                    case HUB_FEATURE_HUB_C_OVERCURRENT:
                        break;
                    default:
                        return -USB_ERR_NOTSUPP;
                }
                break;
            case HUB_REQUEST_SET_FEATURE:
                switch (setup->wValue) {
                    case HUB_FEATURE_HUB_C_LOCALPOWER:
                        break;
                    case HUB_FEATURE_HUB_C_OVERCURRENT:
                        break;
                    default:
                        return -USB_ERR_NOTSUPP;
                }
                break;
            case HUB_REQUEST_GET_DESCRIPTOR:
                break;
            case HUB_REQUEST_GET_STATUS:
                usb_memset(buf, 0, 4);
                break;
            default:
                break;
        }
    } else if (setup->bmRequestType & USB_REQUEST_RECIPIENT_OTHER) {
        switch (setup->bRequest) {
            case HUB_REQUEST_CLEAR_FEATURE:
                if (!port || port > nports) {
                    return -USB_ERR_INVAL;
                }

                switch (setup->wValue) {
                    case HUB_PORT_FEATURE_ENABLE:
                        USB_OTG_HPRT &= ~USB_OTG_HPRT_PENA;
                        break;
                    case HUB_PORT_FEATURE_SUSPEND:
                    case HUB_PORT_FEATURE_C_SUSPEND:
                        break;
                    case HUB_PORT_FEATURE_POWER:
                        break;
                    case HUB_PORT_FEATURE_C_CONNECTION:
                        g_dwc2_hcd[bus->hcd.hcd_id].port_csc = 0;
                        break;
                    case HUB_PORT_FEATURE_C_ENABLE:
                        g_dwc2_hcd[bus->hcd.hcd_id].port_pec = 0;
                        break;
                    case HUB_PORT_FEATURE_C_OVER_CURREN:
                        g_dwc2_hcd[bus->hcd.hcd_id].port_occ = 0;
                        break;
                    case HUB_PORT_FEATURE_C_RESET:
                        break;
                    default:
                        return -USB_ERR_NOTSUPP;
                }
                break;
            case HUB_REQUEST_SET_FEATURE:
                if (!port || port > nports) {
                    return -USB_ERR_INVAL;
                }

                switch (setup->wValue) {
                    case HUB_PORT_FEATURE_SUSPEND:
                        break;
                    case HUB_PORT_FEATURE_POWER:
                        USB_OTG_HPRT &= ~USB_OTG_HPRT_PPWR;
                        break;
                    case HUB_PORT_FEATURE_RESET:
                        usbh_reset_port(bus, port);
                        break;

                    default:
                        return -USB_ERR_NOTSUPP;
                }
                break;
            case HUB_REQUEST_GET_STATUS:
                if (!port || port > nports) {
                    return -USB_ERR_INVAL;
                }
                hprt0 = USB_OTG_HPRT;

                status = 0;
                if (g_dwc2_hcd[bus->hcd.hcd_id].port_csc) {
                    status |= (1 << HUB_PORT_FEATURE_C_CONNECTION);
                }
                if (g_dwc2_hcd[bus->hcd.hcd_id].port_pec) {
                    status |= (1 << HUB_PORT_FEATURE_C_ENABLE);
                }
                if (g_dwc2_hcd[bus->hcd.hcd_id].port_occ) {
                    status |= (1 << HUB_PORT_FEATURE_C_OVER_CURREN);
                }

                if (hprt0 & USB_OTG_HPRT_PCSTS) {
                    status |= (1 << HUB_PORT_FEATURE_CONNECTION);
                }
                if (hprt0 & USB_OTG_HPRT_PENA) {
                    status |= (1 << HUB_PORT_FEATURE_ENABLE);
                    if (usbh_get_port_speed(bus, port) == USB_SPEED_LOW) {
                        status |= (1 << HUB_PORT_FEATURE_LOWSPEED);
                    } else if (usbh_get_port_speed(bus, port) == USB_SPEED_HIGH) {
                        status |= (1 << HUB_PORT_FEATURE_HIGHSPEED);
                    }
                }
                if (hprt0 & USB_OTG_HPRT_POCA) {
                    status |= (1 << HUB_PORT_FEATURE_OVERCURRENT);
                }
                if (hprt0 & USB_OTG_HPRT_PRST) {
                    status |= (1 << HUB_PORT_FEATURE_RESET);
                }
                if (hprt0 & USB_OTG_HPRT_PPWR) {
                    status |= (1 << HUB_PORT_FEATURE_POWER);
                }

                usb_memcpy(buf, &status, 4);
                break;
            default:
                break;
        }
    }
    return 0;
}

int usbh_submit_urb(struct usbh_urb *urb)
{
    struct dwc2_chan *chan;
    struct usbh_bus *bus;
    size_t flags;
    int ret = 0;
    int chidx;

    if (!urb || !urb->hport || !urb->ep || !urb->hport->bus) {
        return -USB_ERR_INVAL;
    }

    /* dma addr must be aligned 4 bytes */
    if ((((uint32_t)urb->setup) & 0x03) || (((uint32_t)urb->transfer_buffer) & 0x03)) {
        return -USB_ERR_INVAL;
    }

    bus = urb->hport->bus;

    if (!(USB_OTG_HPRT & USB_OTG_HPRT_PCSTS) || !urb->hport->connected) {
        return -USB_ERR_NOTCONN;
    }

    if (urb->errorcode == -USB_ERR_BUSY) {
        return -USB_ERR_BUSY;
    }

    chidx = dwc2_chan_alloc(bus);
    if (chidx == -1) {
        return -USB_ERR_NOMEM;
    }

    flags = usb_osal_enter_critical_section();
    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[chidx];
    chan->chidx = chidx;
    chan->urb = urb;
    
    chan->buf = NULL;
    chan->align_used = 0;

    if (urb->hport->speed != USB_SPEED_HIGH &&
        usbh_get_port_speed(bus, 0) == USB_SPEED_HIGH) {
        chan->do_split = 1;
        chan->complete_split = 0;
        chan->hub_port = urb->hport->port;
        chan->hub_addr = urb->hport->parent->hub_addr;
#if 0
        USB_LOG_DBG("detect a FS/LS device behind a HS hub\r\n");
#endif
    }

    urb->hcpriv = chan;
    urb->errorcode = -USB_ERR_BUSY;
    urb->actual_length = 0;

    usb_osal_leave_critical_section(flags);

    switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
        case USB_ENDPOINT_TYPE_CONTROL:
            chan->ep0_state = DWC2_EP0_STATE_SETUP;
            dwc2_control_urb_init(bus, chidx, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
            break;
        case USB_ENDPOINT_TYPE_BULK:
        case USB_ENDPOINT_TYPE_INTERRUPT:
            dwc2_bulk_intr_urb_init(bus, chidx, urb, urb->transfer_buffer, urb->transfer_buffer_length);
            break;
        case USB_ENDPOINT_TYPE_ISOCHRONOUS:
            break;
        default:
            break;
    }

    if (urb->timeout > 0) {
        /* wait until timeout or sem give */
        ret = usb_osal_sem_take(chan->waitsem, urb->timeout);
        if (ret < 0) {
            goto errout_timeout;
        }
        urb->timeout = 0;
        ret = urb->errorcode;
        /* we can free chan when waitsem is done */
        dwc2_chan_free(chan);
    }
    return ret;
errout_timeout:
    urb->timeout = 0;
    usbh_kill_urb(urb);
    return ret;
}

int usbh_kill_urb(struct usbh_urb *urb)
{
    struct dwc2_chan *chan;
    struct usbh_bus *bus;
    size_t flags;

    if (!urb || !urb->hcpriv || !urb->hport->bus) {
        return -USB_ERR_INVAL;
    }

    bus = urb->hport->bus;

    flags = usb_osal_enter_critical_section();

    chan = (struct dwc2_chan *)urb->hcpriv;

    dwc2_halt(bus, chan->chidx);

    chan->urb = NULL;
    urb->hcpriv = NULL;
    urb->errorcode = -USB_ERR_SHUTDOWN;

    if (urb->timeout) {
        usb_osal_sem_give(chan->waitsem);
    } else {
        dwc2_chan_free(chan);
    }

    usb_osal_leave_critical_section(flags);

    return 0;
}

static inline void dwc2_urb_waitup(struct usbh_urb *urb)
{
    struct dwc2_chan *chan;

    chan = (struct dwc2_chan *)urb->hcpriv;
    chan->urb = NULL;
    urb->hcpriv = NULL;

    if (urb->timeout) {
        usb_osal_sem_give(chan->waitsem);
    } else {
        dwc2_chan_free(chan);
    }

    if (urb->complete) {
        if (urb->errorcode < 0) {
            urb->complete(urb->arg, urb->errorcode);
        } else {
            urb->complete(urb->arg, urb->actual_length);
        }
    }
}
static inline uint16_t dwc2_get_frame_num(struct usbh_bus *bus)
{
    uint32_t hfnum;

    hfnum = USB_OTG_HOST->HFNUM;

    return (hfnum & USB_OTG_HFNUM_FRNUM_Msk) >> USB_OTG_HFNUM_FRNUM_Pos;
}

static inline uint16_t dwc2_get_full_frame_num(struct usbh_bus *bus)
{
    uint16_t frame = dwc2_get_frame_num(bus);

    /* USB_OTG_HFNUM_FRNUM_Msk is 0xFFFF，but max frame num is 0x3FFF */
    return ((frame & 0x3FFF) >> 3);
}

static void dwc2_inchan_irq_handler(struct usbh_bus *bus, uint8_t ch_num)
{
    uint32_t chan_intstatus;
    struct dwc2_chan *chan;
    struct usbh_urb *urb;

    chan_intstatus = USB_OTG_HC(ch_num)->HCINT;

    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[ch_num];
    urb = chan->urb;
    //printf("s1:%08x\r\n", chan_intstatus);

    if (chan_intstatus & USB_OTG_HCINT_CHH) {
        if (chan_intstatus & USB_OTG_HCINT_XFRC) {
            urb->errorcode = 0;

            uint32_t count = chan->xferlen - (USB_OTG_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_XFRSIZ);                        /* how many size has received */
            uint32_t has_used_packets = chan->num_packets - ((USB_OTG_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_PKTCNT) >> 19); /* how many packets have used */

            if(chan->align_used)
            {
                rt_hw_cpu_dcache_invalidate((void *)chan->buf_align, count);
                usb_memcpy(chan->buf, chan->buf_align, count);
            }
            else {
                rt_hw_cpu_dcache_invalidate((void *)chan->buf, count);
            }
            
            urb->actual_length += count;

            uint8_t data_toggle = ((USB_OTG_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_DPID) >> USB_OTG_HCTSIZ_DPID_Pos);

            if (data_toggle == HC_PID_DATA0) {
                urb->data_toggle = HC_PID_DATA0;
            } else {
                urb->data_toggle = HC_PID_DATA1;
            }

            if (chan->do_split) {
                if (chan->complete_split == 1) {
                    chan->complete_split = 0;
                }
            }

            if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_CONTROL) {
                if (chan->ep0_state == DWC2_EP0_STATE_INDATA) {
                    if (chan->do_split) {
                        if ((count >= chan->xferlen) && (((urb->actual_length - 8) < urb->transfer_buffer_length))) {
                            dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                            goto out;
                        }
                    }
                    chan->ep0_state = DWC2_EP0_STATE_OUTSTATUS;
                    dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                } else if (chan->ep0_state == DWC2_EP0_STATE_INSTATUS) {
                    chan->ep0_state = DWC2_EP0_STATE_SETUP;
                    dwc2_urb_waitup(urb);
                }
            } else if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            } else {
                if (chan->do_split) {
                        if ((count >= chan->xferlen) && (urb->actual_length < urb->transfer_buffer_length)) {
                            dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                            goto out;
                        }
                }
                dwc2_urb_waitup(urb);
            }
        } else if (chan_intstatus & USB_OTG_HCINT_ACK) {
            if (chan->do_split) {
                /* Handle ACK on SSPLIT. ACK should not occur in CSPLIT. */
                switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
                case USB_ENDPOINT_TYPE_CONTROL:
                        chan->complete_split = 1;
                        dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                   break;
                case USB_ENDPOINT_TYPE_BULK:
                case USB_ENDPOINT_TYPE_INTERRUPT:
                        chan->complete_split = 1;
                        chan->ssplit_frame = dwc2_get_full_frame_num(bus);
                        dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                   break;
                default:
                   break;
                }
            }
        } else if (chan_intstatus & USB_OTG_HCINT_AHBERR) {
            urb->errorcode = -USB_ERR_IO;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_STALL) {
            urb->errorcode = -USB_ERR_STALL;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_NAK) {
            if (chan->do_split) {
                /*
                 * Handle NAK for IN/OUT SSPLIT/CSPLIT transfers, bulk, control, and
                 * interrupt. Re-start the SSPLIT transfer.
                 */
                switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
                case USB_ENDPOINT_TYPE_CONTROL:
                        chan->complete_split = 0;
                        dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                case USB_ENDPOINT_TYPE_BULK:
                case USB_ENDPOINT_TYPE_INTERRUPT:
                        chan->complete_split = 0;
                        dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                default:
                    break;
                }
            } else {
                urb->errorcode = -USB_ERR_NAK;
                dwc2_urb_waitup(urb);
            }
        } else if (chan_intstatus & USB_OTG_HCINT_NYET) {
            if (chan->do_split) {
                /*
                 * NYET on CSPLIT
                 * re-do the CSPLIT immediately on non-periodic
                 */
                switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
                case USB_ENDPOINT_TYPE_CONTROL:
                    chan->complete_split = 1;
                    dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                case USB_ENDPOINT_TYPE_BULK:
                case USB_ENDPOINT_TYPE_INTERRUPT:
                    if ((dwc2_get_full_frame_num(bus) - chan->ssplit_frame) > 4) {
                        chan->complete_split = 0;
                    } else {
                        chan->complete_split = 1;
                    }
                    dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                default:
                    break;
                }
            } else {
                urb->errorcode = -USB_ERR_NAK;
                dwc2_urb_waitup(urb);
            }
        } else if (chan_intstatus & USB_OTG_HCINT_TXERR) {
            urb->errorcode = -USB_ERR_IO;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_BBERR) {
            urb->errorcode = -USB_ERR_BABBLE;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_DTERR) {
            urb->errorcode = -USB_ERR_DT;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_FRMOR) {
            urb->errorcode = -USB_ERR_IO;
            dwc2_urb_waitup(urb);
        }
out:
        USB_OTG_HC(ch_num)->HCINT = chan_intstatus;
    }
}

static void dwc2_outchan_irq_handler(struct usbh_bus *bus, uint8_t ch_num)
{
    uint32_t chan_intstatus;
    struct dwc2_chan *chan;
    struct usbh_urb *urb;

    chan_intstatus = USB_OTG_HC(ch_num)->HCINT;

    chan = &g_dwc2_hcd[bus->hcd.hcd_id].chan_pool[ch_num];
    urb = chan->urb;
    //printf("s2:%08x\r\n", chan_intstatus);

    if (chan_intstatus & USB_OTG_HCINT_CHH) {
        if (chan_intstatus & USB_OTG_HCINT_XFRC) {
            urb->errorcode = 0;

            uint32_t count = USB_OTG_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_XFRSIZ;                                          /* last packet size */
            uint32_t has_used_packets = chan->num_packets - ((USB_OTG_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_PKTCNT) >> 19); /* how many packets have used */
            uint32_t olen = (has_used_packets - 1) * USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize) + count;

            urb->actual_length += olen; //the same with urb->actual_length += chan->xferlen;

            uint8_t data_toggle = ((USB_OTG_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_DPID) >> USB_OTG_HCTSIZ_DPID_Pos);

            if (data_toggle == HC_PID_DATA0) {
                urb->data_toggle = HC_PID_DATA0;
            } else {
                urb->data_toggle = HC_PID_DATA1;
            }

            if (chan->do_split) {
                if (chan->complete_split == 1) {
                    chan->complete_split = 0;
                }
            }

            if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_CONTROL) {
                if (chan->ep0_state == DWC2_EP0_STATE_SETUP) {
                    if (urb->setup->wLength) {
                        if (urb->setup->bmRequestType & 0x80) {
                            chan->ep0_state = DWC2_EP0_STATE_INDATA;
                        } else {
                            chan->ep0_state = DWC2_EP0_STATE_OUTDATA;
                        }
                    } else {
                        chan->ep0_state = DWC2_EP0_STATE_INSTATUS;
                    }
                    dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                } else if (chan->ep0_state == DWC2_EP0_STATE_OUTDATA) {
                    if (chan->do_split) {
                        /* maybe need zero packet */
                        if ((olen >= chan->xferlen) && (((urb->actual_length - 8) < urb->transfer_buffer_length))) {
                            dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                            goto out;
                        }
                    }
                    chan->ep0_state = DWC2_EP0_STATE_INSTATUS;
                    dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                } else if (chan->ep0_state == DWC2_EP0_STATE_OUTSTATUS) {
                    chan->ep0_state = DWC2_EP0_STATE_SETUP;
                    dwc2_urb_waitup(urb);
                }
            } else if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            } else {
                if (chan->do_split) {
                        /* maybe need zero packet */
                        if ((olen >= chan->xferlen) && (urb->actual_length < urb->transfer_buffer_length)) {
                            dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                            goto out;
                        }
                }
                dwc2_urb_waitup(urb);
            }
        } else if (chan_intstatus & USB_OTG_HCINT_ACK) {
            if (chan->do_split) {
                /* Handle ACK on SSPLIT. ACK should not occur in CSPLIT. */
                switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
                case USB_ENDPOINT_TYPE_CONTROL:
                        chan->complete_split = 1;
                        dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                case USB_ENDPOINT_TYPE_BULK:
                case USB_ENDPOINT_TYPE_INTERRUPT:
                        chan->complete_split = 1;
                        chan->ssplit_frame = dwc2_get_full_frame_num(bus);
                        dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                   break;
                default:
                    break;
                }
            }
        } else if (chan_intstatus & USB_OTG_HCINT_AHBERR) {
            urb->errorcode = -USB_ERR_IO;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_STALL) {
            urb->errorcode = -USB_ERR_STALL;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_NAK) {
            if (chan->do_split) {
                /*
                 * Handle NAK for IN/OUT SSPLIT/CSPLIT transfers, bulk, control, and
                 * interrupt. Re-start the SSPLIT transfer.
                 */
                switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
                case USB_ENDPOINT_TYPE_CONTROL:
                        chan->complete_split = 0;
                        dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                case USB_ENDPOINT_TYPE_BULK:
                case USB_ENDPOINT_TYPE_INTERRUPT:
                        chan->complete_split = 0;
                        dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                default:
                    break;
                }
            } else {
                urb->errorcode = -USB_ERR_NAK;
                dwc2_urb_waitup(urb);
            }
        } else if (chan_intstatus & USB_OTG_HCINT_NYET) {
            if (chan->do_split) {
                /*
                 * NYET on CSPLIT
                 * re-do the CSPLIT immediately on non-periodic
                 */
                switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
                case USB_ENDPOINT_TYPE_CONTROL:
                        chan->complete_split = 1;
                        dwc2_control_urb_init(bus, ch_num, urb, urb->setup, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                case USB_ENDPOINT_TYPE_BULK:
                case USB_ENDPOINT_TYPE_INTERRUPT:
                    if ((dwc2_get_full_frame_num(bus) - chan->ssplit_frame) > 4) {
                        chan->complete_split = 0;
                    } else {
                        chan->complete_split = 1;
                    }
                    dwc2_bulk_intr_urb_init(bus, ch_num, urb, urb->transfer_buffer, urb->transfer_buffer_length);
                    break;
                default:
                    break;
                }
            } else {
                urb->errorcode = -USB_ERR_NAK;
                dwc2_urb_waitup(urb);
            }
        } else if (chan_intstatus & USB_OTG_HCINT_TXERR) {
            urb->errorcode = -USB_ERR_IO;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_BBERR) {
            urb->errorcode = -USB_ERR_BABBLE;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_DTERR) {
            urb->errorcode = -USB_ERR_DT;
            dwc2_urb_waitup(urb);
        } else if (chan_intstatus & USB_OTG_HCINT_FRMOR) {
            urb->errorcode = -USB_ERR_IO;
            dwc2_urb_waitup(urb);
        }
out:
        USB_OTG_HC(ch_num)->HCINT = chan_intstatus;
    }
}

static void dwc2_port_irq_handler(struct usbh_bus *bus)
{
    __IO uint32_t hprt0, hprt0_dup, regval;

    /* Handle Host Port Interrupts */
    hprt0 = USB_OTG_HPRT;
    hprt0_dup = USB_OTG_HPRT;

    hprt0_dup &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |
                   USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG);

    /* Check whether Port Connect detected */
    if ((hprt0 & USB_OTG_HPRT_PCDET) == USB_OTG_HPRT_PCDET) {
        if ((hprt0 & USB_OTG_HPRT_PCSTS) == USB_OTG_HPRT_PCSTS) {
            bus->hcd.roothub.int_buffer[0] = (1 << 1);
            usbh_hub_thread_wakeup(&bus->hcd.roothub);
        }
        hprt0_dup |= USB_OTG_HPRT_PCDET;
        g_dwc2_hcd[bus->hcd.hcd_id].port_csc = 1;
    }

    /* Check whether Port Enable Changed */
    if ((hprt0 & USB_OTG_HPRT_PENCHNG) == USB_OTG_HPRT_PENCHNG) {
        hprt0_dup |= USB_OTG_HPRT_PENCHNG;
        g_dwc2_hcd[bus->hcd.hcd_id].port_pec = 1;

        if ((hprt0 & USB_OTG_HPRT_PENA) == USB_OTG_HPRT_PENA) {
#if defined(CONFIG_USB_HS)
#else
            if ((hprt0 & USB_OTG_HPRT_PSPD) == (HPRT0_PRTSPD_LOW_SPEED << 17)) {
                USB_OTG_HOST->HFIR = 6000U;
                if ((USB_OTG_HOST->HCFG & USB_OTG_HCFG_FSLSPCS) != USB_OTG_HCFG_FSLSPCS_1) {
                    regval = USB_OTG_HOST->HCFG;
                    regval &= ~USB_OTG_HCFG_FSLSPCS;
                    regval |= USB_OTG_HCFG_FSLSPCS_1;
                    USB_OTG_HOST->HCFG = regval;
                }
            } else {
                USB_OTG_HOST->HFIR = 48000U;
                if ((USB_OTG_HOST->HCFG & USB_OTG_HCFG_FSLSPCS) != USB_OTG_HCFG_FSLSPCS_0) {
                    regval = USB_OTG_HOST->HCFG;
                    regval &= ~USB_OTG_HCFG_FSLSPCS;
                    regval |= USB_OTG_HCFG_FSLSPCS_0;
                    USB_OTG_HOST->HCFG = regval;
                }
            }
#endif
        } else {
        }
    }

    /* Check for an overcurrent */
    if ((hprt0 & USB_OTG_HPRT_POCCHNG) == USB_OTG_HPRT_POCCHNG) {
        hprt0_dup |= USB_OTG_HPRT_POCCHNG;
        g_dwc2_hcd[bus->hcd.hcd_id].port_occ = 1;
    }
    /* Clear Port Interrupts */
    USB_OTG_HPRT = hprt0_dup;
}

void USBH_IRQHandler(uint8_t busid)
{
    uint32_t gint_status, chan_int;
    struct usbh_bus *bus;

    bus = &g_usbhost_bus[busid];
    gint_status = dwc2_get_glb_intstatus(bus);
    if ((USB_OTG_GLB->GINTSTS & 0x1U) == USB_OTG_MODE_HOST) {
        /* Avoid spurious interrupt */
        if (gint_status == 0) {
            return;
        }

        if (gint_status & USB_OTG_GINTSTS_HPRTINT) {
            dwc2_port_irq_handler(bus);
        }
        if (gint_status & USB_OTG_GINTSTS_DISCINT) {
            g_dwc2_hcd[bus->hcd.hcd_id].port_csc = 1;
            bus->hcd.roothub.int_buffer[0] = (1 << 1);
            usbh_hub_thread_wakeup(&bus->hcd.roothub);

            USB_OTG_GLB->GINTSTS = USB_OTG_GINTSTS_DISCINT;
        }
        if (gint_status & USB_OTG_GINTSTS_HCINT) {
            chan_int = (USB_OTG_HOST->HAINT & USB_OTG_HOST->HAINTMSK) & 0xFFFFU;
            for (uint8_t i = 0U; i < CONFIG_USBHOST_PIPE_NUM; i++) {
                if ((chan_int & (1UL << (i & 0xFU))) != 0U) {
                    if ((USB_OTG_HC(i)->HCCHAR & USB_OTG_HCCHAR_EPDIR) == USB_OTG_HCCHAR_EPDIR) {
                        dwc2_inchan_irq_handler(bus, i);
                    } else {
                        dwc2_outchan_irq_handler(bus, i);
                    }
                }
            }
            USB_OTG_GLB->GINTSTS = USB_OTG_GINTSTS_HCINT;
        }
    }
}
