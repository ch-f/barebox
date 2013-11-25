#include <driver.h>
#include <xfuncs.h>
#include <common.h>
#include <watchdog.h>
#include <errno.h>
#include <init.h>
#include <stdio.h>
#include <io.h>
#include <malloc.h>
#include <clock.h>

#include <mach/am33xx-usb.h>
#include <asm/omap_musb.h>

#include <usb/usb.h>
#include "linux-compat.h"
#include "usb-compat.h"
#include "musb_core.h"
#include "musb_host.h"

static struct musb *host;
static struct usb_hcd hcd;
static int host_speed;

struct am33xxusb_device {
	struct device_d *dev;

	void __iomem *usbss;
	void __iomem *usb;
	void __iomem *usbphy;
	void __iomem *usbcore;

	void __iomem *ctrl_base;
};

static void musb_host_complete_urb(struct urb *urb)
{
	urb->dev->status &= ~USB_ST_NOT_PROC;
	urb->dev->act_len = urb->actual_length;
}

static struct usb_host_endpoint hep;
static struct urb urb;

static struct urb *construct_urb(struct usb_device *dev, int endpoint_type,
				unsigned long pipe, void *buffer, int len,
				struct devrequest *setup, int interval)
{
	int epnum = usb_pipeendpoint(pipe);
	int is_in = usb_pipein(pipe);

	memset(&urb, 0, sizeof(struct urb));
	memset(&hep, 0, sizeof(struct usb_host_endpoint));
	INIT_LIST_HEAD(&hep.urb_list);
	INIT_LIST_HEAD(&urb.urb_list);
	urb.ep = &hep;
	urb.complete = musb_host_complete_urb;
	urb.status = -EINPROGRESS;
	urb.dev = dev;
	urb.pipe = pipe;
	urb.transfer_buffer = buffer;
	urb.transfer_dma = (unsigned long)buffer;
	urb.transfer_buffer_length = len;
	urb.setup_packet = (unsigned char *)setup;

	urb.ep->desc.wMaxPacketSize =
		__cpu_to_le16(is_in ? dev->epmaxpacketin[epnum] :
				dev->epmaxpacketout[epnum]);
	urb.ep->desc.bmAttributes = endpoint_type;
	urb.ep->desc.bEndpointAddress =
		(is_in ? USB_DIR_IN : USB_DIR_OUT) | epnum;
	urb.ep->desc.bInterval = interval;

	return &urb;
}

#define MUSB_HOST_TIMEOUT	0x3ffffff

static int submit_urb(struct usb_hcd *hcd, struct urb *urb)
{
	struct musb *host = hcd->hcd_priv;
	int ret;
	int timeout;

	ret = musb_urb_enqueue(hcd, urb, 0);
	if (ret < 0) {
		printf("Failed to enqueue URB to controller\n");
		return ret;
	}

	timeout = MUSB_HOST_TIMEOUT;
	do {
		if (ctrlc())
			return -EIO;
		host->isr(0, host);
	} while ((urb->dev->status & USB_ST_NOT_PROC) && --timeout);

	return urb->status;
}

int submit_control_msg(struct usb_device *dev, unsigned long pipe,
			void *buffer, int len, struct devrequest *setup,
			int timeout)
{
	struct urb *urb = construct_urb(dev, USB_ENDPOINT_XFER_CONTROL, pipe,
					buffer, len, setup, 0);

	/* Fix speed for non hub-attached devices */
	if (!dev->parent)
		dev->speed = host_speed;

	return submit_urb(&hcd, urb);
}


int submit_bulk_msg(struct usb_device *dev, unsigned long pipe,
					void *buffer, int len, int timeout)
{
	struct urb *urb = construct_urb(dev, USB_ENDPOINT_XFER_BULK, pipe,
					buffer, len, NULL, 0);
	return submit_urb(&hcd, urb);
}

int submit_int_msg(struct usb_device *dev, unsigned long pipe,
				void *buffer, int len, int interval)
{
	struct urb *urb = construct_urb(dev, USB_ENDPOINT_XFER_INT, pipe,
					buffer, len, NULL, interval);
	return submit_urb(&hcd, urb);
}

int usb_lowlevel_init(struct usb_host *bb_host)
{
	u8 power;
	void *mbase;
	int timeout = MUSB_HOST_TIMEOUT;
	int rc;

	if (!host) {
		printf("MUSB host is not registered\n");
		return -ENODEV;
	}

	musb_start(host);
	mbase = host->mregs;
	do {
		if (musb_readb(mbase, MUSB_DEVCTL) & MUSB_DEVCTL_HM)
			break;
	} while (--timeout);
	if (!timeout)
		return -ENODEV;

	mdelay(200);
	power = musb_readb(mbase, MUSB_POWER);
	musb_writeb(mbase, MUSB_POWER, MUSB_POWER_RESET | power);
	mdelay(200);
	power = musb_readb(mbase, MUSB_POWER);
	musb_writeb(mbase, MUSB_POWER, ~MUSB_POWER_RESET & power);
	mdelay(10);
	host->isr(0, host);
	host_speed = (musb_readb(mbase, MUSB_POWER) & MUSB_POWER_HSMODE) ?
			USB_SPEED_HIGH :
			(musb_readb(mbase, MUSB_DEVCTL) & MUSB_DEVCTL_FSDEV) ?
			USB_SPEED_FULL : USB_SPEED_LOW;
	host->is_active = 1;
	hcd.hcd_priv = host;

	return 0;
}

int usb_lowlevel_stop(int index)
{
	if (!host) {
		printf("MUSB host is not registered\n");
		return -ENODEV;
	}

	musb_stop(host);
	return 0;
}

int musb_register(struct musb_hdrc_platform_data *plat, void *bdata,
			void *ctl_regs)
{
	struct musb **musbp;
	struct usb_host *bb_host;

	switch (plat->mode) {
	case MUSB_HOST:
		musbp = &host;
		break;
	default:
		return -EINVAL;
	}

	*musbp = musb_init_controller(plat, (struct device_d *)bdata, ctl_regs);
	if (!musbp) {
		printf("Failed to init the controller\n");
		return -EIO;
	}

	bb_host = &host->barebox_host;
	bb_host->init = usb_lowlevel_init;
	bb_host->submit_int_msg = submit_int_msg;
	bb_host->submit_control_msg = submit_control_msg;
	bb_host->submit_bulk_msg = submit_bulk_msg;

	/* Is there more left that needs to get bonded to Barebox ? */
	usb_register_host(bb_host);

	return 0;
}

/* USB 2.0 PHY Control */
#define CM_PHY_PWRDN			(1 << 0)
#define CM_PHY_OTG_PWRDN		(1 << 1)
#define OTGVDET_EN			(1 << 19)
#define OTGSESSENDEN			(1 << 20)

static u32 a3u_read(void __iomem *addr)
{
	return __raw_readl(addr);
}

static void a3u_write(u32 val, void __iomem *addr)
{
	__raw_writel(val, addr);
}

void a3u_clrsetbits(void __iomem *addr, u32 clr, u32 set)
{
	u32 x;
	x = a3u_read(addr);
	x &= ~clr;
	x |= set;
	a3u_write(x, addr);
}

void __iomem *reg_addr_phy_pwr;

static void am33xx_usb_set_phy_power(u8 on)
{
	if (on) {
		a3u_clrsetbits(reg_addr_phy_pwr, CM_PHY_PWRDN |
				CM_PHY_OTG_PWRDN, OTGVDET_EN | OTGSESSENDEN);
	} else {
		a3u_clrsetbits(reg_addr_phy_pwr, 0, CM_PHY_PWRDN |
				CM_PHY_OTG_PWRDN);
	}
}

static struct musb_hdrc_config musb_config = {
	.multipoint     = 1,
	.dyn_fifo       = 1,
	.num_eps        = 16,
	.ram_bits       = 12,
};

struct omap_musb_board_data otg0_board_data = {
	.set_phy_power = am33xx_usb_set_phy_power,
};

static struct musb_hdrc_platform_data otg0_plat = {
	.mode           = MUSB_HOST,
	.config         = &musb_config,
	.power          = 50,
	.platform_ops	= &musb_dsps_ops,
	.board_data	= &otg0_board_data,
};

static int am33xxusb_probe(struct device_d *dev)
{
	struct am33xx_usb_pdata const *pdata = dev->platform_data;
	struct am33xxusb_device *a3dev;

	a3dev = xzalloc(sizeof *a3dev);

	a3dev->usbss   = dev_request_mem_region_by_name(dev, "am33xx_usbss");
	a3dev->ctrl_base = dev_request_mem_region_by_name(dev,
					"am33xx_ctrl_device_base");
	if (pdata->bus_number == 1)
		a3dev->usb = dev_request_mem_region_by_name(dev, "am33xx_usb1");
	else
		a3dev->usb = dev_request_mem_region_by_name(dev, "am33xx_usb0");

	reg_addr_phy_pwr = a3dev->ctrl_base + 0x28;

	a3u_clrsetbits(a3dev->usbss + 0x10, 0, 1);

	mdelay(20);

	return musb_register(&otg0_plat, &otg0_board_data,
			a3dev->usb);
}

static struct driver_d musb_driver = {
		.name	= "am33xx_usb",
		.probe	= am33xxusb_probe,
};

static int am33xx_usb_init(void)
{
	return platform_driver_register(&musb_driver);
}

device_initcall(am33xx_usb_init);
