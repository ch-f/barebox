#include <driver.h>
#include <common.h>
#include <linux/ioport.h>
#include <mach/am33xx-silicon.h>
#include <mach/am33xx-usb.h>

#ifdef CONFIG_USB_MUSB_HOST
static struct resource am33xx_usb_resources[] = {
	{
		.name	= "am33xx_usbss",
		.start	= AM33XX_USBSS_BASE,
		.end	= AM33XX_USBSS_BASE + 1024 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_usb0",
		.start	= AM33XX_USB0_BASE,
		.end	= AM33XX_USB0_BASE + 767 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_usb1",
		.start	= AM33XX_USB1_BASE,
		.end	= AM33XX_USB1_BASE + 767 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_usb0phy",
		.start	= AM33XX_USB0_PHY_BASE,
		.end	= AM33XX_USB0_PHY_BASE + 255 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_usb1phy",
		.start	= AM33XX_USB1_PHY_BASE,
		.end	= AM33XX_USB1_PHY_BASE + 255 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_usb0core",
		.start	= AM33XX_USB0_CORE_BASE,
		.end	= AM33XX_USB0_CORE_BASE + 1023 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_usb1core",
		.start	= AM33XX_USB1_CORE_BASE,
		.end	= AM33XX_USB1_CORE_BASE + 1023 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.name	= "am33xx_ctrl_device_base",
		.start	= AM33XX_IDCODE_REG,
		.end	= AM33XX_IDCODE_REG + 128 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	},
};

struct device_d *am33xx_add_usb(struct am33xx_usb_pdata *a_pdata)
{
	return add_generic_device_res("am33xx_usb", -1,
				am33xx_usb_resources,
				ARRAY_SIZE(am33xx_usb_resources),
				a_pdata);
}
#else
struct device_d *am33xx_add_usb(struct am33xx_usb_pdata *a_pdata)
{
	return NULL;
}
#endif
EXPORT_SYMBOL(am33xx_add_usb0);
