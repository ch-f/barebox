#ifndef H_BAREBOX_ARCH_ARM_MACH_OMAP_MACH_AUSB_H
#define H_BAREBOX_ARCH_ARM_MACH_OMAP_MACH_AUSB_H

struct am33xx_usb_pdata {
	unsigned int	bus_number;
};

struct device_d;
struct device_d *am33xx_add_usb(struct am33xx_usb_pdata *a_pdata);

#endif	/* H_BAREBOX_ARCH_ARM_MACH_OMAP_MACH_AUSB_H */
