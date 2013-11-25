#ifndef __LINUX_COMPAT_H__
#define __LINUX_COMPAT_H__

#include <malloc.h>
#include <linux/list.h>
#include <notifier.h>

#define __init
#define __devinit
#define __devinitdata
#define __devinitconst
#define __iomem
#define __deprecated

struct unused {};
typedef struct unused unused_t;

typedef int irqreturn_t;

struct work_struct {};

struct timer_list {};

typedef unsigned long dmaaddr_t;

#define setup_timer(timer, func, data) do {} while (0)
#define del_timer_sync(timer) do {} while (0)
#define schedule_work(work) do {} while (0)
#define INIT_WORK(work, fun) do {} while (0)

#define cpu_relax() do {} while (0)

#define pm_runtime_get_sync(dev) do {} while (0)
#define pm_runtime_put(dev) do {} while (0)
#define pm_runtime_put_sync(dev) do {} while (0)
#define pm_runtime_use_autosuspend(dev) do {} while (0)
#define pm_runtime_set_autosuspend_delay(dev, delay) do {} while (0)
#define pm_runtime_enable(dev) do {} while (0)

#define MODULE_ALIAS(alias)
#define module_param(name, type, perm)
#define MODULE_PARM_DESC(name, desc)

#define IRQ_NONE 0
#define IRQ_HANDLED 0

#define dev_set_drvdata(dev, data) do {} while (0)

#define disable_irq_wake(irq) do {} while (0)
#define enable_irq_wake(irq) -EINVAL
#define free_irq(irq, data) do {} while (0)
#define request_irq(nr, f, flags, nm, data) 0

#define device_init_wakeup(dev, a) do {} while (0)

#ifndef wmb
#define wmb()			asm volatile (""   : : : "memory")
#endif

#define msleep(a)	udelay(a * 1000)

/*
 * Map U-Boot config options to Linux ones
 */
#ifdef CONFIG_OMAP34XX
#define CONFIG_SOC_OMAP3430
#endif

#endif /* __LINUX_COMPAT_H__ */
