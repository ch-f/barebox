#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#

obj-$(CONFIG_USB_MUSB_HOST) += musb_host.o musb_core.o
obj-$(CONFIG_USB_MUSB_DSPS) += musb_dsps.o

CFLAGS_NO_WARN := $(call cc-option,-Wno-unused-variable) \
			$(call cc-option,-Wno-unused-but-set-variable) \
			$(call cc-option,-Wno-unused-label)
CFLAGS += $(CFLAGS_NO_WARN)
