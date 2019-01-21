#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include "usb_private.h"

uint8_t st_usbfs_ep_in_ready(usbd_device *dev, uint8_t addr)
{
	(void)dev;
	addr &= 0x7f;
	if ((*USB_EP_REG(addr) & USB_EP_TX_STAT) == USB_EP_TX_STAT_VALID) {
		return 0;
	}
	return 1;	
}

