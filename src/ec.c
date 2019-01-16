/*
 * This file is part of the open-ec (Embedded controller) project.
 *
 * Copyright (C) 2019 Zhiyuan Wan <h@iloli.bid>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <stdlib.h>


//TODO: Move platform specific code out from main.c
/* Here it starts */
static void rcc_clock_setup_in_hse_24mhz_out_72mhz(void)
{
    /* Set system clock to 72 MHz, base frequency = 24MHz */
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 24MHz. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 72MHz Max. 72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);  /* Set.  9MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);     /* Set. 36MHz Max. 36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 72MHz Max. 72MHz */

	/*
	 * Sysclk runs with 72MHz -> 2 waitstates. (STM32 only)
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_ACR_LATENCY_2WS);

	/*
	 * Set the PLL multiplication factor to 6.
	 * 12MHz (external) * 6 (multiplier) = 72MHz
	 */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL6);

	/* Select HSE as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	/* Some board will have issues in high external clock input > 16MHz
	 * Set prescaler to 2 to avoid this issue and make system more stable
	 */
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2); 

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = 72000000;
	rcc_apb1_frequency = 36000000;
	rcc_apb2_frequency = 72000000;
	
}


static void clock_setup(void)
{
	rcc_clock_setup_in_hse_24mhz_out_72mhz();
	//rcc_clock_setup_in_hsi_out_48mhz();

	/* Enable clocks. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	//rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
	
	SCB_VTOR = 0x08000000; //TODO: Bootloader support
}


#define dtr_set() gpio_set(GPIOB, GPIO15); gpio_set(GPIOB, GPIO10)
#define dtr_clr() gpio_clear(GPIOB, GPIO15); gpio_clear(GPIOB, GPIO10)

#define rts_set() gpio_set(GPIOA, GPIO2); gpio_set(GPIOB, GPIO11)
#define rts_clr() gpio_clear(GPIOA, GPIO2); gpio_clear(GPIOB, GPIO11)

static void gpio_setup(void)
{
	/* LED 引脚 */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2 | GPIO10 | GPIO11);
	/* USB 上拉 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
		      
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO15); //1.8V IO BOOT脚（DTR）
			
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO2);  //1.8V IO RST脚（RTS）

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO5 | GPIO7); //TCK TDI
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO14); //TMS
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO6); //TDO
	//暂且用Bitbang模拟MPSSE
				
	dtr_set();
	rts_set();
}

/* Here it ends */

/* USB process */

#include <libopencm3/usb/usbd.h>

usbd_device *usbd_dev_handler;


uint8_t st_usbfs_ep_out_free(usbd_device *dev, uint8_t addr);

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 32,
	.idVendor = 0x0403,
	.idProduct = 0x6010, /* Specific VID/PID to make debugger happy */
	.bcdDevice = 0x0500,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor jtag_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x02,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};


static const struct usb_interface_descriptor jtag_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = USB_CLASS_VENDOR,
	.bInterfaceProtocol = USB_CLASS_VENDOR,
	.iInterface = 2,

	.endpoint = jtag_endp,
}};

static const struct usb_endpoint_descriptor serial_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x04,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};


static const struct usb_interface_descriptor serial_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = USB_CLASS_VENDOR,
	.bInterfaceProtocol = USB_CLASS_VENDOR,
	.iInterface = 2,

	.endpoint = serial_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = jtag_iface,
}, {
	.num_altsetting = 1,
	.altsetting = serial_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Zhiyuan Wan",
	"USB Debugger"
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[64];

/* USB interrupt handler */
void usb_wakeup_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_hp_can_tx_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_lp_can_rx0_isr(void)
	__attribute__ ((alias ("usb_int_relay")));


static void usb_int_relay(void) 
{
	usbd_poll(usbd_dev_handler);
}

#define FTDI_SET_REQUEST_TYPE 0x40
#define FTDI_GET_REQUEST_TYPE 0xC0

#define FTDI_SIO_RESET			0 /* Reset the port */
#define FTDI_SIO_MODEM_CTRL		1 /* Set the modem control register */
#define FTDI_SIO_SET_FLOW_CTRL		2 /* Set flow control register */
#define FTDI_SIO_SET_BAUD_RATE		3 /* Set baud rate */
#define FTDI_SIO_SET_DATA		4 /* Set the data characteristics of
					     the port */
#define FTDI_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modem
					     status register */
#define FTDI_SIO_SET_EVENT_CHAR		6 /* Set the event character */
#define FTDI_SIO_SET_ERROR_CHAR		7 /* Set the error character */
#define FTDI_SIO_SET_LATENCY_TIMER	9 /* Set the latency timer */
#define FTDI_SIO_GET_LATENCY_TIMER	0x0a /* Get the latency timer */
#define FTDI_SIO_SET_BITMODE		0x0b /* Set bitbang mode */
#define FTDI_SIO_READ_PINS		0x0c /* Read immediate value of pins */
#define FTDI_SIO_READ_EEPROM		0x90 /* Read EEPROM */

#define FTDI_SIO_SET_DTR_MASK 0x1
#define FTDI_SIO_SET_RTS_MASK 0x2

/** MPSSE bitbang modes */
enum ftdi_mpsse_mode
{
    BITMODE_RESET  = 0x00,    /**< switch off bitbang mode, back to regular serial/FIFO */
    BITMODE_BITBANG= 0x01,    /**< classical asynchronous bitbang mode, introduced with B-type chips */
    BITMODE_MPSSE  = 0x02,    /**< MPSSE mode, available on 2232x chips */
    BITMODE_SYNCBB = 0x04,    /**< synchronous bitbang mode, available on 2232x and R-type chips  */
    BITMODE_MCU    = 0x08,    /**< MCU Host Bus Emulation mode, available on 2232x chips */
    /* CPU-style fifo mode gets set via EEPROM */
    BITMODE_OPTO   = 0x10,    /**< Fast Opto-Isolated Serial Interface Mode, available on 2232x chips  */
    BITMODE_CBUS   = 0x20,    /**< Bitbang on CBUS pins of R-type chips, configure in EEPROM before */
    BITMODE_SYNCFF = 0x40,    /**< Single Channel Synchronous FIFO mode, available on 2232H chips */
    BITMODE_FT1284 = 0x80,    /**< FT1284 mode, available on 232H chips */
};

/* Control request handler */


uint8_t bulkout_buf[2][64] = {{0x01, 0x60}, {0x01, 0x60}};
volatile uint8_t latency_timer[2] = {16, 16};

uint8_t dtr = 1, rts = 1;
//volatile uint32_t baudrate_divisor[2];

uint8_t handler_buf[8];
/* 尴尬的双串口，虽然有一个根本没用 */

volatile uint8_t mpsse_enable = 0;


#define C_CLK  48000000
static void uart_set_baudrate(int itdf_divisor)
{
	uint8_t frac[] = {0, 8, 4, 2, 6, 10, 12, 14};
	int divisor = itdf_divisor & 0x3fff;
	divisor <<= 4;
	divisor |= frac[itdf_divisor >> 14];
	int baudrate = C_CLK / divisor;
	
	usart_set_baudrate(USART1, baudrate);
}

static int ec_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;
	(void)len;
	
	int port = req->wIndex & 0x03;
	
	*len = 0;

	if(req->bmRequestType == FTDI_SET_REQUEST_TYPE)
	{
		switch (req->bRequest)
		{ 
			case FTDI_SIO_SET_LATENCY_TIMER:
			{
				latency_timer[port] = req->wValue & 0xff;
				return 1;
			}
			case FTDI_SIO_SET_BAUD_RATE:
			{
				if(port == 2)
				{//设置波特率
					uint8_t Baudrate_High = (req->wIndex >> 8);
					uart_set_baudrate(req->wValue | (Baudrate_High << 16));
				}
				return 1;
			}
			case FTDI_SIO_MODEM_CTRL:
			{
				if(port == 2) /* 忽略掉 MPSSE口 */
				{
				#if 1
					uint8_t wValueH = req->wValue >> 8;
					if(wValueH & FTDI_SIO_SET_DTR_MASK)
					{
						if(req->wValue & FTDI_SIO_SET_DTR_MASK)
						{
							dtr = 1;
						}
						else
						{
							dtr = 0;
						}
					}
					if(wValueH & FTDI_SIO_SET_RTS_MASK)
					{
						if(req->wValue & FTDI_SIO_SET_RTS_MASK)
						{
							rts = 1;
						}
						else
						{
							rts = 0;
						}
					}
					if(dtr == rts)
					{
						dtr_set();
						rts_set();
					}
					if(dtr == 1 && rts == 0)
					{
						rts_clr();
						dtr_clr();
					}
					if(dtr == 0 && rts == 1)
					{
						rts_set();
						dtr_clr();
					}
				#endif
				}
				return 1;
			}
			case FTDI_SIO_SET_BITMODE:
			{
				if(port == 1) /* MPSSE口 */
				{
					enum ftdi_mpsse_mode mode = req->wValue >> 8;
					uint8_t bitmask = req->wValue & 0xff;
					if(mode == BITMODE_RESET)
					{//BITMODE复位
						mpsse_enable = 0;
					}
					else if(mode == BITMODE_MPSSE)
					{//TODO: 仅支持MPSSE
						mpsse_enable = 1;
					}
				}
				return 1;
			}
		}
	}
	else
	{
		switch (req->bRequest)
		{ 
			case FTDI_SIO_GET_LATENCY_TIMER:
			{
				handler_buf[0] = latency_timer[port];
				*buf = handler_buf;
				*len = 1; /* 送回latency_timer */
				return 1;
			}
			case FTDI_SIO_GET_MODEM_STATUS:
			{
				handler_buf[0] = 0x60;
				*buf = handler_buf;
				*len = 1; /* Modem status */
				return 1;
			}
			case FTDI_SIO_READ_EEPROM: /* 直接全部返回FF */
			{
				handler_buf[0] = 0xff;
				handler_buf[1] = 0xff;
				*len = 2;
				*buf = handler_buf;
				return 1;
			}
		}	
	}
	return 1; //TODO: 实现所有FT2232功能，这个时候只要微笑就好了
}

/* USB数据处理开始 */

typedef int32_t ring_size_t;

struct ring {
	uint8_t *data;
	ring_size_t size;
	uint32_t begin;
	uint32_t end;
};

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)

static void ring_init(volatile struct ring *ring, uint8_t *buf, ring_size_t size)
{
	ring->data = buf;
	ring->size = size;
	ring->begin = 0;
	ring->end = 0;
}

static inline int32_t ring_write_ch(volatile struct ring *ring, uint8_t ch)
{
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;
		return (uint32_t)ch;
	}

	return -1;
}

static inline int32_t ring_write(volatile struct ring *ring, uint8_t *data, ring_size_t size)
{
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static inline int32_t ring_read_ch(volatile struct ring *ring, uint8_t *ch)
{
	int32_t ret = -1;

	if (ring->begin != ring->end) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

static inline int32_t ring_read(volatile struct ring *ring, uint8_t *data, ring_size_t size)
{
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_read_ch(ring, data + i) < 0)
			return i;
	}

	return -i;
}


static inline uint32_t ring_size(volatile struct ring *ring)
{
	int size = (ring->end - ring->begin);
	if (size < 0) size = size + ring->size;
	return size;
}

static inline uint32_t ring_remain(volatile struct ring *ring)
{
	return ring->size - ring_size(ring);
}


#define BUFFER_SIZE_IN 256
#define BUFFER_SIZE_OUT 384

#define SERIAL_IN_SINGLEBUF 1

volatile struct ring jtag_in_ring;
volatile struct ring jtag_out_ring;
#if SERIAL_IN_SINGLEBUF
volatile struct ring serial_in_ring;
#endif
volatile struct ring serial_out_ring;

/* 256 byte的 收发缓冲区 */
 uint8_t ringbuf_jtag_in_buffer[BUFFER_SIZE_IN];
 uint8_t ringbuf_jtag_out_buffer[BUFFER_SIZE_OUT];
 uint8_t ringbuf_serial_in_buffer[BUFFER_SIZE_IN];
 uint8_t ringbuf_serial_out_buffer[BUFFER_SIZE_OUT];

static void ring_init_all(void)
{
	ring_init(&jtag_in_ring, ringbuf_jtag_in_buffer, BUFFER_SIZE_IN);
	ring_init(&jtag_out_ring, ringbuf_jtag_out_buffer, BUFFER_SIZE_OUT);
#if SERIAL_IN_SINGLEBUF
	ring_init(&serial_in_ring, ringbuf_serial_in_buffer, BUFFER_SIZE_IN);
#endif
	ring_init(&serial_out_ring, ringbuf_serial_out_buffer, BUFFER_SIZE_OUT);
}

/* 环形缓冲区结束 */

static void jtag_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	uint8_t buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x02, buf, 64);
	
	if(len)
	{
		ring_write(&jtag_in_ring, buf, len);
	}
	
	if(ring_remain(&jtag_in_ring) < 128) //缓冲区满
	{
		usbd_ep_nak_set(usbd_dev, 0x02, 1); //阻塞
	}
	
	gpio_toggle(GPIOB, GPIO2);
}

#if SERIAL_IN_SINGLEBUF
uint8_t recv_buf[64];
uint8_t recv_len;
uint8_t recv_i;
#endif

static void serial_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
#if SERIAL_IN_SINGLEBUF
	recv_len = usbd_ep_read_packet(usbd_dev, 0x04, recv_buf, 64);
	if (recv_len)
	{
		usbd_ep_nak_set(usbd_dev, 0x04, 1); //阻塞
		recv_i = 0;
		USART_CR1(USART1) |= USART_CR1_TXEIE;//开USART1空发送中断
	}
#else
	uint8_t buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x04, buf, 64);
	int remains = ring_remain(&serial_in_ring);
	
	if(remains - len < 64)
	{
		usbd_ep_nak_set(usbd_dev, 0x04, 1); //阻塞
	}
		
	if(len)
	{
		ring_write(&serial_in_ring, buf, len);
		USART_CR1(USART1) |= USART_CR1_TXEIE;//开USART1空发送中断
	}
#endif
}

/* 数据处理结束 */
volatile uint8_t attached = 0;

static void ec_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	
	
	usbd_register_control_callback(
				usbd_dev,
				FTDI_SET_REQUEST_TYPE,
				FTDI_SET_REQUEST_TYPE,
				ec_control_request);

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64,
			NULL);
	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, jtag_data_rx_cb); //JTAG

	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64,
			NULL);
	usbd_ep_setup(usbd_dev, 0x04, USB_ENDPOINT_ATTR_BULK, 64, serial_data_rx_cb); //Serial
	
	attached = 1;
}

static void interrupt_setup(void)
{
	/* 开一个 1ms 的定时器 */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); /* 72MHz / 8 = 9MHz */
	/* 定时器每N次中断一次 */
	systick_set_reload(8999); /* 9000 / 9000 = 1kHz */
	systick_interrupt_enable();
	systick_counter_enable();

	nvic_set_priority(NVIC_USART1_IRQ, 0);//串口最高优先级
	nvic_set_priority(NVIC_USB_WAKEUP_IRQ, 1 << 4);
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1 << 4);
	nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 1 << 4);
	nvic_set_priority(NVIC_SYSTICK_IRQ, 2 << 4);//systick最低优先级
	
	nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);


	/* 开接收中断 */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	__asm__("cpsie i"); 
}

volatile uint16_t timer_count = 0;
volatile uint16_t last_send = 0;

void sys_tick_handler(void)
{
	//gpio_toggle(GPIOB, GPIO2);
	timer_count ++;
}

static void usb_packet_handler(void)
{
	uint8_t timeout;
	if(ring_size(&serial_out_ring) > 62 && usbd_ep_stall_get(usbd_dev_handler, 0x83) == 0) //需要接收 (串口)
	{
		ring_read(&serial_out_ring, bulkout_buf[1] + 2, 62);//读62个byte
		//timeout = 0;while(usbd_ep_write_packet(usbd_dev_handler, 0x83, bulkout_buf[1], 64) == 0) {timeout++; if (timeout > 16) break;} //发MPSSE
		usbd_ep_write_packet(usbd_dev_handler, 0x83, bulkout_buf[1], 64);
	}

	if(ring_size(&jtag_out_ring) > 62 && usbd_ep_stall_get(usbd_dev_handler, 0x81) == 0) //需要接收 (MPSSE)
	{
		ring_read(&jtag_out_ring, bulkout_buf[0] + 2, 62);//读62个byte
		//timeout = 0;while(usbd_ep_write_packet(usbd_dev_handler, 0x81, bulkout_buf[1], 64) == 0) {timeout++; if (timeout > 16) break;} //发出去
		usbd_ep_write_packet(usbd_dev_handler, 0x81, bulkout_buf[1], 64);
	}	
	
	if((unsigned)(timer_count - last_send) > latency_timer[0]) //超时
	{
		last_send = timer_count;
		int len;
		if(usbd_ep_stall_get(usbd_dev_handler, 0x81) == 0)
		{
			len = ring_read(&jtag_out_ring, bulkout_buf[0] + 2, 62);//读N个byte
			//timeout = 0;while(usbd_ep_write_packet(usbd_dev_handler, 0x81, bulkout_buf[0], 2 + len) == 0) {timeout++; if (timeout > 16) break;} //发MPSSE
			usbd_ep_write_packet(usbd_dev_handler, 0x81, bulkout_buf[0], 2 + len);
		}
		if(usbd_ep_stall_get(usbd_dev_handler, 0x83) == 0)
		{
			len = ring_read(&serial_out_ring, bulkout_buf[1] + 2, 62);//读N个byte
			//timeout = 0;while(usbd_ep_write_packet(usbd_dev_handler, 0x83, bulkout_buf[1], 2 + len) == 0) {timeout++; if (timeout > 16) break;} //发串口
			usbd_ep_write_packet(usbd_dev_handler, 0x83, bulkout_buf[1], 2 + len);
		}
	}
}
/* 串口开始 */
static void uart_setup(void)
{
	//启动IO口
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX); //TODO:奇偶校验，stop bit

	usart_enable(USART1);	
}

void usart1_isr(void) //串口中断
{
	//gpio_toggle(GPIOB, GPIO2);
	if ((USART_SR(USART1) & USART_SR_RXNE) != 0) {
		ring_write_ch(&serial_out_ring, USART_DR(USART1) & USART_DR_MASK); //接收
	}

	/* 发送完成中断 */
	if (((USART_SR(USART1) & USART_SR_TXE) != 0) && ((USART_CR1(USART1) & USART_CR1_TXEIE) != 0)) 
	{
#if SERIAL_IN_SINGLEBUF
		USART_DR(USART1) = recv_buf[recv_i] & USART_DR_MASK;
		recv_i ++;
		if(recv_i >= recv_len)
		{
			usbd_ep_nak_set(usbd_dev_handler, 0x04, 0); //开放USB接收
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		}
#else
		volatile int32_t data = ring_read_ch(&serial_in_ring, NULL);
		
		if (data == -1) { //没有即停止
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
			usbd_ep_nak_set(usbd_dev_handler, 0x04, 0); //开放USB接收
		} else {
			USART_DR(USART1) = data & USART_DR_MASK; //发送数据
		}
#endif
	}
}

/* 串口结束 */

/* MPSSE处理 */

static void mpsse_process(void)
{
	if(mpsse_enable == 1)
	{
		
	}
}

int main(void)
{
	volatile int i;
	
	clock_setup();
	
	gpio_clear(GPIOB, GPIO2);
	gpio_clear(GPIOB, GPIO11);
	
	ring_init_all(); //开环形缓冲区
	gpio_setup();
	uart_setup();
	
	gpio_clear(GPIOA, GPIO8); //关USB上拉 
	
	usbd_dev_handler = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 2, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev_handler, ec_set_config);
	
	interrupt_setup();/* 开中断 */

	for (i = 0; i < 0x80000; i++)
		__asm__("nop");
	gpio_set(GPIOA, GPIO8);//开USB上拉

		
	while(1)
	{
		//usbd_poll(usbd_dev_handler);
		if(attached)
			usb_packet_handler();
		mpsse_process();
	}

	return 0;
}
