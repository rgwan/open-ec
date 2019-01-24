#define ITDF_SET_REQUEST_TYPE 0x40
#define ITDF_GET_REQUEST_TYPE 0xC0

#define ITDF_SIO_RESET			0 /* Reset the port */
#define ITDF_SIO_MODEM_CTRL		1 /* Set the modem control register */
#define ITDF_SIO_SET_FLOW_CTRL		2 /* Set flow control register */
#define ITDF_SIO_SET_BAUD_RATE		3 /* Set baud rate */
#define ITDF_SIO_SET_DATA		4 /* Set the data characteristics of
					     the port */
#define ITDF_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modem
					     status register */
#define ITDF_SIO_SET_EVENT_CHAR		6 /* Set the event character */
#define ITDF_SIO_SET_ERROR_CHAR		7 /* Set the error character */
#define ITDF_SIO_SET_LATENCY_TIMER	9 /* Set the latency timer */
#define ITDF_SIO_GET_LATENCY_TIMER	0x0a /* Get the latency timer */
#define ITDF_SIO_SET_BITMODE		0x0b /* Set bitbang mode */
#define ITDF_SIO_READ_PINS		0x0c /* Read immediate value of pins */
#define ITDF_SIO_READ_EEPROM		0x90 /* Read EEPROM */

#define ITDF_SIO_WRITE_EEPROM		0x91
#define ITDF_SIO_TEST_EEPROM		0x92

#define ITDF_SIO_SET_DTR_MASK 0x1
#define ITDF_SIO_SET_RTS_MASK 0x2

/** SESPM bitbang modes */
enum itdf_sespm_mode
{
    BITMODE_RESET  = 0x00,    /**< switch off bitbang mode, back to regular serial/FIFO */
    BITMODE_BITBANG= 0x01,    /**< classical asynchronous bitbang mode, introduced with B-type chips */
    BITMODE_SESPM  = 0x02,    /**< SESPM mode, available on 2232x chips */
    BITMODE_SYNCBB = 0x04,    /**< synchronous bitbang mode, available on 2232x and R-type chips  */
    BITMODE_MCU    = 0x08,    /**< MCU Host Bus Emulation mode, available on 2232x chips */
    /* CPU-style fifo mode gets set via EEPROM */
    BITMODE_OPTO   = 0x10,    /**< Fast Opto-Isolated Serial Interface Mode, available on 2232x chips  */
    BITMODE_CBUS   = 0x20,    /**< Bitbang on CBUS pins of R-type chips, configure in EEPROM before */
    BITMODE_SYNCFF = 0x40,    /**< Single Channel Synchronous FIFO mode, available on 2232H chips */
    BITMODE_FT1284 = 0x80,    /**< FT1284 mode, available on 232H chips */
};

const uint16_t itdf_eeprom [] =
{
	0x0800, 0x0403, 0x6010, 0x0500, 0x3280, 0x0000, 0x0200, 0x1096,
	0x1aa6, 0x0000, 0x0046, 0x0310, 0x004f, 0x0070, 0x0065, 0x006e,
	0x002d, 0x0045, 0x0043, 0x031a, 0x0055, 0x0053, 0x0042, 0x0020,
	0x0044, 0x0065, 0x0062, 0x0075, 0x0067, 0x0067, 0x0065, 0x0072,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1027 
};

#define ITDF_SIE_WRITE_MCE		(1 << 0)
#define ITDF_SIE_BIT_MODE		(1 << 1)
#define ITDF_SIE_READ_MCE		(1 << 2)
#define ITDF_SIE_LSB_FIRST		(1 << 3)
#define ITDF_SIE_WRITE_TDI		(1 << 4)
#define ITDF_SIE_READ_TDO		(1 << 5)
#define ITDF_SIE_WRITE_TMS		(1 << 6)
#define ITDF_SIE_SPECIAL		(1 << 7)

#define DELAY() asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop")

enum sespm_state_t
{
	IDLE = 0,
	/* TDI 字节传输 */
	TDI_RECV_LENGL_BYTE = 1,
	TDI_RECV_LENGH_BYTE = 2,
	TDI_DATA_BYTE = 3,
#if 0
	/* TMS 字节传输, 不使用 */
	TMS_RECV_LENGL_BYTE = 4,
	TMS_RECV_LENGH_BYTE = 5,
	TMS_DATA_BYTE = 6,
#endif
	/* Bitbang */
	BUS_DATA = 7,
	BUS_DIR = 8,
	/* TDI 位传输 */
	TDI_RECV_LENG_BIT = 9,
	TDI_DATA_BIT = 10,
	/* TMS 位传输 */
	TMS_RECV_LENG_BIT = 11,
	TMS_DATA_BIT = 12
};
