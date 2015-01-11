# -*- coding: utf-8 -*-
__author__ = 'vahid'

from _bcm2835 cimport *


"""
Common constants
"""
HIGH = c_HIGH  # This means pin HIGH, true, 3.3volts on a pin.
LOW = c_LOW  # This means pin LOW, false, 0volts on a pin.
CORE_CLK_HZ = c_BCM2835_CORE_CLK_HZ  # Speed of the core clock core_clk
PERI_BASE = c_BCM2835_PERI_BASE  # Physical addresses for various peripheral register sets Base Physical Address of the BCM 2835 peripheral registers
ST_BASE = c_BCM2835_ST_BASE  # Base Physical Address of the System Timer registers
CLOCK_BASE = c_BCM2835_CLOCK_BASE  # Base Physical Address of the Clock/timer registers
SPI0_BASE = c_BCM2835_SPI0_BASE  # Base Physical Address of the SPI0 registers
BSC0_BASE = c_BCM2835_BSC0_BASE  # Base Physical Address of the BSC0 registers
BSC1_BASE = c_BCM2835_BSC1_BASE  # Base Physical Address of the BSC1 registers
PAGE_SIZE = c_BCM2835_PAGE_SIZE  # Size of memory page on RPi
BLOCK_SIZE = c_BCM2835_BLOCK_SIZE  # Size of memory block on RPi
GPIO_BASE = c_BCM2835_GPIO_BASE  # Base Physical Address of the GPIO registers
GPIO_PADS = c_BCM2835_GPIO_PADS  # Base Physical Address of the Pads registers
GPIO_PWM = c_BCM2835_GPIO_PWM  # Base Physical Address of the PWM registers


"""
Defines for GPIO
The BCM2835 has 54 GPIO pins.
  BCM2835 data sheet, Page 90 onwards.
  GPIO register offsets from BCM2835_GPIO_BASE. Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
"""
GPFSEL0 = c_BCM2835_GPFSEL0       # GPIO Function Select 0
GPFSEL1 = c_BCM2835_GPFSEL1       # GPIO Function Select 1
GPFSEL2 = c_BCM2835_GPFSEL2       # GPIO Function Select 2
GPFSEL3 = c_BCM2835_GPFSEL3       # GPIO Function Select 3
GPFSEL4 = c_BCM2835_GPFSEL4       # GPIO Function Select 4
GPFSEL5 = c_BCM2835_GPFSEL5       # GPIO Function Select 5
GPSET0 = c_BCM2835_GPSET0         # GPIO Pin Output Set 0
GPSET1 = c_BCM2835_GPSET1         # GPIO Pin Output Set 1
GPCLR0 = c_BCM2835_GPCLR0         # GPIO Pin Output Clear 0
GPCLR1 = c_BCM2835_GPCLR1         # GPIO Pin Output Clear 1
GPLEV0 = c_BCM2835_GPLEV0         # GPIO Pin Level 0
GPLEV1 = c_BCM2835_GPLEV1         # GPIO Pin Level 1
GPEDS0 = c_BCM2835_GPEDS0         # GPIO Pin Event Detect Status 0
GPEDS1 = c_BCM2835_GPEDS1         # GPIO Pin Event Detect Status 1
GPREN0 = c_BCM2835_GPREN0         # GPIO Pin Rising Edge Detect Enable 0
GPREN1 = c_BCM2835_GPREN1         # GPIO Pin Rising Edge Detect Enable 1
GPFEN0 = c_BCM2835_GPFEN0         # GPIO Pin Falling Edge Detect Enable 0
GPFEN1 = c_BCM2835_GPFEN1         # GPIO Pin Falling Edge Detect Enable 1
GPHEN0 = c_BCM2835_GPHEN0         # GPIO Pin High Detect Enable 0
GPHEN1 = c_BCM2835_GPHEN1         # GPIO Pin High Detect Enable 1
GPLEN0 = c_BCM2835_GPLEN0         # GPIO Pin Low Detect Enable 0
GPLEN1 = c_BCM2835_GPLEN1         # GPIO Pin Low Detect Enable 1
GPAREN0 = c_BCM2835_GPAREN0       # GPIO Pin Async. Rising Edge Detect 0
GPAREN1 = c_BCM2835_GPAREN1       # GPIO Pin Async. Rising Edge Detect 1
GPAFEN0 = c_BCM2835_GPAFEN0       # GPIO Pin Async. Falling Edge Detect 0
GPAFEN1 = c_BCM2835_GPAFEN1       # GPIO Pin Async. Falling Edge Detect 1
GPPUD = c_BCM2835_GPPUD           # GPIO Pin Pull-up/down Enable
GPPUDCLK0 = c_BCM2835_GPPUDCLK0   # GPIO Pin Pull-up/down Enable Clock 0
GPPUDCLK1 = c_BCM2835_GPPUDCLK1   # GPIO Pin Pull-up/down Enable Clock 1


"""
Defines for SPI
GPIO register offsets from BCM2835_SPI0_BASE.
Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
"""
SPI0_CS = c_BCM2835_SPI0_CS  # SPI Master Control and Status
SPI0_FIFO = c_BCM2835_SPI0_FIFO  # SPI Master TX and RX FIFOs
SPI0_CLK = c_BCM2835_SPI0_CLK  # SPI Master Clock Divider
SPI0_DLEN = c_BCM2835_SPI0_DLEN  # SPI Master Data Length
SPI0_LTOH = c_BCM2835_SPI0_LTOH  # SPI LOSSI mode TOH
SPI0_DC = c_BCM2835_SPI0_DC  # SPI DMA DREQ Controls


"""
Register masks for SPI0_CS
"""
SPI0_CS_LEN_LONG = c_BCM2835_SPI0_CS_LEN_LONG  # Enable Long data word in Lossi mode if DMA_LEN is set
SPI0_CS_DMA_LEN = c_BCM2835_SPI0_CS_DMA_LEN  # Enable DMA mode in Lossi mode
SPI0_CS_CSPOL2 = c_BCM2835_SPI0_CS_CSPOL2  # Chip Select 2 Polarity
SPI0_CS_CSPOL1 = c_BCM2835_SPI0_CS_CSPOL1  # Chip Select 1 Polarity
SPI0_CS_CSPOL0 = c_BCM2835_SPI0_CS_CSPOL0  # Chip Select 0 Polarity
SPI0_CS_RXF = c_BCM2835_SPI0_CS_RXF  # RXF - RX FIFO Full
SPI0_CS_RXR = c_BCM2835_SPI0_CS_RXR  # RXR RX FIFO needs Reading ( full)
SPI0_CS_TXD = c_BCM2835_SPI0_CS_TXD  # TXD TX FIFO can accept Data
SPI0_CS_RXD = c_BCM2835_SPI0_CS_RXD  # RXD RX FIFO contains Data
SPI0_CS_DONE = c_BCM2835_SPI0_CS_DONE  # Done transfer Done
SPI0_CS_TE_EN = c_BCM2835_SPI0_CS_TE_EN  # Unused
SPI0_CS_LMONO = c_BCM2835_SPI0_CS_LMONO  # Unused
SPI0_CS_LEN = c_BCM2835_SPI0_CS_LEN  # LEN LoSSI enable
SPI0_CS_REN = c_BCM2835_SPI0_CS_REN  # REN Read Enable
SPI0_CS_ADCS = c_BCM2835_SPI0_CS_ADCS  # ADCS Automatically Deassert Chip Select
SPI0_CS_INTR = c_BCM2835_SPI0_CS_INTR  # INTR Interrupt on RXR
SPI0_CS_INTD = c_BCM2835_SPI0_CS_INTD  # INTD Interrupt on Done
SPI0_CS_DMAEN = c_BCM2835_SPI0_CS_DMAEN  # DMAEN DMA Enable
SPI0_CS_TA = c_BCM2835_SPI0_CS_TA  # Transfer Active
SPI0_CS_CSPOL = c_BCM2835_SPI0_CS_CSPOL  # Chip Select Polarity
SPI0_CS_CLEAR = c_BCM2835_SPI0_CS_CLEAR  # Clear FIFO Clear RX and TX
SPI0_CS_CLEAR_RX = c_BCM2835_SPI0_CS_CLEAR_RX  # Clear FIFO Clear RX
SPI0_CS_CLEAR_TX = c_BCM2835_SPI0_CS_CLEAR_TX  # Clear FIFO Clear TX
SPI0_CS_CPOL = c_BCM2835_SPI0_CS_CPOL  # Clock Polarity
SPI0_CS_CPHA = c_BCM2835_SPI0_CS_CPHA  # Clock Phase
SPI0_CS_CS = c_BCM2835_SPI0_CS_CS  # Chip Select


"""
Defines for I2C
GPIO register offsets from BCM2835_BSC*_BASE.
Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
"""
BSC_C = c_BCM2835_BSC_C  # BSC Master Control
BSC_S = c_BCM2835_BSC_S  # BSC Master Status
BSC_DLEN = c_BCM2835_BSC_DLEN  # BSC Master Data Length
BSC_A = c_BCM2835_BSC_A  # BSC Master Slave Address
BSC_FIFO = c_BCM2835_BSC_FIFO  # BSC Master Data FIFO
BSC_DIV = c_BCM2835_BSC_DIV  # BSC Master Clock Divider
BSC_DEL = c_BCM2835_BSC_DEL  # BSC Master Data Delay
BSC_CLKT = c_BCM2835_BSC_CLKT  # BSC Master Clock Stretch Timeout


"""
Register masks for BSC_C
"""
BSC_C_I2CEN = c_BCM2835_BSC_C_I2CEN  # I2C Enable, 0 = disabled, 1 = enabled
BSC_C_INTR = c_BCM2835_BSC_C_INTR  # Interrupt on RX
BSC_C_INTT = c_BCM2835_BSC_C_INTT  # Interrupt on TX
BSC_C_INTD = c_BCM2835_BSC_C_INTD  # Interrupt on DONE
BSC_C_ST = c_BCM2835_BSC_C_ST  # Start transfer, 1 = Start a new transfer
BSC_C_CLEAR_1 = c_BCM2835_BSC_C_CLEAR_1  # Clear FIFO Clear
BSC_C_CLEAR_2 = c_BCM2835_BSC_C_CLEAR_2  # Clear FIFO Clear
BSC_C_READ = c_BCM2835_BSC_C_READ  # Read transfer


"""
Register masks for BSC_S
"""
BSC_S_CLKT = c_BCM2835_BSC_S_CLKT  # Clock stretch timeout
BSC_S_ERR = c_BCM2835_BSC_S_ERR  # ACK error
BSC_S_RXF = c_BCM2835_BSC_S_RXF  # RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
BSC_S_TXE = c_BCM2835_BSC_S_TXE  # TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
BSC_S_RXD = c_BCM2835_BSC_S_RXD  # RXD FIFO contains data
BSC_S_TXD = c_BCM2835_BSC_S_TXD  # TXD FIFO can accept data
BSC_S_RXR = c_BCM2835_BSC_S_RXR  # RXR FIFO needs reading (full)
BSC_S_TXW = c_BCM2835_BSC_S_TXW  # TXW FIFO needs writing (full)
BSC_S_DONE = c_BCM2835_BSC_S_DONE  # Transfer DONE
BSC_S_TA = c_BCM2835_BSC_S_TA  # Transfer Active
BSC_FIFO_SIZE = c_BCM2835_BSC_FIFO_SIZE  # BSC FIFO size


"""
Defines for ST
GPIO register offsets from BCM2835_ST_BASE.
Offsets into the ST Peripheral block in bytes per 12.1 System Timer Registers
The System Timer peripheral provides four 32-bit timer channels and a single 64-bit free running counter.
BCM2835_ST_CLO is the System Timer Counter Lower bits register.
The system timer free-running counter lower register is a read-only register that returns the current value
of the lower 32-bits of the free running counter.
BCM2835_ST_CHI is the System Timer Counter Upper bits register.
The system timer free-running counter upper register is a read-only register that returns the current value
"""
ST_CS = c_BCM2835_ST_CS  # System Timer Control/Status
ST_CLO = c_BCM2835_ST_CLO  # System Timer Counter Lower 32 bits
ST_CHI = c_BCM2835_ST_CHI  # System Timer Counter Upper 32 bits


"""
Defines for PWM, word offsets (ie 4 byte multiples)
"""
PWM_CONTROL = c_BCM2835_PWM_CONTROL
PWM_STATUS = c_BCM2835_PWM_STATUS
PWM_DMAC = c_BCM2835_PWM_DMAC
PWM0_RANGE = c_BCM2835_PWM0_RANGE
PWM0_DATA = c_BCM2835_PWM0_DATA
PWM_FIF1 = c_BCM2835_PWM_FIF1
PWM1_RANGE = c_BCM2835_PWM1_RANGE
PWM1_DATA = c_BCM2835_PWM1_DATA


"""
Defines for PWM Clock, word offsets (ie 4 byte multiples)
"""
PWMCLK_CNTL = c_BCM2835_PWMCLK_CNTL
PWMCLK_DIV = c_BCM2835_PWMCLK_DIV
PWM_PASSWRD = c_BCM2835_PWM_PASSWRD  # Password to enable setting PWM clock

PWM1_MS_MODE = c_BCM2835_PWM1_MS_MODE  # Run in Mark/Space mode
PWM1_USEFIFO = c_BCM2835_PWM1_USEFIFO  # Data from FIFO
PWM1_REVPOLAR = c_BCM2835_PWM1_REVPOLAR  # Reverse polarity
PWM1_OFFSTATE = c_BCM2835_PWM1_OFFSTATE  # Ouput Off state
PWM1_REPEATFF = c_BCM2835_PWM1_REPEATFF  # Repeat last value if FIFO empty
PWM1_SERIAL = c_BCM2835_PWM1_SERIAL  # Run in serial mode
PWM1_ENABLE = c_BCM2835_PWM1_ENABLE  # Channel Enable

PWM0_MS_MODE = c_BCM2835_PWM0_MS_MODE  # Run in Mark/Space mode
PWM_CLEAR_FIFO = c_BCM2835_PWM_CLEAR_FIFO  # Clear FIFO
PWM0_USEFIFO = c_BCM2835_PWM0_USEFIFO  # Data from FIFO
PWM0_REVPOLAR = c_BCM2835_PWM0_REVPOLAR  # Reverse polarity
PWM0_OFFSTATE = c_BCM2835_PWM0_OFFSTATE  # Ouput Off state
PWM0_REPEATFF = c_BCM2835_PWM0_REPEATFF  # Repeat last value if FIFO empty
PWM0_SERIAL = c_BCM2835_PWM0_SERIAL  # Run in serial mode
PWM0_ENABLE = c_BCM2835_PWM0_ENABLE  # Channel Enable

## Enums

"""
Port function select modes for bcm2835_gpio_fsel()
"""
GPIO_FSEL_INPT = c_BCM2835_GPIO_FSEL_INPT  # Input
GPIO_FSEL_OUTP = c_BCM2835_GPIO_FSEL_OUTP  # Output
GPIO_FSEL_ALT0 = c_BCM2835_GPIO_FSEL_ALT0  # Alternate function 0
GPIO_FSEL_ALT1 = c_BCM2835_GPIO_FSEL_ALT1  # Alternate function 1
GPIO_FSEL_ALT2 = c_BCM2835_GPIO_FSEL_ALT2  # Alternate function 2
GPIO_FSEL_ALT3 = c_BCM2835_GPIO_FSEL_ALT3  # Alternate function 3
GPIO_FSEL_ALT4 = c_BCM2835_GPIO_FSEL_ALT4  # Alternate function 4
GPIO_FSEL_ALT5 = c_BCM2835_GPIO_FSEL_ALT5  # Alternate function 5
GPIO_FSEL_MASK = c_BCM2835_GPIO_FSEL_MASK  # Function select bits mask


"""
Pullup/Pulldown defines for bcm2835_gpio_pud()
"""
GPIO_PUD_OFF = c_BCM2835_GPIO_PUD_OFF  # Off ? disable pull-up/down
GPIO_PUD_DOWN = c_BCM2835_GPIO_PUD_DOWN  # Enable Pull Down control
GPIO_PUD_UP = c_BCM2835_GPIO_PUD_UP  # Enable Pull Up control


"""
Here we define Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
These can be passed as a pin number to any function requiring a pin.
Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
and some can adopt an alternate function.
RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
When SPI0 is in use (ie after bcm2835_spi_begin()), pins 19, 21, 23, 24, 26 are dedicated to SPI
and cant be controlled independently
"""
# RPI Version 1
RPI_GPIO_P1_03 = c_RPI_GPIO_P1_03  # Version 1, Pin P1-03
RPI_GPIO_P1_05 = c_RPI_GPIO_P1_05  # Version 1, Pin P1-05
RPI_GPIO_P1_07 = c_RPI_GPIO_P1_07  # Version 1, Pin P1-07
RPI_GPIO_P1_08 = c_RPI_GPIO_P1_08  # Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD
RPI_GPIO_P1_10 = c_RPI_GPIO_P1_10  # Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD
RPI_GPIO_P1_11 = c_RPI_GPIO_P1_11  # Version 1, Pin P1-11
RPI_GPIO_P1_12 = c_RPI_GPIO_P1_12  # Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5
RPI_GPIO_P1_13 = c_RPI_GPIO_P1_13  # Version 1, Pin P1-13
RPI_GPIO_P1_15 = c_RPI_GPIO_P1_15  # Version 1, Pin P1-15
RPI_GPIO_P1_16 = c_RPI_GPIO_P1_16  # Version 1, Pin P1-16
RPI_GPIO_P1_18 = c_RPI_GPIO_P1_18  # Version 1, Pin P1-18
RPI_GPIO_P1_19 = c_RPI_GPIO_P1_19  # Version 1, Pin P1-19, MOSI when SPI0 in use
RPI_GPIO_P1_21 = c_RPI_GPIO_P1_21  # Version 1, Pin P1-21, MISO when SPI0 in use
RPI_GPIO_P1_22 = c_RPI_GPIO_P1_22  # Version 1, Pin P1-22
RPI_GPIO_P1_23 = c_RPI_GPIO_P1_23  # Version 1, Pin P1-23, CLK when SPI0 in use
RPI_GPIO_P1_24 = c_RPI_GPIO_P1_24  # Version 1, Pin P1-24, CE0 when SPI0 in use
RPI_GPIO_P1_26 = c_RPI_GPIO_P1_26  # Version 1, Pin P1-26, CE1 when SPI0 in use

# RPI Version 2
RPI_V2_GPIO_P1_03 = c_RPI_V2_GPIO_P1_03  # Version 2, Pin P1-03
RPI_V2_GPIO_P1_05 = c_RPI_V2_GPIO_P1_05  # Version 2, Pin P1-05
RPI_V2_GPIO_P1_07 = c_RPI_V2_GPIO_P1_07  # Version 2, Pin P1-07
RPI_V2_GPIO_P1_08 = c_RPI_V2_GPIO_P1_08  # Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD
RPI_V2_GPIO_P1_10 = c_RPI_V2_GPIO_P1_10  # Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD
RPI_V2_GPIO_P1_11 = c_RPI_V2_GPIO_P1_11  # Version 2, Pin P1-11
RPI_V2_GPIO_P1_12 = c_RPI_V2_GPIO_P1_12  # Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5
RPI_V2_GPIO_P1_13 = c_RPI_V2_GPIO_P1_13  # Version 2, Pin P1-13
RPI_V2_GPIO_P1_15 = c_RPI_V2_GPIO_P1_15  # Version 2, Pin P1-15
RPI_V2_GPIO_P1_16 = c_RPI_V2_GPIO_P1_16  # Version 2, Pin P1-16
RPI_V2_GPIO_P1_18 = c_RPI_V2_GPIO_P1_18  # Version 2, Pin P1-18
RPI_V2_GPIO_P1_19 = c_RPI_V2_GPIO_P1_19  # Version 2, Pin P1-19, MOSI when SPI0 in use
RPI_V2_GPIO_P1_21 = c_RPI_V2_GPIO_P1_21  # Version 2, Pin P1-21, MISO when SPI0 in use
RPI_V2_GPIO_P1_22 = c_RPI_V2_GPIO_P1_22  # Version 2, Pin P1-22
RPI_V2_GPIO_P1_23 = c_RPI_V2_GPIO_P1_23  # Version 2, Pin P1-23, CLK when SPI0 in use
RPI_V2_GPIO_P1_24 = c_RPI_V2_GPIO_P1_24  # Version 2, Pin P1-24, CE0 when SPI0 in use
RPI_V2_GPIO_P1_26 = c_RPI_V2_GPIO_P1_26  # Version 2, Pin P1-26, CE1 when SPI0 in use
RPI_V2_GPIO_P5_03 = c_RPI_V2_GPIO_P5_03  # Version 2, Pin P5-03
RPI_V2_GPIO_P5_04 = c_RPI_V2_GPIO_P5_04  # Version 2, Pin P5-04
RPI_V2_GPIO_P5_05 = c_RPI_V2_GPIO_P5_05  # Version 2, Pin P5-05
RPI_V2_GPIO_P5_06 = c_RPI_V2_GPIO_P5_06  # Version 2, Pin P5-06


"""
Specifies the SPI data bit ordering for bcm2835_spi_setBitOrder()
"""
SPI_BIT_ORDER_LSBFIRST = c_BCM2835_SPI_BIT_ORDER_LSBFIRST  # ///< LSB First
SPI_BIT_ORDER_MSBFIRST = c_BCM2835_SPI_BIT_ORDER_MSBFIRST  # ///< MSB First


"""
Specify the SPI chip select pin(s)
"""
SPI_MODE0 = c_BCM2835_SPI_MODE0  # ///< CPOL = 0, CPHA = 0
SPI_MODE1 = c_BCM2835_SPI_MODE1  # ///< CPOL = 0, CPHA = 1
SPI_MODE2 = c_BCM2835_SPI_MODE2  # ///< CPOL = 1, CPHA = 0
SPI_MODE3 = c_BCM2835_SPI_MODE3  # ///< CPOL = 1, CPHA = 1

"""
Specify the SPI chip select pin(s)
"""
SPI_CS0 = c_BCM2835_SPI_CS0  # Chip Select 0
SPI_CS1 = c_BCM2835_SPI_CS1  # Chip Select 1
SPI_CS2 = c_BCM2835_SPI_CS2  # Chip Select 2 (ie pins CS1 and CS2 are asserted)
SPI_CS_NONE = c_BCM2835_SPI_CS_NONE  # No CS, control it yourself

"""
Specifies the divider used to generate the SPI clock from the system clock.
Figures below give the divider, clock period and clock frequency.
Clock divided is based on nominal base clock rate of 250MHz
It is reported that (contrary to the documentation) any even divider may used.
The frequencies shown for each divider have been confirmed by measurement
"""
SPI_CLOCK_DIVIDER_65536 = c_BCM2835_SPI_CLOCK_DIVIDER_65536  # ///< 65536 = 262.144us = 3.814697260kHz
SPI_CLOCK_DIVIDER_32768 = c_BCM2835_SPI_CLOCK_DIVIDER_32768  # ///< 32768 = 131.072us = 7.629394531kHz
SPI_CLOCK_DIVIDER_16384 = c_BCM2835_SPI_CLOCK_DIVIDER_16384  # ///< 16384 = 65.536us = 15.25878906kHz
SPI_CLOCK_DIVIDER_8192 = c_BCM2835_SPI_CLOCK_DIVIDER_8192  # ///< 8192 = 32.768us = 30/51757813kHz
SPI_CLOCK_DIVIDER_4096 = c_BCM2835_SPI_CLOCK_DIVIDER_4096  # ///< 4096 = 16.384us = 61.03515625kHz
SPI_CLOCK_DIVIDER_2048 = c_BCM2835_SPI_CLOCK_DIVIDER_2048  # ///< 2048 = 8.192us = 122.0703125kHz
SPI_CLOCK_DIVIDER_1024 = c_BCM2835_SPI_CLOCK_DIVIDER_1024  # ///< 1024 = 4.096us = 244.140625kHz
SPI_CLOCK_DIVIDER_512 = c_BCM2835_SPI_CLOCK_DIVIDER_512  # ///< 512 = 2.048us = 488.28125kHz
SPI_CLOCK_DIVIDER_256 = c_BCM2835_SPI_CLOCK_DIVIDER_256  # ///< 256 = 1.024us = 976.5625MHz
SPI_CLOCK_DIVIDER_128 = c_BCM2835_SPI_CLOCK_DIVIDER_128  # ///< 128 = 512ns = = 1.953125MHz
SPI_CLOCK_DIVIDER_64 = c_BCM2835_SPI_CLOCK_DIVIDER_64  # ///< 64 = 256ns = 3.90625MHz
SPI_CLOCK_DIVIDER_32 = c_BCM2835_SPI_CLOCK_DIVIDER_32  # ///< 32 = 128ns = 7.8125MHz
SPI_CLOCK_DIVIDER_16 = c_BCM2835_SPI_CLOCK_DIVIDER_16  # ///< 16 = 64ns = 15.625MHz
SPI_CLOCK_DIVIDER_8 = c_BCM2835_SPI_CLOCK_DIVIDER_8  # ///< 8 = 32ns = 31.25MHz
SPI_CLOCK_DIVIDER_4 = c_BCM2835_SPI_CLOCK_DIVIDER_4  # ///< 4 = 16ns = 62.5MHz
SPI_CLOCK_DIVIDER_2 = c_BCM2835_SPI_CLOCK_DIVIDER_2  # ///< 2 = 8ns = 125MHz, fastest you can get
SPI_CLOCK_DIVIDER_1 = c_BCM2835_SPI_CLOCK_DIVIDER_1  # ///< 1 = 262.144us = 3.814697260kHz, same as 0/65536


"""
Specifies the divider used to generate the I2C clock from the system clock.
Clock divided is based on nominal base clock rate of 250MHz
"""
I2C_CLOCK_DIVIDER_2500 = c_BCM2835_I2C_CLOCK_DIVIDER_2500  # ///< 2500 = 10us = 100 kHz
I2C_CLOCK_DIVIDER_626 = c_BCM2835_I2C_CLOCK_DIVIDER_626  # ///< 622 = 2.504us = 399.3610 kHz
I2C_CLOCK_DIVIDER_150 = c_BCM2835_I2C_CLOCK_DIVIDER_150  # ///< 150 = 60ns = 1.666 MHz (default at reset)
I2C_CLOCK_DIVIDER_148 = c_BCM2835_I2C_CLOCK_DIVIDER_148  # ///< 148 = 59ns = 1.689 MHz


"""
Specifies the reason codes for the bcm2835_i2c_write and bcm2835_i2c_read functions.
"""
I2C_REASON_OK = c_BCM2835_I2C_REASON_OK  # ///< Success
I2C_REASON_ERROR_NACK = c_BCM2835_I2C_REASON_ERROR_NACK  # ///< Received a NACK
I2C_REASON_ERROR_CLKT = c_BCM2835_I2C_REASON_ERROR_CLKT  # ///< Received Clock Stretch Timeout
I2C_REASON_ERROR_DATA = c_BCM2835_I2C_REASON_ERROR_DATA  # ///< Not all data is sent / received


"""
Specifies the divider used to generate the PWM clock from the system clock.
Figures below give the divider, clock period and clock frequency.
Clock divided is based on nominal PWM base clock rate of 19.2MHz
The frequencies shown for each divider have been confirmed by measurement
"""
PWM_CLOCK_DIVIDER_32768 = c_BCM2835_PWM_CLOCK_DIVIDER_32768  # ///< 32768 = 585Hz
PWM_CLOCK_DIVIDER_16384 = c_BCM2835_PWM_CLOCK_DIVIDER_16384  # ///< 16384 = 1171.8Hz
PWM_CLOCK_DIVIDER_8192 = c_BCM2835_PWM_CLOCK_DIVIDER_8192  # ///< 8192 = 2.34375kHz
PWM_CLOCK_DIVIDER_4096 = c_BCM2835_PWM_CLOCK_DIVIDER_4096  # ///< 4096 = 4.6875kHz
PWM_CLOCK_DIVIDER_2048 = c_BCM2835_PWM_CLOCK_DIVIDER_2048  # ///< 2048 = 9.375kHz
PWM_CLOCK_DIVIDER_1024 = c_BCM2835_PWM_CLOCK_DIVIDER_1024  # ///< 1024 = 18.75kHz
PWM_CLOCK_DIVIDER_512 = c_BCM2835_PWM_CLOCK_DIVIDER_512  # ///< 512 = 37.5kHz
PWM_CLOCK_DIVIDER_256 = c_BCM2835_PWM_CLOCK_DIVIDER_256  # ///< 256 = 75kHz
PWM_CLOCK_DIVIDER_128 = c_BCM2835_PWM_CLOCK_DIVIDER_128  # ///< 128 = 150kHz
PWM_CLOCK_DIVIDER_64 = c_BCM2835_PWM_CLOCK_DIVIDER_64  # ///< 64 = 300kHz
PWM_CLOCK_DIVIDER_32 = c_BCM2835_PWM_CLOCK_DIVIDER_32  # ///< 32 = 600.0kHz
PWM_CLOCK_DIVIDER_16 = c_BCM2835_PWM_CLOCK_DIVIDER_16  # ///< 16 = 1.2MHz
PWM_CLOCK_DIVIDER_8 = c_BCM2835_PWM_CLOCK_DIVIDER_8  # ///< 8 = 2.4MHz
PWM_CLOCK_DIVIDER_4 = c_BCM2835_PWM_CLOCK_DIVIDER_4  # ///< 4 = 4.8MHz
PWM_CLOCK_DIVIDER_2 = c_BCM2835_PWM_CLOCK_DIVIDER_2  # ///< 2 = 9.6MHz, fastest you can get
PWM_CLOCK_DIVIDER_1 = c_BCM2835_PWM_CLOCK_DIVIDER_1  # ///< 1 = 4.6875kHz, same as divider 4096

def init():
  return c_bcm2835_init()

__all__ = [
  'HIGH',
  'LOW',
  'CORE_CLK_HZ',
  'PERI_BASE',
  'ST_BASE',
  'CLOCK_BASE',
  'SPI0_BASE',
  'BSC0_BASE',
  'BSC1_BASE',
  'PAGE_SIZE',
  'BLOCK_SIZE',
  'GPIO_PADS',
  'GPIO_BASE',
  'GPIO_PWM',

  'GPFSEL0',
  'GPFSEL1',
  'GPFSEL2',
  'GPFSEL3',
  'GPFSEL4',
  'GPFSEL5',
  'GPSET0',
  'GPSET1',
  'GPCLR0',
  'GPCLR1',
  'GPLEV0',
  'GPLEV1',
  'GPEDS0',
  'GPEDS1',
  'GPREN0',
  'GPREN1',
  'GPFEN0',
  'GPFEN1',
  'GPHEN0',
  'GPHEN1',
  'GPLEN0',
  'GPLEN1',
  'GPAREN0',
  'GPAREN1',
  'GPAFEN0',
  'GPAFEN1',
  'GPPUD',
  'GPPUDCLK0',
  'GPPUDCLK1',

  'SPI0_CS',
  'SPI0_FIFO',
  'SPI0_CLK',
  'SPI0_DLEN',
  'SPI0_LTOH',
  'SPI0_DC',

  'SPI0_CS_LEN_LONG',
  'SPI0_CS_DMA_LEN',
  'SPI0_CS_CSPOL2',
  'SPI0_CS_CSPOL1',
  'SPI0_CS_CSPOL0',
  'SPI0_CS_RXF',
  'SPI0_CS_RXR',
  'SPI0_CS_TXD',
  'SPI0_CS_RXD',
  'SPI0_CS_DONE',
  'SPI0_CS_TE_EN',
  'SPI0_CS_LMONO',
  'SPI0_CS_LEN',
  'SPI0_CS_REN',
  'SPI0_CS_ADCS',
  'SPI0_CS_INTR',
  'SPI0_CS_INTD',
  'SPI0_CS_DMAEN',
  'SPI0_CS_TA',
  'SPI0_CS_CSPOL',
  'SPI0_CS_CLEAR',
  'SPI0_CS_CLEAR_RX',
  'SPI0_CS_CLEAR_TX',
  'SPI0_CS_CPOL',
  'SPI0_CS_CPHA',
  'SPI0_CS_CS',

  'BSC_C',
  'BSC_S',
  'BSC_DLEN',
  'BSC_A',
  'BSC_FIFO',
  'BSC_DIV',
  'BSC_DEL',
  'BSC_CLKT',
  'BSC_C_I2CEN',
  'BSC_C_INTR',
  'BSC_C_INTT',
  'BSC_C_INTD',
  'BSC_C_ST',
  'BSC_C_CLEAR_1',
  'BSC_C_CLEAR_2',
  'BSC_C_READ',
  'BSC_S_CLKT',
  'BSC_S_ERR',
  'BSC_S_RXF',
  'BSC_S_TXE',
  'BSC_S_RXD',
  'BSC_S_TXD',
  'BSC_S_RXR',
  'BSC_S_TXW',
  'BSC_S_DONE',
  'BSC_S_TA',

  'BSC_FIFO_SIZE',

  'ST_CS',
  'ST_CLO',
  'ST_CHI',
  'PWM_CONTROL',
  'PWM_STATUS',
  'PWM_DMAC',
  'PWM0_RANGE',
  'PWM0_DATA',
  'PWM_FIF1',
  'PWM1_RANGE',
  'PWM1_DATA',
  'PWMCLK_CNTL',
  'PWMCLK_DIV',
  'PWM_PASSWRD',
  'PWM1_MS_MODE',
  'PWM1_USEFIFO',
  'PWM1_REVPOLAR',
  'PWM1_OFFSTATE',
  'PWM1_REPEATFF',
  'PWM1_SERIAL',
  'PWM1_ENABLE',
  'PWM0_MS_MODE',
  'PWM_CLEAR_FIFO',
  'PWM0_USEFIFO',
  'PWM0_REVPOLAR',
  'PWM0_OFFSTATE',
  'PWM0_REPEATFF',
  'PWM0_SERIAL',
  'PWM0_ENABLE',

  'GPIO_FSEL_INPT',
  'GPIO_FSEL_OUTP',
  'GPIO_FSEL_ALT0',
  'GPIO_FSEL_ALT1',
  'GPIO_FSEL_ALT2',
  'GPIO_FSEL_ALT3',
  'GPIO_FSEL_ALT4',
  'GPIO_FSEL_ALT5',
  'GPIO_FSEL_MASK',

  'GPIO_PUD_OFF',
  'GPIO_PUD_DOWN',
  'GPIO_PUD_UP',

  'RPI_GPIO_P1_03',
  'RPI_GPIO_P1_05',
  'RPI_GPIO_P1_07',
  'RPI_GPIO_P1_08',
  'RPI_GPIO_P1_10',
  'RPI_GPIO_P1_11',
  'RPI_GPIO_P1_12',
  'RPI_GPIO_P1_13',
  'RPI_GPIO_P1_15',
  'RPI_GPIO_P1_16',
  'RPI_GPIO_P1_18',
  'RPI_GPIO_P1_19',
  'RPI_GPIO_P1_21',
  'RPI_GPIO_P1_22',
  'RPI_GPIO_P1_23',
  'RPI_GPIO_P1_24',
  'RPI_GPIO_P1_26',
  'RPI_V2_GPIO_P1_03',
  'RPI_V2_GPIO_P1_05',
  'RPI_V2_GPIO_P1_07',
  'RPI_V2_GPIO_P1_08',
  'RPI_V2_GPIO_P1_10',
  'RPI_V2_GPIO_P1_11',
  'RPI_V2_GPIO_P1_12',
  'RPI_V2_GPIO_P1_13',
  'RPI_V2_GPIO_P1_15',
  'RPI_V2_GPIO_P1_16',
  'RPI_V2_GPIO_P1_18',
  'RPI_V2_GPIO_P1_19',
  'RPI_V2_GPIO_P1_21',
  'RPI_V2_GPIO_P1_22',
  'RPI_V2_GPIO_P1_23',
  'RPI_V2_GPIO_P1_24',
  'RPI_V2_GPIO_P1_26',
  'RPI_V2_GPIO_P5_03',
  'RPI_V2_GPIO_P5_04',
  'RPI_V2_GPIO_P5_05',
  'RPI_V2_GPIO_P5_06',

  "SPI_BIT_ORDER_LSBFIRST",
  "SPI_BIT_ORDER_MSBFIRST",

  "SPI_MODE0",
  "SPI_MODE1",
  "SPI_MODE2",
  "SPI_MODE3",

  "SPI_CS0",
  "SPI_CS1",
  "SPI_CS2",
  "SPI_CS_NONE",

  "SPI_CLOCK_DIVIDER_65536",
  "SPI_CLOCK_DIVIDER_32768",
  "SPI_CLOCK_DIVIDER_16384",
  "SPI_CLOCK_DIVIDER_8192",
  "SPI_CLOCK_DIVIDER_4096",
  "SPI_CLOCK_DIVIDER_2048",
  "SPI_CLOCK_DIVIDER_1024",
  "SPI_CLOCK_DIVIDER_512",
  "SPI_CLOCK_DIVIDER_256",
  "SPI_CLOCK_DIVIDER_128",
  "SPI_CLOCK_DIVIDER_64",
  "SPI_CLOCK_DIVIDER_32",
  "SPI_CLOCK_DIVIDER_16",
  "SPI_CLOCK_DIVIDER_8",
  "SPI_CLOCK_DIVIDER_4",
  "SPI_CLOCK_DIVIDER_2",
  "SPI_CLOCK_DIVIDER_1",

  "I2C_CLOCK_DIVIDER_2500",
  "I2C_CLOCK_DIVIDER_626",
  "I2C_CLOCK_DIVIDER_150",
  "I2C_CLOCK_DIVIDER_148",

  "I2C_REASON_OK",
  "I2C_REASON_ERROR_NACK",
  "I2C_REASON_ERROR_CLKT",
  "I2C_REASON_ERROR_DATA",

  "PWM_CLOCK_DIVIDER_32768",
  "PWM_CLOCK_DIVIDER_16384",
  "PWM_CLOCK_DIVIDER_8192",
  "PWM_CLOCK_DIVIDER_4096",
  "PWM_CLOCK_DIVIDER_2048",
  "PWM_CLOCK_DIVIDER_1024",
  "PWM_CLOCK_DIVIDER_512",
  "PWM_CLOCK_DIVIDER_256",
  "PWM_CLOCK_DIVIDER_128",
  "PWM_CLOCK_DIVIDER_64",
  "PWM_CLOCK_DIVIDER_32",
  "PWM_CLOCK_DIVIDER_16",
  "PWM_CLOCK_DIVIDER_8",
  "PWM_CLOCK_DIVIDER_4",
  "PWM_CLOCK_DIVIDER_2",
  "PWM_CLOCK_DIVIDER_1",

  'init'
]
