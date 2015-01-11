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

  'init'
]
