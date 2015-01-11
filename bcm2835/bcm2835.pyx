# -*- coding: utf-8 -*-
__author__ = 'vahid'

from _bcm2835 cimport *


"""
This means pin HIGH, true, 3.3volts on a pin.
"""
HIGH = c_HIGH

"""
This means pin LOW, false, 0volts on a pin.
"""
LOW = c_LOW

"""
Speed of the core clock core_clk
"""
CORE_CLK_HZ = c_BCM2835_CORE_CLK_HZ

"""
Physical addresses for various peripheral register sets
Base Physical Address of the BCM 2835 peripheral registers
"""
PERI_BASE = c_BCM2835_PERI_BASE

"""
Base Physical Address of the System Timer registers
"""
ST_BASE = c_BCM2835_ST_BASE

"""
Base Physical Address of the Clock/timer registers
"""
CLOCK_BASE = c_BCM2835_CLOCK_BASE

"""
Base Physical Address of the SPI0 registers
"""
SPI0_BASE = c_BCM2835_SPI0_BASE

"""
Base Physical Address of the BSC0 registers
"""
BSC0_BASE = c_BCM2835_BSC0_BASE

"""
Base Physical Address of the BSC1 registers
"""
BSC1_BASE = c_BCM2835_BSC1_BASE

"""
Size of memory page on RPi
"""
PAGE_SIZE = c_BCM2835_PAGE_SIZE

"""
Size of memory block on RPi
"""
BLOCK_SIZE = c_BCM2835_BLOCK_SIZE

"""
Base Physical Address of the GPIO registers
"""
GPIO_BASE = c_BCM2835_GPIO_BASE

"""
Base Physical Address of the Pads registers
"""
GPIO_PADS = c_BCM2835_GPIO_PADS

"""
Base Physical Address of the PWM registers
"""
GPIO_PWM = c_BCM2835_GPIO_PWM

"""
GPIO Function Select 0
"""
GPFSEL0 = c_BCM2835_GPFSEL0

"""
GPIO Function Select 1
"""
GPFSEL1 = c_BCM2835_GPFSEL1

"""
GPIO Function Select 2
"""
GPFSEL2 = c_BCM2835_GPFSEL2

"""
GPIO Function Select 3
"""
GPFSEL3 = c_BCM2835_GPFSEL3

"""
GPIO Function Select 4
"""
GPFSEL4 = c_BCM2835_GPFSEL4

"""
GPIO Function Select 5
"""
GPFSEL5 = c_BCM2835_GPFSEL5

"""
GPIO Pin Output Set 0
"""
GPSET0 = c_BCM2835_GPSET0

"""
GPIO Pin Output Set 1
"""
GPSET1 = c_BCM2835_GPSET1

"""
GPIO Pin Output Clear 0
"""
GPCLR0 = c_BCM2835_GPCLR0

"""
GPIO Pin Output Clear 1
"""
GPCLR1 = c_BCM2835_GPCLR1

"""
GPIO Pin Level 0
"""
GPLEV0 = c_BCM2835_GPLEV0

"""
GPIO Pin Level 1
"""
GPLEV1 = c_BCM2835_GPLEV1

"""
GPIO Pin Event Detect Status 0
"""
GPEDS0 = c_BCM2835_GPEDS0

"""
GPIO Pin Event Detect Status 1
"""
GPEDS1 = c_BCM2835_GPEDS1

"""
GPIO Pin Rising Edge Detect Enable 0
"""
GPREN0 = c_BCM2835_GPREN0

"""
GPIO Pin Rising Edge Detect Enable 1
"""
GPREN1 = c_BCM2835_GPREN1

"""
GPIO Pin Falling Edge Detect Enable 0
"""
GPFEN0 = c_BCM2835_GPFEN0

"""
GPIO Pin Falling Edge Detect Enable 1
"""
GPFEN1 = c_BCM2835_GPFEN1

"""
GPIO Pin High Detect Enable 0
"""
GPHEN0 = c_BCM2835_GPHEN0

"""
GPIO Pin High Detect Enable 1
"""
GPHEN1 = c_BCM2835_GPHEN1

"""
GPIO Pin Low Detect Enable 0
"""
GPLEN0 = c_BCM2835_GPLEN0

"""
GPIO Pin Low Detect Enable 1
"""
GPLEN1 = c_BCM2835_GPLEN1

"""
GPIO Pin Async. Rising Edge Detect 0
"""
GPAREN0 = c_BCM2835_GPAREN0

"""
GPIO Pin Async. Rising Edge Detect 1
"""
GPAREN1 = c_BCM2835_GPAREN1

"""
GPIO Pin Async. Falling Edge Detect 0
"""
GPAFEN0 = c_BCM2835_GPAFEN0

"""
GPIO Pin Async. Falling Edge Detect 1
"""
GPAFEN1 = c_BCM2835_GPAFEN1

"""
GPIO Pin Pull-up/down Enable
"""
GPPUD = c_BCM2835_GPPUD

"""
GPIO Pin Pull-up/down Enable Clock 0
"""
GPPUDCLK0 = c_BCM2835_GPPUDCLK0

"""
GPIO Pin Pull-up/down Enable Clock 1
"""
GPPUDCLK1 = c_BCM2835_GPPUDCLK1


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
  "GPFSEL0",
  "GPFSEL1",
  "GPFSEL2",
  "GPFSEL3",
  "GPFSEL4",
  "GPFSEL5",
  "GPSET0",
  "GPSET1",
  "GPCLR0",
  "GPCLR1",
  "GPLEV0",
  "GPLEV1",
  "GPEDS0",
  "GPEDS1",
  "GPREN0",
  "GPREN1",
  "GPFEN0",
  "GPFEN1",
  "GPHEN0",
  "GPHEN1",
  "GPLEN0",
  "GPLEN1",
  "GPAREN0",
  "GPAREN1",
  "GPAFEN0",
  "GPAFEN1",
  "GPPUD",
  "GPPUDCLK0",
  "GPPUDCLK1",

  'init'
]
