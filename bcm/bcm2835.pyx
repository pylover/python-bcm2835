# -*- coding: utf-8 -*-
__author__ = 'vahid'

from _bcm2835 cimport c_HIGH, c_LOW, c_BCM2835_CORE_CLK_HZ, c_BCM2835_PERI_BASE, \
  c_BCM2835_ST_BASE, c_BCM2835_CLOCK_BASE, c_BCM2835_SPI0_BASE, c_BCM2835_BSC0_BASE, \
  c_BCM2835_BSC1_BASE, c_BCM2835_PAGE_SIZE, c_BCM2835_BLOCK_SIZE, \
  c_BCM2835_GPIO_PADS, c_BCM2835_GPIO_BASE, c_BCM2835_GPIO_PWM, \
  c_bcm2835_init


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

  'init'
]
