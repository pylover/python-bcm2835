__author__ = 'vahid'

import unittest
from bcm2835 import *


class TestBCM2835Extension(unittest.TestCase):

  def test_constants(self):
    self.assertEqual(LOW, 0x0)
    self.assertEqual(HIGH, 0x1)

    self.assertEqual(CORE_CLK_HZ, 250000000)
    self.assertEqual(PERI_BASE, 0x20000000)
    self.assertEqual(ST_BASE, PERI_BASE + 0x3000)
    self.assertEqual(GPIO_PADS, PERI_BASE + 0x100000)
    self.assertEqual(CLOCK_BASE, PERI_BASE + 0x101000)
    self.assertEqual(GPIO_BASE, PERI_BASE + 0x200000)
    self.assertEqual(SPI0_BASE, PERI_BASE + 0x204000)
    self.assertEqual(BSC0_BASE, PERI_BASE + 0x205000)
    self.assertEqual(GPIO_PWM, PERI_BASE + 0x20C000)
    self.assertEqual(BSC1_BASE, PERI_BASE + 0x804000)
    self.assertEqual(PAGE_SIZE, 4*1024)
    self.assertEqual(BLOCK_SIZE, 4*1024)

    self.assertEqual(GPFSEL0, 0x0000)  # GPIO Function Select, 0
    self.assertEqual(GPFSEL1, 0x0004)  # GPIO Function Select 1
    self.assertEqual(GPFSEL2, 0x0008)  # GPIO Function Select 2
    self.assertEqual(GPFSEL3, 0x000c)  # GPIO Function Select 3
    self.assertEqual(GPFSEL4, 0x0010)  # GPIO Function Select 4
    self.assertEqual(GPFSEL5, 0x0014)  # GPIO Function Select 5
    self.assertEqual(GPSET0, 0x001c)  # GPIO Pin Output Set, 0
    self.assertEqual(GPSET1, 0x0020)  # GPIO Pin Output Set 1
    self.assertEqual(GPCLR0, 0x0028)  # GPIO Pin Output Clear, 0
    self.assertEqual(GPCLR1, 0x002c)  # GPIO Pin Output Clear 1
    self.assertEqual(GPLEV0, 0x0034)  # GPIO Pin Level, 0
    self.assertEqual(GPLEV1, 0x0038)  # GPIO Pin Level 1
    self.assertEqual(GPEDS0, 0x0040)  # GPIO Pin Event Detect Status, 0
    self.assertEqual(GPEDS1, 0x0044)  # GPIO Pin Event Detect Status 1
    self.assertEqual(GPREN0, 0x004c)  # GPIO Pin Rising Edge Detect Enable, 0
    self.assertEqual(GPREN1, 0x0050)  # GPIO Pin Rising Edge Detect Enable 1
    self.assertEqual(GPFEN0, 0x0058)  # GPIO Pin Falling Edge Detect Enable, 0
    self.assertEqual(GPFEN1, 0x005c)  # GPIO Pin Falling Edge Detect Enable 1
    self.assertEqual(GPHEN0, 0x0064)  # GPIO Pin High Detect Enable, 0
    self.assertEqual(GPHEN1, 0x0068)  # GPIO Pin High Detect Enable 1
    self.assertEqual(GPLEN0, 0x0070)  # GPIO Pin Low Detect Enable, 0
    self.assertEqual(GPLEN1, 0x0074)  # GPIO Pin Low Detect Enable 1
    self.assertEqual(GPAREN0, 0x007c)  # GPIO Pin Async. Rising Edge Detect, 0
    self.assertEqual(GPAREN1, 0x0080)  # GPIO Pin Async. Rising Edge Detect 1
    self.assertEqual(GPAFEN0, 0x0088)  # GPIO Pin Async. Falling Edge Detect, 0
    self.assertEqual(GPAFEN1, 0x008c)  # GPIO Pin Async. Falling Edge Detect 1
    self.assertEqual(GPPUD, 0x0094)  # GPIO Pin Pull-up/down Enable
    self.assertEqual(GPPUDCLK0, 0x0098)  # GPIO Pin Pull-up/down Enable Clock, 0
    self.assertEqual(GPPUDCLK1, 0x009c)  # GPIO Pin Pull-up/down Enable Clock 1

    self.assertEqual(GPIO_FSEL_INPT, 0b000)
    self.assertEqual(GPIO_FSEL_OUTP, 0b001)
    self.assertEqual(GPIO_FSEL_ALT0, 0b100)
    self.assertEqual(GPIO_FSEL_ALT1, 0b101)
    self.assertEqual(GPIO_FSEL_ALT2, 0b110)
    self.assertEqual(GPIO_FSEL_ALT3, 0b111)
    self.assertEqual(GPIO_FSEL_ALT4, 0b011)
    self.assertEqual(GPIO_FSEL_ALT5, 0b010)
    self.assertEqual(GPIO_FSEL_MASK, 0b111)

    self.assertEqual(GPIO_PUD_OFF, 0b00)
    self.assertEqual(GPIO_PUD_DOWN, 0b01)
    self.assertEqual(GPIO_PUD_UP, 0b10)

if __name__ == '__main__':
  unittest.main()
