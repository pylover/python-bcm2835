__author__ = 'vahid'

import unittest
from bcm2835 import *


class TestCommon(unittest.TestCase):

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

if __name__ == '__main__':
  unittest.main()
