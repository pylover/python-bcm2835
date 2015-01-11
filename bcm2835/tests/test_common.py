__author__ = 'vahid'

import unittest
from bcm2835.common import *


class TestCommon(unittest.TestCase):

  def test_constants(self):
    self.assertEqual(LOW, 0x0)
    self.assertEqual(HIGH, 0x1)


if __name__ == '__main__':
  unittest.main()
