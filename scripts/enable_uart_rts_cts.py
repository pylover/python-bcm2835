# -*- coding: utf-8 -*-
from bcm2835 import *
__author__ = 'vahid'



if __name__ == '__main__':
  if init():
    try:
      print "Setting Pin: J8-08 GPIO: %2d To ALT0" % RPI_B_PLUS_GPIO_J8_08
      gpio_fsel(RPI_B_PLUS_GPIO_J8_08, GPIO_FSEL_ALT0)

      print "Setting Pin: J8-10 GPIO: %2d To ALT0" % RPI_B_PLUS_GPIO_J8_10
      gpio_fsel(RPI_B_PLUS_GPIO_J8_10, GPIO_FSEL_ALT0)

      print "Setting Pin: J8-11 GPIO: %2d To ALT3" % RPI_B_PLUS_GPIO_J8_11
      gpio_fsel(RPI_B_PLUS_GPIO_J8_11, GPIO_FSEL_ALT3)

      print "Setting Pin: J8-36 GPIO: %2d To ALT3" % RPI_B_PLUS_GPIO_J8_36
      gpio_fsel(RPI_B_PLUS_GPIO_J8_36, GPIO_FSEL_ALT3)
    finally:
      close()
  else:
    print "initialization failed, Exiting"

