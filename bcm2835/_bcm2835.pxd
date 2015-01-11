# -*- coding: utf-8 -*-
__author__ = 'vahid'

cdef extern from "_bcm2835.h" nogil:
  int c_HIGH "HIGH"
  int c_LOW "LOW"
  int c_BCM2835_CORE_CLK_HZ "BCM2835_CORE_CLK_HZ"
  int c_BCM2835_PERI_BASE "BCM2835_PERI_BASE"
  int c_BCM2835_ST_BASE "BCM2835_ST_BASE"
  int c_BCM2835_GPIO_PADS "BCM2835_GPIO_PADS"
  int c_BCM2835_CLOCK_BASE "BCM2835_CLOCK_BASE"
  int c_BCM2835_GPIO_BASE "BCM2835_GPIO_BASE"
  int c_BCM2835_SPI0_BASE "BCM2835_SPI0_BASE"
  int c_BCM2835_BSC0_BASE "BCM2835_BSC0_BASE"
  int c_BCM2835_GPIO_PWM "BCM2835_GPIO_PWM"
  int c_BCM2835_BSC1_BASE "BCM2835_BSC1_BASE"
  int c_BCM2835_PAGE_SIZE "BCM2835_PAGE_SIZE"
  int c_BCM2835_BLOCK_SIZE "BCM2835_BLOCK_SIZE"
  int c_bcm2835_init "bcm2835_init"()