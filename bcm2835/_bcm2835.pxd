# -*- coding: utf-8 -*-
__author__ = 'vahid'

cdef extern from "_bcm2835.h" nogil:
  int c_HIGH                  "HIGH"
  int c_LOW                   "LOW"
  int c_BCM2835_CORE_CLK_HZ   "BCM2835_CORE_CLK_HZ"
  int c_BCM2835_PERI_BASE     "BCM2835_PERI_BASE"
  int c_BCM2835_ST_BASE       "BCM2835_ST_BASE"
  int c_BCM2835_GPIO_PADS     "BCM2835_GPIO_PADS"
  int c_BCM2835_CLOCK_BASE    "BCM2835_CLOCK_BASE"
  int c_BCM2835_GPIO_BASE     "BCM2835_GPIO_BASE"
  int c_BCM2835_SPI0_BASE     "BCM2835_SPI0_BASE"
  int c_BCM2835_BSC0_BASE     "BCM2835_BSC0_BASE"
  int c_BCM2835_GPIO_PWM      "BCM2835_GPIO_PWM"
  int c_BCM2835_BSC1_BASE     "BCM2835_BSC1_BASE"
  int c_BCM2835_PAGE_SIZE     "BCM2835_PAGE_SIZE"
  int c_BCM2835_BLOCK_SIZE    "BCM2835_BLOCK_SIZE"
  int c_BCM2835_GPFSEL0       "BCM2835_GPFSEL0"
  int c_BCM2835_GPFSEL1       "BCM2835_GPFSEL1"
  int c_BCM2835_GPFSEL2       "BCM2835_GPFSEL2"
  int c_BCM2835_GPFSEL3       "BCM2835_GPFSEL3"
  int c_BCM2835_GPFSEL4       "BCM2835_GPFSEL4"
  int c_BCM2835_GPFSEL5       "BCM2835_GPFSEL5"
  int c_BCM2835_GPSET0        "BCM2835_GPSET0"
  int c_BCM2835_GPSET1        "BCM2835_GPSET1"
  int c_BCM2835_GPCLR0        "BCM2835_GPCLR0"
  int c_BCM2835_GPCLR1        "BCM2835_GPCLR1"
  int c_BCM2835_GPLEV0        "BCM2835_GPLEV0"
  int c_BCM2835_GPLEV1        "BCM2835_GPLEV1"
  int c_BCM2835_GPEDS0        "BCM2835_GPEDS0"
  int c_BCM2835_GPEDS1        "BCM2835_GPEDS1"
  int c_BCM2835_GPREN0        "BCM2835_GPREN0"
  int c_BCM2835_GPREN1        "BCM2835_GPREN1"
  int c_BCM2835_GPFEN0        "BCM2835_GPFEN0"
  int c_BCM2835_GPFEN1        "BCM2835_GPFEN1"
  int c_BCM2835_GPHEN0        "BCM2835_GPHEN0"
  int c_BCM2835_GPHEN1        "BCM2835_GPHEN1"
  int c_BCM2835_GPLEN0        "BCM2835_GPLEN0"
  int c_BCM2835_GPLEN1        "BCM2835_GPLEN1"
  int c_BCM2835_GPAREN0       "BCM2835_GPAREN0"
  int c_BCM2835_GPAREN1       "BCM2835_GPAREN1"
  int c_BCM2835_GPAFEN0       "BCM2835_GPAFEN0"
  int c_BCM2835_GPAFEN1       "BCM2835_GPAFEN1"
  int c_BCM2835_GPPUD         "BCM2835_GPPUD"
  int c_BCM2835_GPPUDCLK0     "BCM2835_GPPUDCLK0"
  int c_BCM2835_GPPUDCLK1     "BCM2835_GPPUDCLK1"

  int c_bcm2835_init          "bcm2835_init"()
