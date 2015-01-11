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

  ctypedef enum c_bcm2835FunctionSelect "bcm2835FunctionSelect":
    c_BCM2835_GPIO_FSEL_INPT "BCM2835_GPIO_FSEL_INPT"
    c_BCM2835_GPIO_FSEL_OUTP "BCM2835_GPIO_FSEL_OUTP"
    c_BCM2835_GPIO_FSEL_ALT0 "BCM2835_GPIO_FSEL_ALT0"
    c_BCM2835_GPIO_FSEL_ALT1 "BCM2835_GPIO_FSEL_ALT1"
    c_BCM2835_GPIO_FSEL_ALT2 "BCM2835_GPIO_FSEL_ALT2"
    c_BCM2835_GPIO_FSEL_ALT3 "BCM2835_GPIO_FSEL_ALT3"
    c_BCM2835_GPIO_FSEL_ALT4 "BCM2835_GPIO_FSEL_ALT4"
    c_BCM2835_GPIO_FSEL_ALT5 "BCM2835_GPIO_FSEL_ALT5"
    c_BCM2835_GPIO_FSEL_MASK "BCM2835_GPIO_FSEL_MASK"

  ctypedef enum c_bcm2835PUDControl "bcm2835PUDControl":
    c_BCM2835_GPIO_PUD_OFF "BCM2835_GPIO_PUD_OFF"
    c_BCM2835_GPIO_PUD_DOWN "BCM2835_GPIO_PUD_DOWN"
    c_BCM2835_GPIO_PUD_UP "BCM2835_GPIO_PUD_UP"


  ctypedef enum c_RPiGPIOPin "RPiGPIOPin":
    # RPi Version 1
    c_RPI_GPIO_P1_03 "RPI_GPIO_P1_03"  # Version 1, Pin P1-03
    c_RPI_GPIO_P1_05 "RPI_GPIO_P1_05"  # Version 1, Pin P1-05
    c_RPI_GPIO_P1_07 "RPI_GPIO_P1_07"  # Version 1, Pin P1-07
    c_RPI_GPIO_P1_08 "RPI_GPIO_P1_08"  # Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD
    c_RPI_GPIO_P1_10 "RPI_GPIO_P1_10"  # Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD
    c_RPI_GPIO_P1_11 "RPI_GPIO_P1_11"  # Version 1, Pin P1-11
    c_RPI_GPIO_P1_12 "RPI_GPIO_P1_12"  # Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5
    c_RPI_GPIO_P1_13 "RPI_GPIO_P1_13"  # Version 1, Pin P1-13
    c_RPI_GPIO_P1_15 "RPI_GPIO_P1_15"  # Version 1, Pin P1-15
    c_RPI_GPIO_P1_16 "RPI_GPIO_P1_16"  # Version 1, Pin P1-16
    c_RPI_GPIO_P1_18 "RPI_GPIO_P1_18"  # Version 1, Pin P1-18
    c_RPI_GPIO_P1_19 "RPI_GPIO_P1_19"  # Version 1, Pin P1-19, MOSI when SPI0 in use
    c_RPI_GPIO_P1_21 "RPI_GPIO_P1_21"  # Version 1, Pin P1-21, MISO when SPI0 in use
    c_RPI_GPIO_P1_22 "RPI_GPIO_P1_22"  # Version 1, Pin P1-22
    c_RPI_GPIO_P1_23 "RPI_GPIO_P1_23"  # Version 1, Pin P1-23, CLK when SPI0 in use
    c_RPI_GPIO_P1_24 "RPI_GPIO_P1_24"  # Version 1, Pin P1-24, CE0 when SPI0 in use
    c_RPI_GPIO_P1_26 "RPI_GPIO_P1_26"  # Version 1, Pin P1-26, CE1 when SPI0 in use

    # RPi Version 2
    c_RPI_V2_GPIO_P1_03 "RPI_V2_GPIO_P1_03"  # Version 2, Pin P1-03
    c_RPI_V2_GPIO_P1_05 "RPI_V2_GPIO_P1_05"  # Version 2, Pin P1-05
    c_RPI_V2_GPIO_P1_07 "RPI_V2_GPIO_P1_07"  # Version 2, Pin P1-07
    c_RPI_V2_GPIO_P1_08 "RPI_V2_GPIO_P1_08"  # Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD
    c_RPI_V2_GPIO_P1_10 "RPI_V2_GPIO_P1_10"  # Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD
    c_RPI_V2_GPIO_P1_11 "RPI_V2_GPIO_P1_11"  # Version 2, Pin P1-11
    c_RPI_V2_GPIO_P1_12 "RPI_V2_GPIO_P1_12"  # Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5
    c_RPI_V2_GPIO_P1_13 "RPI_V2_GPIO_P1_13"  # Version 2, Pin P1-13
    c_RPI_V2_GPIO_P1_15 "RPI_V2_GPIO_P1_15"  # Version 2, Pin P1-15
    c_RPI_V2_GPIO_P1_16 "RPI_V2_GPIO_P1_16"  # Version 2, Pin P1-16
    c_RPI_V2_GPIO_P1_18 "RPI_V2_GPIO_P1_18"  # Version 2, Pin P1-18
    c_RPI_V2_GPIO_P1_19 "RPI_V2_GPIO_P1_19"  # Version 2, Pin P1-19, MOSI when SPI0 in use
    c_RPI_V2_GPIO_P1_21 "RPI_V2_GPIO_P1_21"  # Version 2, Pin P1-21, MISO when SPI0 in use
    c_RPI_V2_GPIO_P1_22 "RPI_V2_GPIO_P1_22"  # Version 2, Pin P1-22
    c_RPI_V2_GPIO_P1_23 "RPI_V2_GPIO_P1_23"  # Version 2, Pin P1-23, CLK when SPI0 in use
    c_RPI_V2_GPIO_P1_24 "RPI_V2_GPIO_P1_24"  # Version 2, Pin P1-24, CE0 when SPI0 in use
    c_RPI_V2_GPIO_P1_26 "RPI_V2_GPIO_P1_26"  # Version 2, Pin P1-26, CE1 when SPI0 in use
    c_RPI_V2_GPIO_P5_03 "RPI_V2_GPIO_P5_03"  # Version 2, Pin P5-03
    c_RPI_V2_GPIO_P5_04 "RPI_V2_GPIO_P5_04"  # Version 2, Pin P5-04
    c_RPI_V2_GPIO_P5_05 "RPI_V2_GPIO_P5_05"  # Version 2, Pin P5-05
    c_RPI_V2_GPIO_P5_06 "RPI_V2_GPIO_P5_06"  # Version 2, Pin P5-06


  int c_bcm2835_init          "bcm2835_init"()
