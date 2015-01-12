# -*- coding: utf-8 -*-
from libc.stdint cimport uintptr_t, uint32_t, uint8_t, uint64_t, uint16_t
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

  int c_BCM2835_SPI0_CS "BCM2835_SPI0_CS"
  int c_BCM2835_SPI0_FIFO "BCM2835_SPI0_FIFO"
  int c_BCM2835_SPI0_CLK "BCM2835_SPI0_CLK"
  int c_BCM2835_SPI0_DLEN "BCM2835_SPI0_DLEN"
  int c_BCM2835_SPI0_LTOH "BCM2835_SPI0_LTOH"
  int c_BCM2835_SPI0_DC "BCM2835_SPI0_DC"

  int c_BCM2835_SPI0_CS_LEN_LONG "BCM2835_SPI0_CS_LEN_LONG"  # Enable Long data word in Lossi mode if DMA_LEN is set
  int c_BCM2835_SPI0_CS_DMA_LEN "BCM2835_SPI0_CS_DMA_LEN"  # Enable DMA mode in Lossi mode
  int c_BCM2835_SPI0_CS_CSPOL2 "BCM2835_SPI0_CS_CSPOL2"  # Chip Select 2 Polarity
  int c_BCM2835_SPI0_CS_CSPOL1 "BCM2835_SPI0_CS_CSPOL1"  # Chip Select 1 Polarity
  int c_BCM2835_SPI0_CS_CSPOL0 "BCM2835_SPI0_CS_CSPOL0"  # Chip Select 0 Polarity
  int c_BCM2835_SPI0_CS_RXF "BCM2835_SPI0_CS_RXF"  # RXF - RX FIFO Full
  int c_BCM2835_SPI0_CS_RXR "BCM2835_SPI0_CS_RXR"  # RXR RX FIFO needs Reading ( full)
  int c_BCM2835_SPI0_CS_TXD "BCM2835_SPI0_CS_TXD"  # TXD TX FIFO can accept Data
  int c_BCM2835_SPI0_CS_RXD "BCM2835_SPI0_CS_RXD"  # RXD RX FIFO contains Data
  int c_BCM2835_SPI0_CS_DONE "BCM2835_SPI0_CS_DONE"  # Done transfer Done
  int c_BCM2835_SPI0_CS_TE_EN "BCM2835_SPI0_CS_TE_EN"  # Unused
  int c_BCM2835_SPI0_CS_LMONO "BCM2835_SPI0_CS_LMONO"  # Unused
  int c_BCM2835_SPI0_CS_LEN "BCM2835_SPI0_CS_LEN"  # LEN LoSSI enable
  int c_BCM2835_SPI0_CS_REN "BCM2835_SPI0_CS_REN"  # REN Read Enable
  int c_BCM2835_SPI0_CS_ADCS "BCM2835_SPI0_CS_ADCS"  # ADCS Automatically Deassert Chip Select
  int c_BCM2835_SPI0_CS_INTR "BCM2835_SPI0_CS_INTR"  # INTR Interrupt on RXR
  int c_BCM2835_SPI0_CS_INTD "BCM2835_SPI0_CS_INTD"  # INTD Interrupt on Done
  int c_BCM2835_SPI0_CS_DMAEN "BCM2835_SPI0_CS_DMAEN"  # DMAEN DMA Enable
  int c_BCM2835_SPI0_CS_TA "BCM2835_SPI0_CS_TA"  # Transfer Active
  int c_BCM2835_SPI0_CS_CSPOL "BCM2835_SPI0_CS_CSPOL"  # Chip Select Polarity
  int c_BCM2835_SPI0_CS_CLEAR "BCM2835_SPI0_CS_CLEAR"  # Clear FIFO Clear RX and TX
  int c_BCM2835_SPI0_CS_CLEAR_RX "BCM2835_SPI0_CS_CLEAR_RX"  # Clear FIFO Clear RX
  int c_BCM2835_SPI0_CS_CLEAR_TX "BCM2835_SPI0_CS_CLEAR_TX"  # Clear FIFO Clear TX
  int c_BCM2835_SPI0_CS_CPOL "BCM2835_SPI0_CS_CPOL"  # Clock Polarity
  int c_BCM2835_SPI0_CS_CPHA "BCM2835_SPI0_CS_CPHA"  # Clock Phase
  int c_BCM2835_SPI0_CS_CS "BCM2835_SPI0_CS_CS"  # Chip Select

  int c_BCM2835_BSC_C "BCM2835_BSC_C" # BSC Master Control
  int c_BCM2835_BSC_S "BCM2835_BSC_S" # BSC Master Status
  int c_BCM2835_BSC_DLEN "BCM2835_BSC_DLEN" # BSC Master Data Length
  int c_BCM2835_BSC_A "BCM2835_BSC_A" # BSC Master Slave Address
  int c_BCM2835_BSC_FIFO "BCM2835_BSC_FIFO" # BSC Master Data FIFO
  int c_BCM2835_BSC_DIV "BCM2835_BSC_DIV" # BSC Master Clock Divider
  int c_BCM2835_BSC_DEL "BCM2835_BSC_DEL" # BSC Master Data Delay
  int c_BCM2835_BSC_CLKT "BCM2835_BSC_CLKT" # BSC Master Clock Stretch Timeout

  int c_BCM2835_BSC_C_I2CEN "BCM2835_BSC_C_I2CEN" # I2C Enable, 0 = disabled, 1 = enabled
  int c_BCM2835_BSC_C_INTR "BCM2835_BSC_C_INTR" # Interrupt on RX
  int c_BCM2835_BSC_C_INTT "BCM2835_BSC_C_INTT" # Interrupt on TX
  int c_BCM2835_BSC_C_INTD "BCM2835_BSC_C_INTD" # Interrupt on DONE
  int c_BCM2835_BSC_C_ST "BCM2835_BSC_C_ST" # Start transfer, 1 = Start a new transfer
  int c_BCM2835_BSC_C_CLEAR_1 "BCM2835_BSC_C_CLEAR_1" # Clear FIFO Clear
  int c_BCM2835_BSC_C_CLEAR_2 "BCM2835_BSC_C_CLEAR_2" # Clear FIFO Clear
  int c_BCM2835_BSC_C_READ "BCM2835_BSC_C_READ" # Read transfer

  int c_BCM2835_BSC_S_CLKT "BCM2835_BSC_S_CLKT" # Clock stretch timeout
  int c_BCM2835_BSC_S_ERR "BCM2835_BSC_S_ERR" # ACK error
  int c_BCM2835_BSC_S_RXF "BCM2835_BSC_S_RXF" # RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
  int c_BCM2835_BSC_S_TXE "BCM2835_BSC_S_TXE" # TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
  int c_BCM2835_BSC_S_RXD "BCM2835_BSC_S_RXD" # RXD FIFO contains data
  int c_BCM2835_BSC_S_TXD "BCM2835_BSC_S_TXD" # TXD FIFO can accept data
  int c_BCM2835_BSC_S_RXR "BCM2835_BSC_S_RXR" # RXR FIFO needs reading (full)
  int c_BCM2835_BSC_S_TXW "BCM2835_BSC_S_TXW" # TXW FIFO needs writing (full)
  int c_BCM2835_BSC_S_DONE "BCM2835_BSC_S_DONE" # Transfer DONE
  int c_BCM2835_BSC_S_TA "BCM2835_BSC_S_TA" # Transfer Active

  int c_BCM2835_BSC_FIFO_SIZE "BCM2835_BSC_FIFO_SIZE" # BSC FIFO size

  int c_BCM2835_ST_CS "BCM2835_ST_CS" # System Timer Control/Status
  int c_BCM2835_ST_CLO "BCM2835_ST_CLO" # System Timer Counter Lower 32 bits
  int c_BCM2835_ST_CHI "BCM2835_ST_CHI" # System Timer Counter Upper 32 bits

  int c_BCM2835_PWM_CONTROL "BCM2835_PWM_CONTROL"
  int c_BCM2835_PWM_STATUS "BCM2835_PWM_STATUS"
  int c_BCM2835_PWM_DMAC "BCM2835_PWM_DMAC"
  int c_BCM2835_PWM0_RANGE "BCM2835_PWM0_RANGE"
  int c_BCM2835_PWM0_DATA "BCM2835_PWM0_DATA"
  int c_BCM2835_PWM_FIF1 "BCM2835_PWM_FIF1"
  int c_BCM2835_PWM1_RANGE "BCM2835_PWM1_RANGE"
  int c_BCM2835_PWM1_DATA "BCM2835_PWM1_DATA"

  int c_BCM2835_PWMCLK_CNTL "BCM2835_PWMCLK_CNTL"
  int c_BCM2835_PWMCLK_DIV "BCM2835_PWMCLK_DIV"
  int c_BCM2835_PWM_PASSWRD "BCM2835_PWM_PASSWRD"  # Password to enable setting PWM clock

  int c_BCM2835_PWM1_MS_MODE "BCM2835_PWM1_MS_MODE"  # Run in Mark/Space mode
  int c_BCM2835_PWM1_USEFIFO "BCM2835_PWM1_USEFIFO"  # Data from FIFO
  int c_BCM2835_PWM1_REVPOLAR "BCM2835_PWM1_REVPOLAR"  # Reverse polarity
  int c_BCM2835_PWM1_OFFSTATE "BCM2835_PWM1_OFFSTATE"  # Ouput Off state
  int c_BCM2835_PWM1_REPEATFF "BCM2835_PWM1_REPEATFF"  # Repeat last value if FIFO empty
  int c_BCM2835_PWM1_SERIAL "BCM2835_PWM1_SERIAL"  # Run in serial mode
  int c_BCM2835_PWM1_ENABLE "BCM2835_PWM1_ENABLE"  # Channel Enable

  int c_BCM2835_PWM0_MS_MODE "BCM2835_PWM0_MS_MODE"  # Run in Mark/Space mode
  int c_BCM2835_PWM_CLEAR_FIFO "BCM2835_PWM_CLEAR_FIFO"  # Clear FIFO
  int c_BCM2835_PWM0_USEFIFO "BCM2835_PWM0_USEFIFO"  # Data from FIFO
  int c_BCM2835_PWM0_REVPOLAR "BCM2835_PWM0_REVPOLAR"  # Reverse polarity
  int c_BCM2835_PWM0_OFFSTATE "BCM2835_PWM0_OFFSTATE"  # Ouput Off state
  int c_BCM2835_PWM0_REPEATFF "BCM2835_PWM0_REPEATFF"  # Repeat last value if FIFO empty
  int c_BCM2835_PWM0_SERIAL "BCM2835_PWM0_SERIAL"  # Run in serial mode
  int c_BCM2835_PWM0_ENABLE "BCM2835_PWM0_ENABLE"  # Channel Enable


  ctypedef enum c_bcm2835FunctionSelect "bcm2835FunctionSelect":
    c_BCM2835_GPIO_FSEL_INPT "BCM2835_GPIO_FSEL_INPT"  # Input
    c_BCM2835_GPIO_FSEL_OUTP "BCM2835_GPIO_FSEL_OUTP"  # Output
    c_BCM2835_GPIO_FSEL_ALT0 "BCM2835_GPIO_FSEL_ALT0"  # Alternate function 0
    c_BCM2835_GPIO_FSEL_ALT1 "BCM2835_GPIO_FSEL_ALT1"  # Alternate function 1
    c_BCM2835_GPIO_FSEL_ALT2 "BCM2835_GPIO_FSEL_ALT2"  # Alternate function 2
    c_BCM2835_GPIO_FSEL_ALT3 "BCM2835_GPIO_FSEL_ALT3"  # Alternate function 3
    c_BCM2835_GPIO_FSEL_ALT4 "BCM2835_GPIO_FSEL_ALT4"  # Alternate function 4
    c_BCM2835_GPIO_FSEL_ALT5 "BCM2835_GPIO_FSEL_ALT5"  # Alternate function 5
    c_BCM2835_GPIO_FSEL_MASK "BCM2835_GPIO_FSEL_MASK"  # Function select bits mask

  ctypedef enum c_bcm2835PUDControl "bcm2835PUDControl":
    c_BCM2835_GPIO_PUD_OFF "BCM2835_GPIO_PUD_OFF"  # Off ? disable pull-up/down
    c_BCM2835_GPIO_PUD_DOWN "BCM2835_GPIO_PUD_DOWN"  # Enable Pull Down control
    c_BCM2835_GPIO_PUD_UP "BCM2835_GPIO_PUD_UP"  # Enable Pull Up control

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

    # RPI B+
    c_RPI_B_PLUS_GPIO_J8_03 "RPI_B_PLUS_GPIO_J8_03"  # B+, Pin J8-03
    c_RPI_B_PLUS_GPIO_J8_05 "RPI_B_PLUS_GPIO_J8_05"  # B+, Pin J8-05
    c_RPI_B_PLUS_GPIO_J8_07 "RPI_B_PLUS_GPIO_J8_07"  # B+, Pin J8-07
    c_RPI_B_PLUS_GPIO_J8_08 "RPI_B_PLUS_GPIO_J8_08"  # B+, Pin J8-08, defaults to alt function 0 UART0_TXD
    c_RPI_B_PLUS_GPIO_J8_10 "RPI_B_PLUS_GPIO_J8_10"  # B+, Pin J8-10, defaults to alt function 0 UART0_RXD
    c_RPI_B_PLUS_GPIO_J8_11 "RPI_B_PLUS_GPIO_J8_11"  # B+, Pin J8-11
    c_RPI_B_PLUS_GPIO_J8_12 "RPI_B_PLUS_GPIO_J8_12"  # B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5
    c_RPI_B_PLUS_GPIO_J8_13 "RPI_B_PLUS_GPIO_J8_13"  # B+, Pin J8-13
    c_RPI_B_PLUS_GPIO_J8_15 "RPI_B_PLUS_GPIO_J8_15"  # B+, Pin J8-15
    c_RPI_B_PLUS_GPIO_J8_16 "RPI_B_PLUS_GPIO_J8_16"  # B+, Pin J8-16
    c_RPI_B_PLUS_GPIO_J8_18 "RPI_B_PLUS_GPIO_J8_18"  # B+, Pin J8-18
    c_RPI_B_PLUS_GPIO_J8_19 "RPI_B_PLUS_GPIO_J8_19"  # B+, Pin J8-19, MOSI when SPI0 in use
    c_RPI_B_PLUS_GPIO_J8_21 "RPI_B_PLUS_GPIO_J8_21"  # B+, Pin J8-21, MISO when SPI0 in use
    c_RPI_B_PLUS_GPIO_J8_22 "RPI_B_PLUS_GPIO_J8_22"  # B+, Pin J8-22
    c_RPI_B_PLUS_GPIO_J8_23 "RPI_B_PLUS_GPIO_J8_23"  # B+, Pin J8-23, CLK when SPI0 in use
    c_RPI_B_PLUS_GPIO_J8_24 "RPI_B_PLUS_GPIO_J8_24"  # B+, Pin J8-24, CE0 when SPI0 in use
    c_RPI_B_PLUS_GPIO_J8_26 "RPI_B_PLUS_GPIO_J8_26"  # B+, Pin J8-26, CE1 when SPI0 in use
    c_RPI_B_PLUS_GPIO_J8_29 "RPI_B_PLUS_GPIO_J8_29"  # B+, Pin J8-29
    c_RPI_B_PLUS_GPIO_J8_31 "RPI_B_PLUS_GPIO_J8_31"  # B+, Pin J8-31
    c_RPI_B_PLUS_GPIO_J8_32 "RPI_B_PLUS_GPIO_J8_32"  # B+, Pin J8-32
    c_RPI_B_PLUS_GPIO_J8_33 "RPI_B_PLUS_GPIO_J8_33"  # B+, Pin J8-33
    c_RPI_B_PLUS_GPIO_J8_35 "RPI_B_PLUS_GPIO_J8_35"  # B+, Pin J8-35
    c_RPI_B_PLUS_GPIO_J8_36 "RPI_B_PLUS_GPIO_J8_36"  # B+, Pin J8-36
    c_RPI_B_PLUS_GPIO_J8_37 "RPI_B_PLUS_GPIO_J8_37"  # B+, Pin J8-37
    c_RPI_B_PLUS_GPIO_J8_38 "RPI_B_PLUS_GPIO_J8_38"  # B+, Pin J8-38
    c_RPI_B_PLUS_GPIO_J8_40 "RPI_B_PLUS_GPIO_J8_40"  # B+, Pin J8-40

  ctypedef enum c_bcm2835SPIBitOrder "bcm2835SPIBitOrder":
    c_BCM2835_SPI_BIT_ORDER_LSBFIRST "BCM2835_SPI_BIT_ORDER_LSBFIRST"  #  LSB First
    c_BCM2835_SPI_BIT_ORDER_MSBFIRST "BCM2835_SPI_BIT_ORDER_MSBFIRST"  #  MSB First

  ctypedef enum c_bcm2835SPIMode "bcm2835SPIMode":
    c_BCM2835_SPI_MODE0 "BCM2835_SPI_MODE0"  #  CPOL = 0, CPHA = 0
    c_BCM2835_SPI_MODE1 "BCM2835_SPI_MODE1"  #  CPOL = 0, CPHA = 1
    c_BCM2835_SPI_MODE2 "BCM2835_SPI_MODE2"  #  CPOL = 1, CPHA = 0
    c_BCM2835_SPI_MODE3 "BCM2835_SPI_MODE3"  #  CPOL = 1, CPHA = 1

  ctypedef enum c_bcm2835SPIChipSelect "bcm2835SPIChipSelect":
    c_BCM2835_SPI_CS0 "BCM2835_SPI_CS0"  #  Chip Select 0
    c_BCM2835_SPI_CS1 "BCM2835_SPI_CS1"  #  Chip Select 1
    c_BCM2835_SPI_CS2 "BCM2835_SPI_CS2"  #  Chip Select 2 (ie pins CS1 and CS2 are asserted)
    c_BCM2835_SPI_CS_NONE "BCM2835_SPI_CS_NONE"  #  No CS, control it yourself

  ctypedef enum c_bcm2835SPIClockDivider "bcm2835SPIClockDivider":
    c_BCM2835_SPI_CLOCK_DIVIDER_65536 "BCM2835_SPI_CLOCK_DIVIDER_65536"  #  65536 = 262.144us = 3.814697260kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_32768 "BCM2835_SPI_CLOCK_DIVIDER_32768"  #  32768 = 131.072us = 7.629394531kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_16384 "BCM2835_SPI_CLOCK_DIVIDER_16384"  #  16384 = 65.536us = 15.25878906kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_8192 "BCM2835_SPI_CLOCK_DIVIDER_8192"  #  8192 = 32.768us = 30/51757813kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_4096 "BCM2835_SPI_CLOCK_DIVIDER_4096"  #  4096 = 16.384us = 61.03515625kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_2048 "BCM2835_SPI_CLOCK_DIVIDER_2048"  #  2048 = 8.192us = 122.0703125kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_1024 "BCM2835_SPI_CLOCK_DIVIDER_1024"  #  1024 = 4.096us = 244.140625kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_512 "BCM2835_SPI_CLOCK_DIVIDER_512"  #  512 = 2.048us = 488.28125kHz
    c_BCM2835_SPI_CLOCK_DIVIDER_256 "BCM2835_SPI_CLOCK_DIVIDER_256"  #  256 = 1.024us = 976.5625MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_128 "BCM2835_SPI_CLOCK_DIVIDER_128"  #  128 = 512ns = = 1.953125MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_64 "BCM2835_SPI_CLOCK_DIVIDER_64"  #  64 = 256ns = 3.90625MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_32 "BCM2835_SPI_CLOCK_DIVIDER_32"  #  32 = 128ns = 7.8125MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_16 "BCM2835_SPI_CLOCK_DIVIDER_16"  #  16 = 64ns = 15.625MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_8 "BCM2835_SPI_CLOCK_DIVIDER_8"  #  8 = 32ns = 31.25MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_4 "BCM2835_SPI_CLOCK_DIVIDER_4"  #  4 = 16ns = 62.5MHz
    c_BCM2835_SPI_CLOCK_DIVIDER_2 "BCM2835_SPI_CLOCK_DIVIDER_2"  #  2 = 8ns = 125MHz, fastest you can get
    c_BCM2835_SPI_CLOCK_DIVIDER_1 "BCM2835_SPI_CLOCK_DIVIDER_1"  #  1 = 262.144us = 3.814697260kHz, same as 0/65536

  ctypedef enum c_bcm2835I2CClockDivider "bcm2835I2CClockDivider":
    c_BCM2835_I2C_CLOCK_DIVIDER_2500 "BCM2835_I2C_CLOCK_DIVIDER_2500"  #  2500 = 10us = 100 kHz
    c_BCM2835_I2C_CLOCK_DIVIDER_626 "BCM2835_I2C_CLOCK_DIVIDER_626"  #  622 = 2.504us = 399.3610 kHz
    c_BCM2835_I2C_CLOCK_DIVIDER_150 "BCM2835_I2C_CLOCK_DIVIDER_150"  #  150 = 60ns = 1.666 MHz (default at reset)
    c_BCM2835_I2C_CLOCK_DIVIDER_148 "BCM2835_I2C_CLOCK_DIVIDER_148"  #  148 = 59ns = 1.689 MHz

  ctypedef enum c_bcm2835I2CReasonCodes "bcm2835I2CReasonCodes":
    c_BCM2835_I2C_REASON_OK "BCM2835_I2C_REASON_OK"  #  Success
    c_BCM2835_I2C_REASON_ERROR_NACK "BCM2835_I2C_REASON_ERROR_NACK"  #  Received a NACK
    c_BCM2835_I2C_REASON_ERROR_CLKT "BCM2835_I2C_REASON_ERROR_CLKT"  #  Received Clock Stretch Timeout
    c_BCM2835_I2C_REASON_ERROR_DATA "BCM2835_I2C_REASON_ERROR_DATA"  #  Not all data is sent / received

  ctypedef enum c_bcm2835PWMClockDivider "bcm2835PWMClockDivider":
    c_BCM2835_PWM_CLOCK_DIVIDER_32768 "BCM2835_PWM_CLOCK_DIVIDER_32768"  #  32768 = 585Hz
    c_BCM2835_PWM_CLOCK_DIVIDER_16384 "BCM2835_PWM_CLOCK_DIVIDER_16384"  #  16384 = 1171.8Hz
    c_BCM2835_PWM_CLOCK_DIVIDER_8192 "BCM2835_PWM_CLOCK_DIVIDER_8192"  #  8192 = 2.34375kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_4096 "BCM2835_PWM_CLOCK_DIVIDER_4096"  #  4096 = 4.6875kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_2048 "BCM2835_PWM_CLOCK_DIVIDER_2048"  #  2048 = 9.375kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_1024 "BCM2835_PWM_CLOCK_DIVIDER_1024"  #  1024 = 18.75kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_512 "BCM2835_PWM_CLOCK_DIVIDER_512"  #  512 = 37.5kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_256 "BCM2835_PWM_CLOCK_DIVIDER_256"  #  256 = 75kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_128 "BCM2835_PWM_CLOCK_DIVIDER_128"  #  128 = 150kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_64 "BCM2835_PWM_CLOCK_DIVIDER_64"  #  64 = 300kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_32 "BCM2835_PWM_CLOCK_DIVIDER_32"  #  32 = 600.0kHz
    c_BCM2835_PWM_CLOCK_DIVIDER_16 "BCM2835_PWM_CLOCK_DIVIDER_16"  #  16 = 1.2MHz
    c_BCM2835_PWM_CLOCK_DIVIDER_8 "BCM2835_PWM_CLOCK_DIVIDER_8"  #  8 = 2.4MHz
    c_BCM2835_PWM_CLOCK_DIVIDER_4 "BCM2835_PWM_CLOCK_DIVIDER_4"  #  4 = 4.8MHz
    c_BCM2835_PWM_CLOCK_DIVIDER_2 "BCM2835_PWM_CLOCK_DIVIDER_2"  #  2 = 9.6MHz, fastest you can get
    c_BCM2835_PWM_CLOCK_DIVIDER_1 "BCM2835_PWM_CLOCK_DIVIDER_1"  #  1 = 4.6875kHz, same as divider 4096


  # Variables
  extern uintptr_t c_bcm2835_gpio "bcm2835_gpio"

  # Functions
  extern int c_bcm2835_init "bcm2835_init" ()
  extern int c_bcm2835_close "bcm2835_close" ()
  extern void c_bcm2835_set_debug "bcm2835_set_debug" (uint8_t debug)
  extern uint32_t c_bcm2835_peri_read "bcm2835_peri_read" (uintptr_t paddr)
  extern uint32_t c_bcm2835_peri_read_nb "bcm2835_peri_read_nb" (uintptr_t paddr)
  extern void c_bcm2835_peri_write "bcm2835_peri_write" (uintptr_t paddr, uint32_t value)
  extern void c_bcm2835_peri_write_nb "bcm2835_peri_write_nb" (uintptr_t paddr, uint32_t value)
  extern void c_bcm2835_peri_set_bits "bcm2835_peri_set_bits" (uintptr_t paddr, uint32_t value, uint32_t mask)
  extern void c_bcm2835_gpio_fsel "bcm2835_gpio_fsel" (uint8_t pin, uint8_t mode)
  extern void c_bcm2835_gpio_set "bcm2835_gpio_set" (uint8_t pin)
  extern void c_bcm2835_gpio_clr "bcm2835_gpio_clr" (uint8_t pin)
  extern void c_bcm2835_gpio_set_multi "bcm2835_gpio_set_multi" (uint32_t mask)
  extern void c_bcm2835_gpio_clr_multi "bcm2835_gpio_clr_multi" (uint32_t mask)
  extern uint8_t c_bcm2835_gpio_lev "bcm2835_gpio_lev" (uint8_t pin)
  extern uint8_t c_bcm2835_gpio_eds "bcm2835_gpio_eds" (uint8_t pin)
  extern void c_bcm2835_gpio_set_eds "bcm2835_gpio_set_eds" (uint8_t pin)
  extern void c_bcm2835_gpio_ren "bcm2835_gpio_ren" (uint8_t pin)
  extern void c_bcm2835_gpio_clr_ren "bcm2835_gpio_clr_ren" (uint8_t pin)
  extern void c_bcm2835_gpio_fen "bcm2835_gpio_fen" (uint8_t pin)
  extern void c_bcm2835_gpio_clr_fen "bcm2835_gpio_clr_fen" (uint8_t pin)
  extern void c_bcm2835_gpio_hen "bcm2835_gpio_hen" (uint8_t pin)
  extern void c_bcm2835_gpio_clr_hen "bcm2835_gpio_clr_hen" (uint8_t pin)
  extern void c_bcm2835_gpio_len "bcm2835_gpio_len" (uint8_t pin)
  extern void c_bcm2835_gpio_clr_len "bcm2835_gpio_clr_len" (uint8_t pin)
  extern void c_bcm2835_gpio_aren "bcm2835_gpio_aren" (uint8_t pin)
  extern void c_bcm2835_gpio_clr_aren "bcm2835_gpio_clr_aren" (uint8_t pin)
  extern void c_bcm2835_gpio_afen "bcm2835_gpio_afen" (uint8_t pin)
  extern void c_bcm2835_gpio_clr_afen "bcm2835_gpio_clr_afen" (uint8_t pin)
  extern void c_bcm2835_gpio_pud "bcm2835_gpio_pud" (uint8_t pud)
  extern void c_bcm2835_gpio_pudclk "bcm2835_gpio_pudclk" (uint8_t pin, uint8_t on)
  extern uint32_t c_bcm2835_gpio_pad "bcm2835_gpio_pad" (uint8_t group)
  extern void c_bcm2835_gpio_set_pad "bcm2835_gpio_set_pad" (uint8_t group, uint32_t control)
  extern void c_bcm2835_delay "bcm2835_delay" (unsigned int millis)
  extern void c_bcm2835_delayMicroseconds "bcm2835_delayMicroseconds" (uint64_t micros)
  extern void c_bcm2835_gpio_write "bcm2835_gpio_write" (uint8_t pin, uint8_t on)
  extern void c_bcm2835_gpio_write_multi "bcm2835_gpio_write_multi" (uint32_t mask, uint8_t on)
  extern void c_bcm2835_gpio_write_mask "bcm2835_gpio_write_mask" (uint32_t value, uint32_t mask)
  extern void c_bcm2835_gpio_set_pud "bcm2835_gpio_set_pud" (uint8_t pin, uint8_t pud)
  extern void c_bcm2835_spi_begin "bcm2835_spi_begin" ()
  extern void c_bcm2835_spi_end "bcm2835_spi_end" ()
  extern void c_bcm2835_spi_setBitOrder "bcm2835_spi_setBitOrder" (uint8_t order)
  extern void c_bcm2835_spi_setClockDivider "bcm2835_spi_setClockDivider" (uint16_t divider)
  extern void c_bcm2835_spi_setDataMode "bcm2835_spi_setDataMode" (uint8_t mode)
  extern void c_bcm2835_spi_chipSelect "bcm2835_spi_chipSelect" (uint8_t cs)
  extern void c_bcm2835_spi_setChipSelectPolarity "bcm2835_spi_setChipSelectPolarity" (uint8_t cs, uint8_t active)
  extern uint8_t c_bcm2835_spi_transfer "bcm2835_spi_transfer" (uint8_t value)
  extern void c_bcm2835_spi_transfernb "bcm2835_spi_transfernb" (char* tbuf, char* rbuf, uint32_t len)
  extern void c_bcm2835_spi_transfern "bcm2835_spi_transfern" (char* buf, uint32_t len)
  extern void c_bcm2835_spi_writenb "bcm2835_spi_writenb" (char* buf, uint32_t len)
  extern void c_bcm2835_i2c_begin "bcm2835_i2c_begin" ()
  extern void c_bcm2835_i2c_end "bcm2835_i2c_end" ()
  extern void c_bcm2835_i2c_setSlaveAddress "bcm2835_i2c_setSlaveAddress" (uint8_t addr)
  extern void c_bcm2835_i2c_setClockDivider "bcm2835_i2c_setClockDivider" (uint16_t divider)
  extern void c_bcm2835_i2c_set_baudrate "bcm2835_i2c_set_baudrate" (uint32_t baudrate)
  extern uint8_t c_bcm2835_i2c_write "bcm2835_i2c_write" (const char * buf, uint32_t len)
  extern uint8_t c_bcm2835_i2c_read "bcm2835_i2c_read" (char* buf, uint32_t len)
  extern uint8_t c_bcm2835_i2c_read_register_rs "bcm2835_i2c_read_register_rs" (char* regaddr, char* buf, uint32_t len)
  extern uint64_t c_bcm2835_st_read "bcm2835_st_read" ()
  extern void c_bcm2835_st_delay "bcm2835_st_delay" (uint64_t offset_micros, uint64_t micros)
  extern void c_bcm2835_pwm_set_clock "bcm2835_pwm_set_clock" (uint32_t divisor)
  extern void c_bcm2835_pwm_set_mode "bcm2835_pwm_set_mode" (uint8_t channel, uint8_t markspace, uint8_t enabled)
  extern void c_bcm2835_pwm_set_range "bcm2835_pwm_set_range" (uint8_t channel, uint32_t range)
  extern void c_bcm2835_pwm_set_data "bcm2835_pwm_set_data" (uint8_t channel, uint32_t data)
