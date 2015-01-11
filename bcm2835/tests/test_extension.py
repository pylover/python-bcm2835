__author__ = 'vahid'

import unittest
from bcm2835 import *


class TestBCM2835Extension(unittest.TestCase):

  def test_constants(self):

    ## Constants
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

    self.assertEqual(SPI0_CS, 0x0000)
    self.assertEqual(SPI0_FIFO, 0x0004)
    self.assertEqual(SPI0_CLK, 0x0008)
    self.assertEqual(SPI0_DLEN, 0x000c)
    self.assertEqual(SPI0_LTOH, 0x0010)
    self.assertEqual(SPI0_DC, 0x0014)

    self.assertEqual(SPI0_CS_LEN_LONG, 0x02000000)
    self.assertEqual(SPI0_CS_DMA_LEN, 0x01000000)
    self.assertEqual(SPI0_CS_CSPOL2, 0x00800000)
    self.assertEqual(SPI0_CS_CSPOL1, 0x00400000)
    self.assertEqual(SPI0_CS_CSPOL0, 0x00200000)
    self.assertEqual(SPI0_CS_RXF, 0x00100000)
    self.assertEqual(SPI0_CS_RXR, 0x00080000)
    self.assertEqual(SPI0_CS_TXD, 0x00040000)
    self.assertEqual(SPI0_CS_RXD, 0x00020000)
    self.assertEqual(SPI0_CS_DONE, 0x00010000)
    self.assertEqual(SPI0_CS_TE_EN, 0x00008000)
    self.assertEqual(SPI0_CS_LMONO, 0x00004000)
    self.assertEqual(SPI0_CS_LEN, 0x00002000)
    self.assertEqual(SPI0_CS_REN, 0x00001000)
    self.assertEqual(SPI0_CS_ADCS, 0x00000800)
    self.assertEqual(SPI0_CS_INTR, 0x00000400)
    self.assertEqual(SPI0_CS_INTD, 0x00000200)
    self.assertEqual(SPI0_CS_DMAEN, 0x00000100)
    self.assertEqual(SPI0_CS_TA, 0x00000080)
    self.assertEqual(SPI0_CS_CSPOL, 0x00000040)
    self.assertEqual(SPI0_CS_CLEAR, 0x00000030)
    self.assertEqual(SPI0_CS_CLEAR_RX, 0x00000020)
    self.assertEqual(SPI0_CS_CLEAR_TX, 0x00000010)
    self.assertEqual(SPI0_CS_CPOL, 0x00000008)
    self.assertEqual(SPI0_CS_CPHA, 0x00000004)
    self.assertEqual(SPI0_CS_CS, 0x00000003)

    self.assertEqual(BSC_C, 0x0000)
    self.assertEqual(BSC_S, 0x0004)
    self.assertEqual(BSC_DLEN, 0x0008)
    self.assertEqual(BSC_A, 0x000c)
    self.assertEqual(BSC_FIFO, 0x0010)
    self.assertEqual(BSC_DIV, 0x0014)
    self.assertEqual(BSC_DEL, 0x0018)
    self.assertEqual(BSC_CLKT, 0x001c)

    self.assertEqual(BSC_C_I2CEN, 0x00008000)
    self.assertEqual(BSC_C_INTR, 0x00000400)
    self.assertEqual(BSC_C_INTT, 0x00000200)
    self.assertEqual(BSC_C_INTD, 0x00000100)
    self.assertEqual(BSC_C_ST, 0x00000080)
    self.assertEqual(BSC_C_CLEAR_1, 0x00000020)
    self.assertEqual(BSC_C_CLEAR_2, 0x00000010)
    self.assertEqual(BSC_C_READ, 0x00000001)

    self.assertEqual(BSC_S_CLKT, 0x00000200)
    self.assertEqual(BSC_S_ERR, 0x00000100)
    self.assertEqual(BSC_S_RXF, 0x00000080)
    self.assertEqual(BSC_S_TXE, 0x00000040)
    self.assertEqual(BSC_S_RXD, 0x00000020)
    self.assertEqual(BSC_S_TXD, 0x00000010)
    self.assertEqual(BSC_S_RXR, 0x00000008)
    self.assertEqual(BSC_S_TXW, 0x00000004)
    self.assertEqual(BSC_S_DONE, 0x00000002)
    self.assertEqual(BSC_S_TA, 0x00000001)
    self.assertEqual(BSC_FIFO_SIZE, 16)

    self.assertEqual(ST_CS, 0x0000)
    self.assertEqual(ST_CLO, 0x0004)
    self.assertEqual(ST_CHI, 0x0008)

    self.assertEqual(PWM_CONTROL, 0)
    self.assertEqual(PWM_STATUS, 1)
    self.assertEqual(PWM_DMAC, 2)
    self.assertEqual(PWM0_RANGE, 4)
    self.assertEqual(PWM0_DATA, 5)
    self.assertEqual(PWM_FIF1, 6)
    self.assertEqual(PWM1_RANGE, 8)
    self.assertEqual(PWM1_DATA, 9)

    self.assertEqual(PWMCLK_CNTL, 40)
    self.assertEqual(PWMCLK_DIV, 41)
    self.assertEqual(PWM_PASSWRD, 0x5A << 24 )
    self.assertEqual(PWM1_MS_MODE, 0x8000)
    self.assertEqual(PWM1_USEFIFO, 0x2000)
    self.assertEqual(PWM1_REVPOLAR, 0x1000)
    self.assertEqual(PWM1_OFFSTATE, 0x0800)
    self.assertEqual(PWM1_REPEATFF, 0x0400)
    self.assertEqual(PWM1_SERIAL, 0x0200)
    self.assertEqual(PWM1_ENABLE, 0x0100)
    self.assertEqual(PWM0_MS_MODE, 0x0080)
    self.assertEqual(PWM_CLEAR_FIFO, 0x0040)
    self.assertEqual(PWM0_USEFIFO, 0x0020)
    self.assertEqual(PWM0_REVPOLAR, 0x0010)
    self.assertEqual(PWM0_OFFSTATE, 0x0008)
    self.assertEqual(PWM0_REPEATFF, 0x0004)
    self.assertEqual(PWM0_SERIAL, 0x0002)
    self.assertEqual(PWM0_ENABLE, 0x0001)


    ## Enums

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

    self.assertEqual(RPI_GPIO_P1_03, 0)
    self.assertEqual(RPI_GPIO_P1_05, 1)
    self.assertEqual(RPI_GPIO_P1_07, 4)
    self.assertEqual(RPI_GPIO_P1_08, 14)
    self.assertEqual(RPI_GPIO_P1_10, 15)
    self.assertEqual(RPI_GPIO_P1_11, 17)
    self.assertEqual(RPI_GPIO_P1_12, 18)
    self.assertEqual(RPI_GPIO_P1_13, 21)
    self.assertEqual(RPI_GPIO_P1_15, 22)
    self.assertEqual(RPI_GPIO_P1_16, 23)
    self.assertEqual(RPI_GPIO_P1_18, 24)
    self.assertEqual(RPI_GPIO_P1_19, 10)
    self.assertEqual(RPI_GPIO_P1_21, 9)
    self.assertEqual(RPI_GPIO_P1_22, 25)
    self.assertEqual(RPI_GPIO_P1_23, 11)
    self.assertEqual(RPI_GPIO_P1_24, 8)
    self.assertEqual(RPI_GPIO_P1_26, 7)
    self.assertEqual(RPI_V2_GPIO_P1_03, 2)
    self.assertEqual(RPI_V2_GPIO_P1_05, 3)
    self.assertEqual(RPI_V2_GPIO_P1_07, 4)
    self.assertEqual(RPI_V2_GPIO_P1_08, 14)
    self.assertEqual(RPI_V2_GPIO_P1_10, 15)
    self.assertEqual(RPI_V2_GPIO_P1_11, 17)
    self.assertEqual(RPI_V2_GPIO_P1_12, 18)
    self.assertEqual(RPI_V2_GPIO_P1_13, 27)
    self.assertEqual(RPI_V2_GPIO_P1_15, 22)
    self.assertEqual(RPI_V2_GPIO_P1_16, 23)
    self.assertEqual(RPI_V2_GPIO_P1_18, 24)
    self.assertEqual(RPI_V2_GPIO_P1_19, 10)
    self.assertEqual(RPI_V2_GPIO_P1_21, 9)
    self.assertEqual(RPI_V2_GPIO_P1_22, 25)
    self.assertEqual(RPI_V2_GPIO_P1_23, 11)
    self.assertEqual(RPI_V2_GPIO_P1_24, 8)
    self.assertEqual(RPI_V2_GPIO_P1_26, 7)
    self.assertEqual(RPI_V2_GPIO_P5_03, 28)
    self.assertEqual(RPI_V2_GPIO_P5_04, 29)
    self.assertEqual(RPI_V2_GPIO_P5_05, 30)
    self.assertEqual(RPI_V2_GPIO_P5_06, 31)


if __name__ == '__main__':
  unittest.main()
