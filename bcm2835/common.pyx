
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
  int c_bcm2835_init "bcm2835_init"()


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
Base Physical Address of the Pads registers
"""
GPIO_PADS = c_BCM2835_GPIO_PADS

"""
Base Physical Address of the Clock/timer registers
"""
CLOCK_BASE = c_BCM2835_CLOCK_BASE

"""
Base Physical Address of the GPIO registers
"""
GPIO_BASE = c_BCM2835_GPIO_BASE

"""
Base Physical Address of the SPI0 registers
"""
SPI0_BASE = c_BCM2835_SPI0_BASE

"""
Base Physical Address of the BSC0 registers
"""
BSC0_BASE = c_BCM2835_BSC0_BASE

"""
Base Physical Address of the PWM registers
"""
GPIO_PWM = c_BCM2835_GPIO_PWM

"""
Base Physical Address of the BSC1 registers
"""
BSC1_BASE = c_BCM2835_BSC1_BASE

def init():
  return c_bcm2835_init()

__all__ = [
  'HIGH',
  'LOW',
  'CORE_CLK_HZ',
  'PERI_BASE',
  'ST_BASE',
  'GPIO_PADS',
  'CLOCK_BASE',
  'GPIO_BASE',
  'SPI0_BASE',
  'BSC0_BASE',
  'GPIO_PWM',
  'BSC1_BASE',
  'init'
]
