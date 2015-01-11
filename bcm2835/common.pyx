
cdef extern from "_bcm2835.h" nogil:
  # This means pin HIGH, true, 3.3volts on a pin.
  int c_HIGH "HIGH"

  # This means pin LOW, false, 0volts on a pin.
  int c_LOW "LOW"

  # Speed of the core clock core_clk
  int c_BCM2835_CORE_CLK_HZ "BCM2835_CORE_CLK_HZ"

  # Physical addresses for various peripheral register sets
  # Base Physical Address of the BCM 2835 peripheral registers
  int c_BCM2835_PERI_BASE "BCM2835_PERI_BASE"

  # Base Physical Address of the System Timer registers
  int c_BCM2835_ST_BASE "BCM2835_ST_BASE"

  # Base Physical Address of the Pads registers
  int c_BCM2835_GPIO_PADS "BCM2835_GPIO_PADS"

  # Base Physical Address of the Clock/timer registers
  int c_BCM2835_CLOCK_BASE "BCM2835_CLOCK_BASE"

  # Base Physical Address of the GPIO registers
  int c_BCM2835_GPIO_BASE "BCM2835_GPIO_BASE"

  # Base Physical Address of the SPI0 registers
  int c_BCM2835_SPI0_BASE "BCM2835_SPI0_BASE"

  # Base Physical Address of the BSC0 registers
  int c_BCM2835_BSC0_BASE "BCM2835_BSC0_BASE"

  # Base Physical Address of the PWM registers
  int c_BCM2835_GPIO_PWM "BCM2835_GPIO_PWM"

  # Base Physical Address of the BSC1 registers
  int c_BCM2835_BSC1_BASE "BCM2835_BSC1_BASE"


  int c_bcm2835_init "bcm2835_init"()


HIGH = c_HIGH
LOW = c_LOW
CORE_CLK_HZ = c_BCM2835_CORE_CLK_HZ
PERI_BASE = c_BCM2835_PERI_BASE
ST_BASE = c_BCM2835_ST_BASE
GPIO_PADS = c_BCM2835_GPIO_PADS
CLOCK_BASE = c_BCM2835_CLOCK_BASE
GPIO_BASE = c_BCM2835_GPIO_BASE
SPI0_BASE = c_BCM2835_SPI0_BASE
BSC0_BASE = c_BCM2835_BSC0_BASE
GPIO_PWM = c_BCM2835_GPIO_PWM
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