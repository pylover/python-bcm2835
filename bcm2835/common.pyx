

from _bcm2835 cimport c_HIGH, bcm2835_init

HIGH = c_HIGH

def init():
  return bcm2835_init()

