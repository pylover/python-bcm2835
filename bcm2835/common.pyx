

from _bcm2835 cimport c_HIGH, c_init

HIGH = c_HIGH

def init():
  return c_init()

