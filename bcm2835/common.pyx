
cdef extern from "_bcm2835.h" nogil:
  int c_HIGH "HIGH"
  int c_LOW "LOW"
  int c_bcm2835_init "bcm2835_init"()


HIGH = c_HIGH
LOW = c_LOW

def init():
  return c_bcm2835_init()

__all__ = [
  'HIGH',
  'LOW',

  'init'
]