

cdef extern from "_bcm2835.h" nogil:
    int bcm2835_init()

def init():
  return bcm2835_init()

