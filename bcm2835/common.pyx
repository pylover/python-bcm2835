

cdef extern from "_bcm2835.h" nogil:
    int bcm8235_init()

def init():
  return bcm8235_init()

