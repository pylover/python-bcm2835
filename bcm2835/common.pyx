

cdef extern from "bcm8235/_bcm8235.h" nogil:
    int bcm8235_init()

def init():
  return bcm8235_init()

