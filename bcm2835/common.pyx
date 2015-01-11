

cdef extern from "_bcm8235.h":
    int bcm8235_init()

def init():
  return bcm8235_init()

