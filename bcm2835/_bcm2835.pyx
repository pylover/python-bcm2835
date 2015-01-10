
cdef extern from "bcm2835.h":
  cdef int c_HIGH "HIGH"
  int bcm2835_init(void)
  int bcm2835_close(void)

def init():
    return bcm2835_init()

def close():
    return bcm2835_close()