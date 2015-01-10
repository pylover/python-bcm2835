
cdef extern from "bcm2835.h":
  cdef int c_HIGH "HIGH"
  int bcm2835_init()

