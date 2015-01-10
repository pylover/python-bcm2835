
cdef extern from "bcm2835.h":
  cdef int c_HIGH "HIGH"
  cdef extern int c_init "bcm2835_init" ()

