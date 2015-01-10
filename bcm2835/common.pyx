

cimport _bcm2835 as bcm

HIGH = bcm.c_HIGH

def init():
  return bcm.bcm2835_init()

