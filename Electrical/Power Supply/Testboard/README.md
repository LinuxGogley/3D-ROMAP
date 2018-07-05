# Intel Realsense Capture
  DCDC buck converter test board with 5v and 12v tests.

  Tested with the following resistance values:
  * 5v -> 1.96k
  * 12v -> 714

  Approximate voltage is 4.9v and 12v, due to lower resistor value for 5v.

## Noise
  Upon testing, an approximate 1Vpp is detected close to the switching frequency.  This
  has been tested via analysis and inspection.
  
  Testing has prompted design change to exclude the 5v source due to its high inductance requirement.
