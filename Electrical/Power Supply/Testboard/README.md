# Power Supply - Test Board
  DCDC buck converter test board with 5v and 12v tests.

  Tested with the following resistance values:
  * 5v -> 1.96k
  * 12v -> 714

  Approximate voltage is 4.9v and 12v, due to lower resistor value for 5v.

## Noise
  Upon testing, an approximate 1Vpp is detected due to low inductance.  This
  has been tested via analysis and inspection.

  The 12v router supply will be shared with a 5v LDO to increase current draw,
  lower cost, and lower the minimum inductor value.
  
  Testing has prompted design change to use a larger inductor on sources, and
  5v LDO source to reduce ripple.
