/* DUMMY CLASS! USE AS A TEMPLATE FOR NEW LIBRARIES! */

// only define this class if it hasn't been before
#ifndef __GPSLockLED_H__
#define __GPSLockLED_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"
#include <SensorGPS.h>

// controls how often and when in the loop this class's functions run


class GPSLockLED {
public: // for functions outside code might call
  GPSLockLED(void);

  void init(void);

  void flashLED(gps_state_t * gps_state_p);

  int lastExecutionTime = -1;

private: // for helper functions and hidden variables

  int cycleNum = 0;
  
};

#endif
