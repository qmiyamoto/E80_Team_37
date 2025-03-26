#ifndef __BUTTONSAMPLER_h__
#define __BUTTONSAMPLER_h__

#include <Arduino.h>
#include "DataSource.h"
#include "Pinouts.h"

/*
 * ButtonSampler implements SD logging for the onboard pushbutton 
 */


class ButtonSampler : public DataSource
{
public:
  ButtonSampler(void);

  void init(void);

  // Managing state
  bool buttonState;
  void updateState(void);
  String printState(void);

  // Write out
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
  
};

#endif