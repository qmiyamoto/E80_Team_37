#ifndef __ERRORFLAGSAMPLER_h__
#define __ERRORFLAGSAMPLER_h__

#include <Arduino.h>
#include "DataSource.h"
#include "Pinouts.h"
//#include <avr/io.h>
//#include <avr/interrupt.h>

/*
 * ErrorFlagSampler implements SD logging for the three digital 
 * channels hardwired to the error flag output of each H-bridge
 */

#define NUM_FLAGS 3

class ErrorFlagSampler : public DataSource
{
public:
  ErrorFlagSampler(void);

  void init();

  // Managing state
  bool flagStates [NUM_FLAGS];
  void updateStates(bool EFA_State, bool EFB_State, bool EFC_State);
  String printStates(void);

  // Write out
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

  //ISR functions
	//void EFA_Detected(void);
	//void EFB_Detected(void);
  //void EFC_Detected(void);

private:

  const int pinMap[NUM_FLAGS] =  {ERROR_FLAG_A,ERROR_FLAG_B,ERROR_FLAG_C};

  // volatile bool EF_States[NUM_FLAGS] = {1,1,1};


};

#endif