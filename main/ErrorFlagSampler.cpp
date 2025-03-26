#include "ErrorFlagSampler.h"
#include <math.h>
#include "Printer.h"

extern Printer printer;

ErrorFlagSampler::ErrorFlagSampler(void) 
  : DataSource("ErrorFlagA,ErrorFlagB,ErrorFlagC","bool,bool,bool") // from DataSource
{}

void ErrorFlagSampler::init(void)
{
  for (int i=0; i<NUM_FLAGS; i++){
    pinMode(pinMap[i],INPUT);
  }
}

void ErrorFlagSampler::updateStates(bool EFA_State, bool EFB_State, bool EFC_State)
{
  flagStates[0] = !EFA_State;
  flagStates[1] = !EFB_State;
  flagStates[2] = !EFC_State;

  //delay(5);
  //detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
  //detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
  //detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
  // for (int i=0; i<NUM_FLAGS; i++){
  //  flagStates[i] = !EF_States[i];
  //  EF_States[i] = 0;
  //}
}
/*
void ErrorFlagSampler::EFA_Detected(void){
  EF_States[0] = 0;
}

void ErrorFlagSampler::EFB_Detected(void){
  EF_States[1] = 0;
}

void ErrorFlagSampler::EFC_Detected(void){
  EF_States[2] = 0;
}
*/
String ErrorFlagSampler::printStates(void)
{
  String motorNamesList [NUM_FLAGS] = {"MotorA: "," MotorB: "," MotorC: "};
  String printString = "Error Flags: ";
  for (int i=0; i<NUM_FLAGS; i++) {
    printString += motorNamesList [i];
    printString += String(flagStates[i]);
  }
  return printString;
}

size_t ErrorFlagSampler::writeDataBytes(unsigned char * buffer, size_t idx)
{
  bool * data_slot = (bool *) &buffer[idx];
  for (int i=0; i<NUM_FLAGS; i++) {
    data_slot[i] = flagStates[i];
  }
  return idx + NUM_FLAGS*sizeof(bool);
}
