#include "ButtonSampler.h"
#include "Printer.h"

extern Printer printer;

ButtonSampler::ButtonSampler(void) 
  : DataSource("Button","bool") // from DataSource
{}


void ButtonSampler::init(void)
{
  pinMode(USER_BUTTON,INPUT_PULLUP); 
  // when the button is not pressed the voltage
  // at USER_BUTTON will be high
}


void ButtonSampler::updateState(void)
// This function is called in the main loop of Default_Robot.ino
{
  // when the voltage at USER_BUTTON is low, the button 
  // has been pressed, thus buttonState is set to high
  buttonState = !digitalRead(USER_BUTTON);
}


String ButtonSampler::printState(void)
// This function returns a string that the Printer class 
// can print to the serial monitor if desired
{
  return "Button: " + String(buttonState);
}

size_t ButtonSampler::writeDataBytes(unsigned char * buffer, size_t idx)
// This function writes data to the micro SD card
{
  bool * data_slot = (bool *) &buffer[idx];
  data_slot[0] = buttonState;
  return idx + sizeof(bool);
}
