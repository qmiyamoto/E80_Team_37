/* DUMMY CLASS! USE AS A TEMPLATE FOR NEW LIBRARIES! */

// this includes everything from the header file
#include "GPSLockLED.h"

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
GPSLockLED::GPSLockLED(void) {
}

void GPSLockLED::init(void) {
  pinMode(GPS_LOCK_LED,OUTPUT); // GPS_LOCK_LED is defined in pinouts

  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  // printer.printMessage("Initialized LED at " + String(millis()), 10);
}


void GPSLockLED::flashLED(gps_state_t * gps_state_p) {
// flashLED blinks the GPS Lock LED with duty cycle proprotional 
// to the number satellites acquired. When the number of satellites
// reaches N_SATS_THRESHOLD as defined in SensorGPS.h, the LED
// remains illuminated
  if(gps_state_p->num_sat >= N_SATS_THRESHOLD){
    digitalWrite(GPS_LOCK_LED,HIGH);
  }
  else{
    if(cycleNum > N_SATS_THRESHOLD){
      cycleNum = 0;
    }
    if(cycleNum < gps_state_p->num_sat){
      digitalWrite(GPS_LOCK_LED,HIGH);
      cycleNum++;
    }
    else{
      digitalWrite(GPS_LOCK_LED,LOW);
      cycleNum++;
    }

  }
}

