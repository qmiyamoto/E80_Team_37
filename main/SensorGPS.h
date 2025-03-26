#ifndef __SENSOR_GPS_H__
#define __SENSOR_GPS_H__

#define GPS_READ_INTERVAL 3

// Number of aquired satellites needed for accurate position measurement
// 6 is recommeneded
#define N_SATS_THRESHOLD 5 


#include <Arduino.h>
#include "DataSource.h"
#include "Pinouts.h"
#include "Adafruit_GPS.h"

typedef struct {
  float lat;
  float lon;
  uint32_t age;
  uint16_t hdop;
  uint8_t num_sat;
} gps_state_t;

class SensorGPS : public DataSource {
public:
	SensorGPS(void);
  
  // Starts the connection to the sensor
  void init(Adafruit_GPS* GPS);

  // Reads data from the sensor, returns whether new data was received
  void read(Adafruit_GPS* GPS);

  // Latest reported data is stored here
  gps_state_t state;
  String printState(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  
  // Adafruit_GPS GPS(Serial1); // *** doesn't work

  HardwareSerial * Uart_p;

  // copies over latest data from gps object to the state struct
  void updateState(Adafruit_GPS* GPS);

  float convertDegMinToDecDeg (float degMin);

  
};

#endif
