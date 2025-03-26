#include "SensorGPS.h"
#include "Printer.h"
extern Printer printer;
  
SensorGPS::SensorGPS(void) 
: DataSource("lat,lon,nsats","float,float,uint8") 
{}

void SensorGPS::init(Adafruit_GPS* GPS)
{
  Serial.print("Initializing GPS... ");
  //Uart_p->begin(9600);
  //Serial.print("0");
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS->begin(9600);
  //Serial.print("1");
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //Serial.print("2");
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  //Serial.print("3");
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // GPS->sendCommand(PGCMD_ANTENNA);
  //Serial.print("4");
  
  // Ask for firmware version
  // mySerial.println(PMTK_Q_RELEASE);
  Serial.println(" done");
}

void SensorGPS::read(Adafruit_GPS* GPS)
{
  GPS->read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS->newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    updateState(GPS);
  
    if (!GPS->parse(GPS->lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}

void SensorGPS::updateState(Adafruit_GPS* GPS)
{
  //gps.f_get_position(&state.lat, &state.lon, &state.age);
  //state.hdop = gps.hdop();
  state.lat = GPS->latitudeDegrees;
  state.lon = GPS->longitudeDegrees;
  //state.age = GPS->age
  state.num_sat =GPS->satellites;
}

//convertDegMinToDecDeg (

float SensorGPS::convertDegMinToDecDeg (float degMin){
  float min = 0.0;
  float decDeg = 0.0;
  min =fmod((float)degMin,100.0);
  degMin= (int)(degMin/100);
  decDeg =degMin +(min/60);
  return decDeg;
}

String SensorGPS::printState(void)
{
  String printString = "GPS: Lat: ";
  printString += String(state.lat,7);
  printString += "[deg],";
  printString += " Lon: ";
  printString += String(state.lon,7);
  printString += "[deg],";
  printString += " Nsats: ";
  printString += String(state.num_sat);

  return printString;
}

size_t SensorGPS::writeDataBytes(unsigned char * buffer, size_t idx)
{
  float * float_slot = (float *) (buffer + idx);
  float_slot[0] = state.lat;
  float_slot[1] = state.lon;
  idx += 2*sizeof(float);

  //uint16_t * uint16_slot = (uint16_t *) (buffer + idx);
  //uint16_slot[0] = state.hdop;
  //idx += 1*sizeof(uint16_t);

  uint8_t * uint8_slot = (uint8_t *) (buffer + idx);
  uint8_slot[0] = state.num_sat;
  idx += sizeof(uint8_t);
  return idx;
}
