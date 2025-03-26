#include "SensorIMU.h"
#include "Printer.h"
extern Printer printer;

SensorIMU::SensorIMU(void)
  : DataSource("rollIMU,pitchIMU,headingIMU,accelX,accelY,accelZ,magX,magY,magZ",
               "float,float,float,float,float,float,float,float,float") {
}

void SensorIMU::init(void) {
  Serial.print("Initializing IMU... ");

	// Creating i2c interface
  myIMU.dev_i2c->begin();
  
  // Starting and Enabling Accelerometer and Magnetometer
  if ( myIMU.Acc->begin()                    == LSM303AGR_ACC_STATUS_OK
    && myIMU.Acc->Enable()                   == LSM303AGR_ACC_STATUS_OK 
    && myIMU.Acc->EnableTemperatureSensor()  == LSM303AGR_ACC_STATUS_OK 
    && myIMU.Mag->begin()                    == LSM303AGR_MAG_STATUS_OK
    && myIMU.Mag->Enable()                   == LSM303AGR_MAG_STATUS_OK ){
  
    Serial.println("done");
  }
  else {
    Serial.println("failed!");
  }
}

void SensorIMU::read(void) {

  // Get new data samples
  int32_t raw_acc_data[3];
  myIMU.Acc->GetAxes(raw_acc_data);
		
  float ax = (float)raw_acc_data[0];
  float ay = -(float)raw_acc_data[1]; // The IMU STILL doesn't use the RHR
  float az = (float)raw_acc_data[2];

  int32_t raw_mag_data[3];
  myIMU.Mag->GetAxes(raw_mag_data);

  float mx = (float)raw_mag_data[0]; 
  float my = -(float)raw_mag_data[1]; // The IMU STILL doesn't use the RHR
  float mz = (float)raw_mag_data[2]; 

  // Remove offsets from acceleration measurements
  state.accelX = ax - accel_offsets[0];
  state.accelY = ay - accel_offsets[1];
  state.accelZ = az - accel_offsets[2];

  // mGauss to uTesla
  mx = mx * .1;
  my = my * .1;
  mz = mz * .1;

  // Remove offsets from magnertometer measurements (base values in uTesla)
  mx = mx - mag_offsets[0];
  my = my - mag_offsets[1];
  mz = mz - mag_offsets[2];
  
  // Apply magnetometer soft iron error compensation
  state.magX = mx * mag_ironcomp[0][0] + my * mag_ironcomp[0][1] + mz * mag_ironcomp[0][2];
  state.magY = mx * mag_ironcomp[1][0] + my * mag_ironcomp[1][1] + mz * mag_ironcomp[1][2];
  state.magZ = mx * mag_ironcomp[2][0] + my * mag_ironcomp[2][1] + mz * mag_ironcomp[2][2];

  // populate the roll, pitch, yaw with simple orientation calcs  
  getOrientation(state.accelX,state.accelY,state.accelZ,state.magX,state.magY,state.magY);
}

String SensorIMU::printRollPitchHeading(void) {
  String printString = "IMU:"; 
  printString += " roll: ";
  printString += String(state.roll);
  printString += "[deg],";
  printString += " pitch: "; 
  printString += String(state.pitch);
  printString += "[deg],";
  printString += " heading: ";
  printString += String(state.heading);
  printString += "[deg]";
  return printString; 
}

String SensorIMU::printAccels(void) {
  String printString = "IMU:";

  printString += " accelX: ";
  printString += String(state.accelX);
  printString += "[mg], ";
  printString += " accelY: ";
  printString += String(state.accelY);
  printString += "[mg], ";
  printString += " accelZ: ";
  printString += String(state.accelZ);
  printString += "[mg]";

	// Used to Debug
  
  //printString += " magX: ";
  //printString += String(state.magX);
  //printString += "[uT], ";
  //printString += " magY: ";
  //printString += String(state.magY);
  //printString += "[uT], ";
  //printString += " magZ: ";
  //printString += String(state.magZ);
  //printString += "[uT]";

  return printString;
}

size_t SensorIMU::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = state.roll;
  data_slot[1] = state.pitch;
  data_slot[2] = state.heading;
  data_slot[3] = state.accelX;
  data_slot[4] = state.accelY;
  data_slot[5] = state.accelZ;
  data_slot[6] = state.magX;
  data_slot[7] = state.magY;
  data_slot[8] = state.magZ;
  return idx + 9*sizeof(float);
}

void SensorIMU::getOrientation(float ax, float ay, float az, float mx, float my, float mz) {
  // copied from Adafruit_Simple_AHRS
  
  float const PI_F = 3.14159265F;

  // roll: Rotation around the X-axis. -180 <= roll <= 180                                          
  // a positive roll angle is defined to be a clockwise rotation about the positive X-axis          
  //                                                                                                
  //                    ay                                                                           
  //      roll = atan2(----)                                                                         
  //                    az                                                                           
  //                                                                                                                                      
  state.roll = (float)atan2(ay, az);

  // pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         
  // a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         
  //                                                                                                
  //                                 -ax                                                             
  //      pitch = atan(---------------------------------)                                             
  //                    ay * sin(roll) + az * cos(roll)                                               
  //                                                                                                                                  
  if (ay * sin(state.roll) + az * cos(state.roll) == 0)
    state.pitch = ax > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    state.pitch = (float)atan(-ax / (ay * sin(state.roll) + az * cos(state.roll)));

  // heading: Rotation around the Z-axis. -180 <= roll <= 180                                       
  // a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       
  //                                                                                                
  //                                       mz * sin(roll) - my * cos(roll)                            
  //   heading = atan2(------------------------------------------------------------------------------)  
  //                    mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll))   
  //                                                                                                                                   
  state.heading = -(float)atan2(mz * sin(state.roll) - my * cos(state.roll), \
                               mx * cos(state.pitch) + \
                               my * sin(state.pitch) * sin(state.roll) + \
                               mz * sin(state.pitch) * cos(state.roll));

  // convert to degrees
  state.roll *= 180.0/PI_F;
  state.pitch *= 180.0/PI_F;
  state.heading *= 180.0/PI_F;
  
}
