/********
Default E80 Code
Authors:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)                    
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <XYStateEstimator.h>
#include <ADCSampler.h>
#include <ErrorFlagSampler.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <SurfaceControl.h>
#define UartSerial Serial1
#define DELAY 0
#include <GPSLockLED.h>
#include <BurstADCSampler.h>
#include <HCSR04.h>
#include <AnemSampler.h>
#define ECHO 17
#define hallSensor 14

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
XYStateEstimator state_estimator;
SurfaceControl surface_control;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ADCSampler adc;
ErrorFlagSampler ef;
ButtonSampler button_sampler;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;
BurstADCSampler burst_adc;
AnemSampler anem_sampler;


// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
int echo_status = analogRead(ECHO);
volatile bool EF_States[NUM_FLAGS] = {1,1,1};
float data;

// GPS Waypoints
const int number_of_waypoints = 2;
const int waypoint_dimensions = 2;       // waypoints are set to have two pieces of information, x then y.
double waypoints [] = { 0, 10, 0, 0 };   // listed as x0,y0,x1,y1, ... etc.

// ANEMOMETER CODE
//int hallSensor = 14; //connect hall effect sensor output to the corresponding pin (A0)
int forceSensor = 15;
int revolution = 0; //initialize revolution count
double endTime = 0; //time of last rotation
double timeDifference;
double sampleTime = 1000; //how long to count number of rotations for one sample 
constexpr int N = 5; //number of samples to average across for cleanliness
double sampleSet[N];
int sampleIndex = 0;
double RPS; //number of rotations per second
const double anemometerConst = 10; // Anemometer constant (from wind tunnel) to change RPS to m/s
double windSpeed;

////////////////////////* Setup *////////////////////////////////

void setup() {
  analogReadAveraging(0);
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&surface_control);
  logger.include(&anem_sampler);
  logger.include(&motor_driver);
  logger.include(&adc);
  logger.include(&ef);
  logger.include(&button_sampler);
  logger.init();
  burst_adc.init();
  

  printer.init();
  ef.init();
  //button_sampler.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();

  surface_control.init(number_of_waypoints, waypoints, DELAY);
  
  state_estimator.init(); 

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  ef.lastExecutionTime              = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
//  button_sampler.lastExecutionTime  = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + XY_STATE_ESTIMATOR_LOOP_OFFSET;
  surface_control.lastExecutionTime = loopStartTime - LOOP_PERIOD + SURFACE_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  burst_adc.lastExecutionTime       = loopStartTime;
  anem_sampler.lastExecutionTime    = loopStartTime;

  // ANEMOMETER CODE
  //pinMode(hallSensor, INPUT_PULLUP);
  //when hallSensor pin goes from HIGH to LOW, call the ISR function
  attachInterrupt(digitalPinToInterrupt(hallSensor), ISR, FALLING);
}

//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();
 if (forceSensor <= 500) {
    motor_driver.drive(290,290,0);
  }else {
    motor_driver.drive(-150,0,0);
  }

  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,adc.printSample());
    printer.printValue(1,anem_sampler.printState());
    printer.printValue(2,logger.printState());
    printer.printValue(3,gps.printState());   
    printer.printValue(4,state_estimator.printState());     
    printer.printValue(5,surface_control.printWaypointUpdate());
    printer.printValue(6,surface_control.printString());
    printer.printValue(7,motor_driver.printState());
    printer.printValue(8,imu.printRollPitchHeading());        
    printer.printValue(9,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-surface_control.lastExecutionTime > LOOP_PERIOD ) {
    surface_control.lastExecutionTime = currentTime;
    surface_control.navigate(&state_estimator.state, &gps.state, DELAY);
    motor_driver.drive(surface_control.uL,surface_control.uR,0);
  }

  if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }

  if ( currentTime-ef.lastExecutionTime > LOOP_PERIOD ) {
    ef.lastExecutionTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
    delay(5);
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
    ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
    EF_States[0] = 1;
    EF_States[1] = 1;
    EF_States[2] = 1;
  }

 // uses the ButtonSampler library to read a button -- use this as a template for new libraries!
  // if ( currentTime-button_sampler.lastExecutionTime > LOOP_PERIOD ) {
  //   button_sampler.lastExecutionTime = currentTime;
  //   button_sampler.updateState();
  // }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }


  gps.read(&GPS); // blocking UART calls, need to check for UART every cycle

  if ( currentTime-state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    state_estimator.lastExecutionTime = currentTime;
    state_estimator.updateState(&imu.state, &gps.state);
  }
  
  if ( currentTime-led.lastExecutionTime > LOOP_PERIOD ) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if ( currentTime-logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }

  if ( currentTime-anem_sampler.lastExecutionTime > LOOP_PERIOD) {
    anem_sampler.lastExecutionTime = currentTime;
    float timeDifference = currentTime - endTime; 

    if (timeDifference > sampleTime) { 
    // RPS/
    RPS = (revolution / sampleTime) * 1000;
    anem_sampler.updateState(RPS);

    endTime = currentTime;
    revolution = 0; 
  }

  //windSpeed = avgRPS * anemometerConst;

  // Serial Monitor
  // Serial.print("Average RPS (Hz): ");
  // Serial.println(avgRPS);
  // Serial.print("Wind Speed (m/s): ");
  // Serial.println(windSpeed);
  // // Wait for 1 second before printing again
  // delay(1000);
  }

}

void EFA_Detected(void){
  EF_States[0] = 0;
}

void EFB_Detected(void){
  EF_States[1] = 0;
}

void EFC_Detected(void){
  EF_States[2] = 0;
}
  
// ANEMOMETER CODE
void ISR(){ 
  revolution += 1;
}

 double calculateAverage(double sampleSet[], int N) {
    double sum = 0.0;

    for (int i = 0; i < N; i++) {
        sum += sampleSet[i];
    }

    return sum / N;
}
