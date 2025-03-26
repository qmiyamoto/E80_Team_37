# E80-AUV
Arduino project for the Teensy microcontroller used for the HMC E80 AUV.

## Authors
- Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
- Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
- Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
- Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016) 

COPY OR SYMLINK THIS LIBRARIES FOLDER INTO THE ARDUINO FOLDER -- IT DOES NOTHING ANYWHERE ELSE

## Overview
This document describes the functionality of the 2019 E80 code base. It is highly recommended that you also read the motherboard instructional document which describes the hardware interfaced via this code.

## TimingOffsets.h
- [TimingOffsets.h](./TimingOffsets.h)

This header file defines LOOP_PERIOD to be 99 ms to achieve a sampling rate of 10Hz (experimentally 99 ms achieved this better than 100 ms). The file also defines offsets for each service in the main loop. The suggested defaults are already set in the file, but if any significant alterations are made or if additional functionality is implemented in new classes, these timing offsets will need to be modified and/or added to.

## Pinouts.h
- [Pinouts.h](./Pinouts.h)

This header file defines the functionality of many of the pins that are hardwired on the motherboard. See the motherboard instructional document and the motherboard schematic for specifics. SPEAKER_PIN and MIC_PIN are for the time of flight library and are currently out of date

## DataSource
- [DataSource.h](./DataSource.h)
  
'DataSource' is an abstract class representing a source of data to be logged. 'DataSource' objects must have a 'varNames' String member which contains the names of the columns, a 'varTypes' String member which contains the data types of all the columns and a 'writeDataBytes' function which writes the raw binary data of each variable to the passed in buffer. varNames and varTypes are created by an implicit constructor that is inherited from DataSource.

## ADCSampler
- [ADCSampler.h](./ADCSampler.h)
- [ADCSampler.cpp](./ADCSampler.cpp)
  
By default this class samples the analog voltage at all of the Teensy's analog input pins and uses the DataSource interface to log their values to the SD card as ints. Teensy analog pin A7 (21) is hardwired to the output of a current sensing circuit implemented of the motherboard and is measured and logged the same way as all the other analog pins. Many of the unused pins wired to the protoboard are located on surface mount pads on the underside of the Teensy. These pins can be used the same as any other Teensy pins with two exceptions. Analog pins A10-A14 cannot be used as digital IO pins. In addition, these same pins are not 5V tolerant. They will break if more than 5V is applied directly to the pin. A comprehensive Teensy 3.2 pinout spreadsheet should be linked on the E80 website.

## ButtonSampler
- [ButtonSampler.h](./ButtonSampler.h)
- [ButtonSampler.cpp](./ButtonSampler.cpp)

This class simply reads the state of the user button of the motherboard and logs the state as a boolean through the DataSource interface. This is a great bare-bones class the use as a template for new classes.

## ErrorFlagSampler
- [ErrorFlagSampler.h](./ErrorFlagSampler.h)
- [ErrorFlagSampler.cpp](./ErrorFlagSampler.cpp)

Each H-bridge has a digital output pin called EF for error flag that is pulled low under various fault conditions such as short circuit or under-voltage. Unfortunately these H-bridge outputs are not buffered and seem to be somewhat coupled to the H-bridge output. As a result the voltage output varies significantly over time. To get around this complication, hardware interrupts are set for each pin in the DefaultRobot.ino file using the attachInterrupt() function. These interrupts detect the falling edge of each error flag signal. The current state of all the error flag signals are passed to the ErrorFlagSampler class from within the DefaultRobot.ino file using the updateStates() function.
ErrorFlagSampler and its interrupts can serve as good inspiration for programming an encoder or a fast pushbutton detector.

## GPSLockLED
- [GPSLockLED.h](./GPSLockLED.h)
- [GPSLockLED.cpp](./GPSLockLED.cpp)

This class controls an LED on the motherboard that blinks with a duty cycle proportional to the number of satellites received by the GPS module. The function flashLED takes the current state of the GPS (which includes num_sat: the number of acquired satellites) as input.


## Logger
- [Logger.h](./Logger.h)
- [Logger.cpp](./Logger.cpp)

The 'Logger' class handles all the logging. It is written in a way to hide away the intricacies of logging the heterogenous set of variables a user may want to log.
In the main program's setup function, the 'Logger' object's 'include' function is run, passing pointers to any 'DataSource' objects that should be logged.
Next, the 'Logger' object's 'init' function is run, which automatically finds an unused filename. Each log creates two files, 'INFXXX.txt' and 'LOGXXX.bin', where the 'XXX' is the smallest number for which a file with the same name doesn't already exist on the SD card.
The 'INF' file describes the schema of the data being logged. Its first line contains comma separated column names, and its second line contains comma separated column data types. The 'LOG' file contains the raw binary values of each variable being logged, with each row of data written in the order described by the 'INF' file.
The logging actually happens through two functions: 'log' and 'write';
The 'log' function samples all the 'DataSource' objects that were registered to the 'Logger' during setup by the 'include' function. It calls the 'writeDataBytes' function of each 'DataSource', to write the binary values of the variables to be logged to a buffer.  This buffer is composed of 16 blocks stored in the "blocks" variable--on the Teensy, not the SD.  Internally, the 'Logger' object keeps track of which of these blocks have been logged and which have been written to the sd. The 'log' function grabs a blank block and writes the data to it.  This transition is done no matter how full the previous block was, typically adding about 120 bytes of buffer into each log.  This blank space stops any strange writing hiccups from throwing off later data values.
** Note: the 'log' function does NOT actually write anything to the SD card. The fact that this function only writes to a buffer means it takes less than a ms typically. **
** Note: the 'log' function does decide what data is recorded.  If it is called more often than data is updated, data will appear duplicated on the SD card.  If it is called less often, measured data will be lost.  'write' has no effect on what data is kept, unless it is called so infrequently that the buffer blocks overflow. **
The actual writing to the SD card is handled by the 'write' function. This function loops as long as there is space in the LOG file on the SD card available to write. To add space, increase the FILE_BLOCK_COUNT variable.  Upon each loop, it checks if there are any newly logged blocks to write to the SD card.  With the standard times, there should be 4 blocks logged, which are then transferred over to the SD card.
** Note: logger.write() is the longest function in the main loop by a factor of ~8.  It takes around 23 ms in my experience -- but verify that for your teensy and data if you're concerned about it.  Take this time into account when deciding how to offset calls in the main loop. **
** Note: The current logging infrastructure is not designed to log long series of ADC values all at once (for example if an ADC were setup to sample continuously for a period of time). If this functionality is absolutely necessary, prepare to do some restructuring of the logger class.

## MotorDriver
- [MotorDiver.h](./MotorDriver.h)
- [MotorDiver.cpp](./MotorDriver.cpp)

This is the class that actually drives each of the motor pins.
`init()` initializes each of the pins for PWM signals (where the pins are decided by Pinouts.h).
`apply()` writes PWM signals to the pins according to the pwmValues and pwmDir arrays which determine the PWM duty cycle and polarity sent to the H-bridges. 

Changing these variables and then calling `apply()` allows you to change the power directed to each motor.
Additionally, a public function called `drive()` is provided which takes control signals for each motor in the form of signed integers as input and internally calls the apply() function.

## SurfaceControl
- [SurfaceControl.h](./SurfaceControl.h)
- [SurfaceControl.cpp](./SurfaceControl.cpp)

Controls the movement of default_robot.
The `init()` call stores the desired waypoints -- to change the waypoints, add more, or set up a more complicated system (such as adding a new dimensions like heading or height to the state), go to the `init()` call in the Initialize section of code.

The `calculateControl()` function is the heart of SurfaceControl.
It uses `updatePoint()` to check whether it should change which point it is targeting, sorts out how it should get there using proportional control of yaw, then sends this information to the motors with `driveMotors()`.  Some of this may be left to you to code.

**Note**: To get the coordinates of a wayPoint, you should use `getWayPoint()`.
The short answer is just because; use 0 for x and 1 for y of the current coordinate.
The long one is: though wayPoints is initially created as a two dimensional array, it's stored in SurfaceControl as a pointer to a double.
While you can often treat pointers as arrays, two dimensional arrays are more complicated.
The call wayPoints[i] really translates to `*(wayPoints + i)`, or the data value stored wherever wayPoints points to plus `i*sizeof(double)` bytes over. Since it isn't stored in SurfaceControl as a two dimensional array, a call to `wayPoints[i][j]` would return `*(*(wayPoints + i) + j)` which doesn't make any sense to the compiler, because the inner value is a double, not a pointer. 
Therefore getWayPoint only uses one bracket, and more manually increments by factors of stateDims if totalWayPoints != 0. Hope that made any sense.

## Printer
- [Printer.h](./Printer.h)
- [Printer.cpp](./Printer.cpp)

This is a really useful library that should stop your having to use any `Serial.println()` calls, except for debugging.  
The `init()` for printer opens the Serial port, so this is especially important.
The `printValue()` and `printMessage()` functions allow you to print repeated values or one-time messages easily.  
Finally, the `printToSerial()` call actually sends all of the information to your computer (if available).

`printValue()` takes in a row number, which is the row the value String will be displayed on, and a string. 
This is useful for values that you plan on displaying over and over, where you will simply want to update the value itself as you get new information.

`printMessage()` takes in a String and an integer representing the amount of times you want to display the message.  By default, `printToSerial()` is called 10 times per second, so 10 corresponds to 1 second, and 100 to 10 seconds, etc.
This is useful for error messages and notifications that you want to appear once but don't want to constantly update.

**PROTIP**: A time value of 0 will stay displayed indefinitely, until the teensy is reset or until other messages force it off the screen.  This is used in many libraries for printing error messages that will stay visible over time.

## SensorGPS
- [SensorGPS.h](./SensorGPS.h)
- [SensorGPS.cpp](./SensorGPS.cpp)

This class is a wrapper for the GPS sensor, using the AdafruitGPS library included in a separate folder.  
As with the IMU library, internal state variables hold the latest results, which are then stored into the logger through the DataSource interface.
Read calls again take about 3 ms.

Before the GPS module acquires sufficient satellites the values read from the module are not accurate. Based on some preliminary testing, when the number of acquired satellites is 6 or greater the GPS achieves full accuracy.
To prevent StateEstimator and PControl from doing the wrong thing before accurate position measurements are made, a threshold is defined in the header file (`#define N_SATS_THRESHOLD 6`).
Checks are built into PControl and StateEstimator such that if the `num_sats` variable within the `gps_state_t` struct is below this threshold, the AUV will not start trying to navigate to the first waypoint and appropriate messages will be printed to the serial monitor

## SensorIMU
- [SensorIMU.h](./SensorIMU.h)
- [SensorIMU.cpp](./SensorIMU.cpp)

This class is a wrapper for the IMU sensor, offering functions to poll the IMU for new data, and keeping internal state variables holding the latest received data.
Calls are made using functions from the IMU-libraries folder, which contains libraries Sparkfun made to work with the IMU. 
This class implements the DataSource interface.
Read calls for the IMU typically take about 3 ms.

## StateEstimator
- [StateEstimator.h](./StateEstimator.h)
- [StateEstimator.cpp](./StateEstimator.cpp)

The `StateEstimator` class defines an object which handles all the state estimation of the robot. 
It keeps an internal `state` struct which stores the current estimate of the state. 
The latitude and longitude of the origin are set as private variables in the header file.

The main `updateState()` function incorporates data from the IMU and GPS and converts them to states to be used by PControl or any other driving library.
It also implements the 'DataSource' interface to plug into the 'Logger' framework.

