#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 99
#define PRINTER_LOOP_OFFSET 0
#define IMU_LOOP_OFFSET 10
#define GPS_LOOP_OFFSET 20
#define ADC_LOOP_OFFSET 30
#define ERROR_FLAG_LOOP_OFFSET 40
#define BUTTON_LOOP_OFFSET 45
#define XY_STATE_ESTIMATOR_LOOP_OFFSET 50
#define Z_STATE_ESTIMATOR_LOOP_OFFSET 55
#define DEPTH_CONTROL_LOOP_OFFSET 60
#define SURFACE_CONTROL_LOOP_OFFSET 65
#define LED_LOOP_OFFSET 70
#define LOGGER_LOOP_OFFSET 75

#endif
