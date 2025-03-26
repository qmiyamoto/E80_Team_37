#ifndef __Z_STATE_ESTIMATOR_H__
#define __Z_STATE_ESTIMATOR_H__

#include <Arduino.h>

#include "DataSource.h"

typedef struct {
    float z = 0; // z position (depth) in globad frame [m]
} z_state_t;

/*
 * ZStateEstimator class keeps track of the robot's depth state,
 * incorperating measurements of the system outputs from the Pressure
 * Sensor, as well as the control inputs to the system
 */
class ZStateEstimator : public DataSource
{
public:
  ZStateEstimator(void);

  // init
  void init(void);

  // State Access
  z_state_t state;

  void updateState(int pressure_signal);
  String printState(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  // set pressure sensor calibration slope and incercept below
  const float depthCal_slope = 1;
  const float depthCal_intercept = 1;

};

#endif
