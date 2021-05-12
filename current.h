#include "config.h"

int currentRawOffset = CURRENT_SENSOR_OFFSET;
int currentRawRatio = CURRENT_SENSOR_RATIO;

void updateCurrent(int* data){
  data[0] = ((analogRead(AIN_0) * 3.22f) - currentRawOffset)/currentRawRatio;
  data[1] = ((analogRead(AIN_0) * 3.22f) - currentRawOffset)/currentRawRatio;
  data[2] = ((analogRead(AIN_0) * 3.22f) - currentRawOffset)/currentRawRatio;
  data[3] = ((analogRead(AIN_0) * 3.22f) - currentRawOffset)/currentRawRatio;
  data[4] = ((analogRead(AIN_0) * 3.22f) - currentRawOffset)/currentRawRatio;
  }
