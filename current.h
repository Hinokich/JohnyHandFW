#include "config.h"

int currentRawOffset = CURRENT_SENSOR_OFFSET;
int currentRawRatio = CURRENT_SENSOR_RATIO;

void updateCurrent(int* data){
  data[0] = analogRead(AIN_0) * 3.22f;
  data[1] = analogRead(AIN_1) * 3.22f;
  data[2] = analogRead(AIN_2) * 3.22f;
  data[3] = analogRead(AIN_3) * 3.22f;
  data[4] = analogRead(AIN_4) * 3.22f;
  }
