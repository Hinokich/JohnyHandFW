#include "config.h"
#include "current.h"
#include "motor.h"

int current[5];

void setup() {
  initPins();
  initCAN();
  initI2C();
  Serial.begin(115200);
}

void loop() {
}
