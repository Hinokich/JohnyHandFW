#include "config.h"
#include "current.h"
#include "motor.h"

int current[5];
int dir = 1;
IntervalTimer t1;
void foo();

void setup() {
  initPins();
  initCAN();
  initI2C();
  Serial.begin(115200);
  t1.begin(foo, 5000000);
}

void loop() {
  motor0.handle();
  updateCurrent(current);
  //Serial.println(current[0]);
}

void foo(){
  motor0.toPosition(35*7*1*dir, 200);
  dir = dir * -1;
  }
