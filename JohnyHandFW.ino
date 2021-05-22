#include "config.h"
#include "current.h"
#include "motor.h"

int current[5];
int dir = 1;
int target = 250/2;
IntervalTimer t1;
void foo();

void setup() {
  initPins();
  initCAN();
  initI2C();
  Serial.begin(115200);
  t1.begin(foo, 1000000);
}

void loop() {
  motor0.handle();
  motor1.handle();
  motor2.handle();
  motor3.handle();
  updateCurrent(current);
}

void foo(){
  target = target * -1;
  motor0.toPosition(target, 255);
  motor1.toPosition(target, 255);
  motor2.toPosition(target, 255);
  motor3.toPosition(target, 255);
  }
