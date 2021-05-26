#include "config.h"
#include "current.h"
#include "motor.h"
#include "thumb.h"
#include "canhandler.h"

int current[5];
int defaultSpeed = 150;

void setup() {
  initPins();
  initCAN();
  initI2C();
  Serial.begin(115200);
}

void loop() {  
  calculatePosition();
  
  if(checkIncomingData()){
    proceedIncomingData();
    if((status == STATUS_IDLE)or(status == STATUS_POSITION_MODE)){
      if(updateNewPosition){
        motor0.toPosition(newPosition[0], defaultSpeed);
        motor1.toPosition(newPosition[1], defaultSpeed);
        motor2.toPosition(newPosition[2], defaultSpeed);
        motor3.toPosition(newPosition[3], defaultSpeed);
        motor4.toPosition(newPosition[4], defaultSpeed);
        updateNewPosition = false;
        }
      }
    }
    
  switch(status){
    case 0:{
      stopAllMotors();
      break;
      }
    case 1:{
      motor0.handle();
      motor1.handle();
      motor2.handle();
      motor3.handle();
      motor4.handle();
      break;
      }
    default:{
      stopAllMotors();
      }
    }
}
