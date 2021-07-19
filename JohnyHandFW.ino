#include "config.h"
#include "current.h"
#include "motor.h"
#include "thumb.h"
#include "datahandler.h"
//#include "canhandler.h"

int current[5];
int defaultSpeed = 250;
int defaultHandler();
bool doReset = true;


void setup() {
  initPins();
  initCAN();
  initI2C();
  Serial.begin(115200);
}

void loop() {  
  calculatePosition();
  if(incomingCAN()){
    proceedCAN();
    defaultHandler();
  }
    
  if(incomingSerial()){
    proceedSerial();
    defaultHandler();
  }
 
  switch(status){
    case STATUS_IDLE:{
      stopAllMotors();
      break;
      }
    case STATUS_POSITION_MODE:{
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

int defaultHandler(){
  if((status == STATUS_IDLE)or(status == STATUS_POSITION_MODE)){
      if(doReset){
        doReset = false;
        motor0.reset();
        motor1.reset();
        motor2.reset();
        motor3.reset();
        motor4.reset();
        }
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
