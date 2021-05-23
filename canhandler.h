#include "config.h"

CAN_message_t msgIn; //для входящих сообщений
CAN_message_t msgOut; //для исходящих сообщений
bool updateNewPosition = false;
int newPosition[5];
int proceedCommand();
int proceedStatus();
int turnOff();
int emergencyStop();
int emergencyCancel();


bool checkIncomingData(){
  return can.read(msgIn);
  }

int proceedIncomingData(){
  short id = msgIn.id;
  int len = msgIn.len;
  int returnStatus = 0;
  if(DEBUG_CAN){
    Serial.printf("ID=0x%04x, Len=%d, Data:", id, len);
    for(int i=0; i<len; i++){
      Serial.printf(" %d", msgIn.buf[i]);
      }
    Serial.printf("\n");
    }
  switch(id){
    case CAN_ID_SET: {
      proceedCommand();
      break;
      }
    case CAN_ID_GET: {
      proceedStatus();
      break;
      }
    default:{
      if(DEBUG_CAN){
        Serial.printf("Unknown ID=0x%04x\n", id);
        returnStatus = -1;
        }
      break;
      }
    }
    return returnStatus;
  }

int proceedCommand(){
  int flag = msgIn.buf[0];
  updateNewPosition = false;
  switch(flag){
    case STATUS_IDLE:{
      if((status == STATUS_IDLE) or (status == STATUS_POSITION_MODE)){
        status = STATUS_IDLE;
        turnOff();
        }
      break;
      }
    case STATUS_POSITION_MODE:{
      if((status == STATUS_IDLE) or (status == STATUS_POSITION_MODE)){
        status = STATUS_POSITION_MODE;
        for(int i=0; i<5; i++){
          newPosition[i] = msgIn.buf[i+1];
          }
        updateNewPosition = true;
        }
      break;
      }
    case STATUS_CANCEL_EMERGENCY:{
      status = STATUS_IDLE;
      emergencyCancel();
      break;
      }
    case STATUS_EMERGENCY:{
      status = STATUS_EMERGENCY;
      emergencyStop();
      break;
      }
    }
  return flag;
  } 

int proceedStatus(){
  msgOut.id = CAN_ID_STATUS;
  msgOut.buf[0] = status;
  for(int i=0; i<5; i++){
    msgOut.buf[i+1] = positionRelative[i];
    }
  can.write(msgOut);
  return 0;
  }

int turnOff(){
  status = STATUS_IDLE;
  return 0;
  }

int emergencyStop(){
  status = STATUS_EMERGENCY;
  return 0;
  }

int emergencyCancel(){
  status = STATUS_IDLE;
  return 0;
  }
