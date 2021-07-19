#include "config.h"

int buffer[9]; //ID + 8 data bytes

CAN_message_t msgIn; //для входящих сообщений
CAN_message_t msgOut; //для исходящих сообщений

String msgInSerial;
String msgOutSerial;

int proceedData();
int proceedCommand();
int proceedStatus();
int turnOff();
int emergencyStop();
int emergencyCancel();


bool incomingSerial(){
  return Serial.available();
  }

bool incomingCAN(){
  return can.read(msgIn);
  }

int proceedSerial(){
  msgInSerial = Serial.readString();
  char buf[2];
  char bufID[4];
  for(int i=0; i<4; i++){
    bufID[i] = msgInSerial.charAt(i);
    }
  buffer[0] = strtoul(bufID, NULL, 16);
  for(int i=1; i<9; i++){
    buf[0] = msgInSerial.charAt(i*2 + 2);
    buf[1] = msgInSerial.charAt(i*2 + 3);
    buffer[i] = strtoul(buf, NULL, 16);
    }
  Serial.printf("ID: 0x%02x, data: ", buffer[0]);
  for(int i=1; i<9; i++){
    Serial.printf("0x%02x ", buffer[i]);
    }
  Serial.printf("\n");
  proceedData();
  }

int proceedCAN(){
  buffer[0] = msgIn.id;
  for(int i=0; i<8; i++){
    buffer[i+1]=msgIn.buf[i];
    }
  proceedData();
  }

int proceedData(){
  int id = buffer[0];
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
        }
      break;
      }
    }
  }

int proceedCommand(){
  int flag = buffer[1];
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
          newPosition[i] = buffer[i+2];
          }
        updateNewPosition = true;
        }
      break;
      }
    case STATUS_LIBRARY_MODE:{
      if((status == STATUS_IDLE) or (status == STATUS_POSITION_MODE)){
        status = STATUS_POSITION_MODE;
        for(int i=0; i<5; i++){ 
          int positionID = buffer[2]; //там лежит ID жеста 
          newPosition[i] = positionLibrary[positionID][i]; //берем жест из библиотеки жестов
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
  String bufOut;
  bufOut += "04ea";
  for(int i=0; i<5; i++){
    int temp_rPos = positionRelative[i];
    msgOut.buf[i+1] = temp_rPos;
    char temp_buf[2];
    sprintf(temp_buf, "%02x", temp_rPos); 
    bufOut += temp_buf;
    }
  can.write(msgOut);
  Serial.println(bufOut);
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
