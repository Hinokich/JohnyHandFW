#include "config.h"
#pragma once 

void ISR_4();

class Thumb{
  public:
  Thumb(int num, int maxRange, int encoderPin, int speedPin, void ISR(), int currentLimit = MOTOR_DEFAULT_CURRENT_LIMIT, int maximumSpeed = 255, bool isInverted = false, int pinA =0, int pinB=1); //большой палец
  int toPosition(int pos, int velocity = 255);
  int toAbsolutePosition(int pos, int velocity);
  int getPosition();
  int getSpeed();
  void ISR();
  int reset();
  int forward(int spd=255);
  int backward(int spd=255);
  int stop(); 
  int handle();
  void setPID(float Kp, float Ki, float Kd);
  int range = MOTOR_THUMB_RANGE; //диапазон работы в тактах энкодера
  
  private:
  int pA;
  int pB;
  int pwmPin; //пин для ШИМ управления драйвером
  int isrPin; //пин для прерываний энкодера
  int position = 0; //текущая позиция
  int speed = 255; //текущая скорость
  int maxSpeed = 255; //макс скорость
  int targetPosition = 0; //целевая позиция
  bool inverted = false; //инвертирсия управления 
  int id = 0; //номер мотора в кисти  
  int direction = 0; //-1 движется обратно, 0 стоит на месте, 1 движется вперед
  int curLimit = MOTOR_DEFAULT_CURRENT_LIMIT; //ток остановки\

  //для PID-регулятора
  float compute(float value);
  void resetPID();
  float KP = 1.0f; //1.0f лучшее
  float KI = 0.00007f; //0.00007f лучшее
  float KD = 5.0f; //5.0f лучшее
  float P;
  float I;
  float D;
  float U;
  float Ureal;
  float error;
  float errorPrev;
  float errorSum;
  };

Thumb::Thumb(int num, int maxRange, int encoderPin, int speedPin, void ISR(), int currentLimit, int maximumSpeed, bool isInverted, int pinA, int pinB){
  id = num;
  pwmPin = speedPin;
  maxSpeed = maximumSpeed;
  inverted = isInverted;
  curLimit = currentLimit;
  isrPin = encoderPin;
  pA = pinA;
  pB = pinB;
  attachInterrupt(digitalPinToInterrupt(isrPin), ISR, FALLING);
  }

float Thumb::compute(float value){
  error = targetPosition - value;
  P = KP * error;
  errorSum += error;
  I = KI * errorSum;
  D = KD * (error - errorPrev);
  errorPrev = error;
  U = P+I+D;
  if(abs(U)>maxSpeed){
    Ureal = maxSpeed * (U/abs(U));
    }else{
    Ureal = U;  
    }
  return Ureal;
  }

void Thumb::resetPID(){
  error = 0;
  errorSum = 0;
  errorPrev = 0;
  }

int Thumb::reset(){
  backward(maxSpeed/2);
  delay(400);
  stop();
  position = 0;
  }

int Thumb::getPosition(){
  return position;
  }

int Thumb::toPosition(int pos, int velocity){
  resetPID();
  maxSpeed = velocity;
  targetPosition = map(pos, 0, 255, 0, range);
  handle();
  return pos;
}

int Thumb::toAbsolutePosition(int pos, int velocity){
  resetPID();
  maxSpeed = velocity;
  targetPosition = pos;
  handle();
  return pos;
}

int Thumb::handle(){
  int curSpeed = int(compute(position));
  speed = curSpeed;
  if(curSpeed > 0){
    forward(curSpeed);
  }else if(curSpeed < 0){
    backward(curSpeed);
  }
  return 0;
}

void Thumb::ISR(){
  if(DEBUG_ISR){
    Serial.printf("ISR ID = %d, ISR Pin = %d\n", id, isrPin);
  }
  if(direction==-1){
    position--;
    }else if(direction==1){
    position++;
    }
  }

int Thumb::forward(int spd){
  direction = 1;
  if(!inverted){
    digitalWrite(pA, HIGH);
    digitalWrite(pB, LOW);
    }
  else{
    digitalWrite(pA, LOW);
    digitalWrite(pB, HIGH);
    }
  analogWrite(pwmPin, abs(spd));
  return 0;
  }

int Thumb::backward(int spd){
  direction = -1;
  if(!inverted){
    digitalWrite(pA, LOW);
    digitalWrite(pB, HIGH);
    }
  else{
    digitalWrite(pA, HIGH);
    digitalWrite(pB, LOW);
    }
  analogWrite(pwmPin, abs(spd));
  return 0;
  }

int Thumb::stop(){
  direction = 0;
  analogWrite(pwmPin, 0);
  speed = 0;
  digitalWrite(pA, LOW);
  digitalWrite(pB, LOW);
  return 0;
  }

int Thumb::getSpeed(){
  return speed;
  }

void Thumb::setPID(float Kp, float Ki, float Kd){
  KP = Kp;
  KI = Ki;
  KD = Kd;
  }
