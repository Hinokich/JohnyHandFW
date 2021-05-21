#include "config.h"
#include <Wire.h>

void ISR_0();
void ISR_1();
void ISR_2();
void ISR_3();

byte muxParcel = 0b00000000;
byte muxAdr = MOTOR_MUX_ADDRESS;

class Motor{
  public:
  Motor(int num, int maxRange, int encoderPin, int speedPin, void ISR(), int currentLimit=MOTOR_DEFAULT_CURRENT_LIMIT, int maximumSpeed = 255, bool isInverted = false); //обычный мотор
  int toPosition(int pos, int velocity = 255);
  int getPosition();
  void ISR();
  int reset();
  int forward(int spd=255);
  int backward(int spd=255);
  int stop(); 
  int handle();
  void pushParcel();
  
  private:
  int pwmPin;
  int getSpeed(); //расчет скорости
  int position = 0;
  int speed = 255; //текущая скорость
  int maxSpeed = 255; //макс скорость
  int targetPosition = 0;
  bool inverted = false;
  int id = 0;
  int range = MOTOR_DEFAULT_RANGE;
  int direction = 0; //-1 движется обратно, 0 стоит на месте, 1 движется вперед
  int curLimit = MOTOR_DEFAULT_CURRENT_LIMIT;
  byte forwardMask[4] = {0b01000000, 0b00010000, 0b00000100, 0b00000001}; //bitwise OR
  byte backwardMask[4] = {0b10000000, 0b00100000, 0b00001000, 0b00000010}; //bitwise OR
  byte clearMask[4] = {0b00111111, 0b11001111, 0b11110011, 0b11111100}; //bitwise AND
  //сброс маски при помощи clearMask останавливает мотор, установка обеих масок движения переводит мотор в удержание

  //PID related
  float compute(float value);
  void resetPID();
  float KP = 1.0f; //1.0f best
  float KI = 0.00007f; //0.00007f best
  float KD = 5.0f; //5.0f best
  float P;
  float I;
  float D;
  float U;
  float Ureal;
  float error;
  float errorPrev;
  float errorSum;
  };

Motor::Motor(int num, int maxRange, int encoderPin, int speedPin, void ISR(), int currentLimit, int maximumSpeed, bool isInverted){
  id = num;
  pwmPin = speedPin;
  maxSpeed = maximumSpeed;
  inverted = isInverted;
  curLimit = currentLimit;
  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR, FALLING);
  }

float Motor::compute(float value){
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
  Serial.printf("%d %d %d %d\n", targetPosition, int(value), int(U), int(Ureal));
  return Ureal;
  }

void Motor::resetPID(){
  error = 0;
  errorSum = 0;
  errorPrev = 0;
  }

int Motor::getPosition(){
  return position;
  }

int Motor::toPosition(int pos, int velocity){
  maxSpeed = velocity;
  targetPosition = pos;
  handle();
  return pos;
}

int Motor::handle(){
  int curSpeed = int(compute(position));
  if(curSpeed > 0){
    forward(curSpeed);
  }else if(curSpeed < 0){
    backward(curSpeed);
  }
  return 0;
}

int Motor::reset(){
  return 0;
  }

void Motor::ISR(){
  if(direction==-1){
    position--;
    }else if(direction==1){
    position++;
    }
  }

int Motor::forward(int spd){
  direction = 1;
  byte _temp = muxParcel & clearMask[id];
  if(!inverted)
    muxParcel = _temp | forwardMask[id];
  else
    muxParcel = _temp | backwardMask[id];
  analogWrite(pwmPin, abs(spd));
  pushParcel();
  return 0;
  }

int Motor::backward(int spd){
  direction = -1;
  byte _temp = muxParcel & clearMask[id];
  if(!inverted)
    muxParcel = _temp | backwardMask[id];
  else
    muxParcel = _temp | forwardMask[id];
  analogWrite(pwmPin, abs(spd));
  pushParcel();
  return 0;
  }

int Motor::stop(){
  direction = 0;
  byte _temp = muxParcel & clearMask[id];
  muxParcel = _temp;
  analogWrite(pwmPin, 0);
  speed = 0;
  pushParcel();
  return 0;
  }

void Motor::pushParcel(){
  Wire.beginTransmission(muxAdr);
  Wire.write(muxParcel);
  Wire.endTransmission();
  }

int Motor::getSpeed(){
  return 0;
  }

Motor motor0(0, 320, ENC_0, PWM_0, ISR_0, 600, 255, false);
Motor motor1(1, 320, ENC_1, PWM_1, ISR_1, 600, 255, true);
Motor motor2(2, 320, ENC_2, PWM_2, ISR_2, 600, 255, false);
Motor motor3(3, 320, ENC_3, PWM_3, ISR_3, 600, 255, true);

void ISR_0(){
  motor0.ISR();
  }

void ISR_1(){
  motor1.ISR();
  }

void ISR_2(){
  motor2.ISR();
  }

void ISR_3(){
  motor3.ISR();
  }
