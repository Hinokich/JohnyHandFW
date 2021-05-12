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
  void ISR();
  int reset();
  int forward();
  int backward();
  int stop();
  int halt();  
  int handle();
  void pushParcel();
  
  private:
  int pwmPin;
  int position = 0;
  int speed = 255;
  int targetPosition = 0;
  bool inverted = false;
  int id = 0;
  int range = MOTOR_DEFAULT_RANGE;
  int maxSpeed = 255;
  int direction = 0; //-1 движется обратно, 0 стоит на месте, 1 движется вперед
  int curLimit = MOTOR_DEFAULT_CURRENT_LIMIT;
  byte forwardMask[4] = {0b01000000, 0b00010000, 0b00000100, 0b00000001}; //bitwise OR
  byte backwardMask[4] = {0b10000000, 0b00100000, 0b00001000, 0b00000010}; //bitwise OR
  byte clearMask[4] = {0b00111111, 0b11001111, 0b11110011, 0b11111100}; //bitwise AND
  //сброс маски при помощи clearMask останавливает мотор, установка обеих масок движения переводит мотор в удержание
  };

Motor::Motor(int num, int maxRange, int encoderPin, int speedPin, void ISR(), int currentLimit, int maximumSpeed, bool isInverted){
  id = num;
  pwmPin = speedPin;
  maxSpeed = maximumSpeed;
  speed = maxSpeed;
  inverted = isInverted;
  curLimit = currentLimit;
  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR, FALLING);
  }

int Motor::toPosition(int pos, int velocity){
  speed = velocity;
  targetPosition = pos;
  analogWrite(pwmPin, speed);
  int delta = targetPosition - position;
  if(delta>0){
    direction = 1;
    }else{
    direction = -1;
  }
  handle();
  return pos;
}

int Motor::handle(){
  int delta = abs(targetPosition - position);
  if(direction != 0){
    if(delta>0){
    if(direction == 1){
      forward();
      }else if(direction == -1){
      backward();
      }
    }else{
    stop();
    }
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

int Motor::forward(){
  byte _temp = muxParcel & clearMask[id];
  if(!inverted)
    muxParcel = _temp | forwardMask[id];
  else
    muxParcel = _temp | backwardMask[id];
  analogWrite(pwmPin, speed);
  pushParcel();
  return 0;
  }

int Motor::backward(){
  byte _temp = muxParcel & clearMask[id];
  if(!inverted)
    muxParcel = _temp | backwardMask[id];
  else
    muxParcel = _temp | forwardMask[id];
  analogWrite(pwmPin, speed);
  pushParcel();
  return 0;
  }

int Motor::stop(){
  byte _temp = muxParcel & clearMask[id];
  muxParcel = _temp;
  analogWrite(pwmPin, 0);
  pushParcel();
  return 0;
  }

int Motor::halt(){
  byte _temp = muxParcel & clearMask[id];
  muxParcel = (_temp | backwardMask[id])|forwardMask[id];
  analogWrite(pwmPin, 255);
  pushParcel();
  return 0;
  }

void Motor::pushParcel(){
  Wire.beginTransmission(muxAdr);
  Wire.write(muxParcel);
  Wire.endTransmission();
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
