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
  int getSpeed();
  void ISR();
  int reset();
  int forward(int spd=255);
  int backward(int spd=255);
  int stop(); 
  int handle();
  void pushParcel();
  void setPID(float Kp, float Ki, float Kd);
  int range = MOTOR_DEFAULT_RANGE; //диапазон работы в тактах энкодера
  
  private:
  int pwmPin; //пин для ШИМ управления драйвером
  int isrPin; //пин для прерываний энкодера
  int position = 0; //текущая позиция
  int speed = 255; //текущая скорость
  int maxSpeed = 255; //макс скорость
  int targetPosition = 0; //целевая позиция
  bool inverted = false; //инвертирсия управления 
  int id = 0; //номер мотора в кисти  
  int direction = 0; //-1 движется обратно, 0 стоит на месте, 1 движется вперед
  int curLimit = MOTOR_DEFAULT_CURRENT_LIMIT; //ток остановки
  byte forwardMask[4] = {0b01000000, 0b00010000, 0b00000100, 0b00000001}; //побитовое ИЛИ
  byte backwardMask[4] = {0b10000000, 0b00100000, 0b00001000, 0b00000010}; //побитовое ИЛИ
  byte clearMask[4] = {0b00111111, 0b11001111, 0b11110011, 0b11111100}; //побитовое И
  //сброс маски при помощи clearMask останавливает мотор, установка обеих масок движения переводит мотор в удержание

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

Motor::Motor(int num, int maxRange, int encoderPin, int speedPin, void ISR(), int currentLimit, int maximumSpeed, bool isInverted){
  id = num;
  pwmPin = speedPin;
  maxSpeed = maximumSpeed;
  inverted = isInverted;
  curLimit = currentLimit;
  isrPin = encoderPin;
  attachInterrupt(digitalPinToInterrupt(isrPin), ISR, FALLING);
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
  resetPID();
  maxSpeed = velocity;
  targetPosition = pos;
  handle();
  return pos;
}

int Motor::handle(){
  int curSpeed = int(compute(position));
  speed = curSpeed;
  int delta = abs(targetPosition - position);
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
  if(DEBUG_ISR){
    Serial.printf("ISR ID = %d, ISR Pin = %d\n", id, isrPin);
  }
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
  return speed;
  }

void Motor::setPID(float Kp, float Ki, float Kd){
  KP = Kp;
  KI = Ki;
  KD = Kd;
  }

Motor motor0(0, 320, ENC_0, PWM_0, ISR_0, 600, 255, false);
Motor motor1(1, 320, ENC_1, PWM_1, ISR_1, 600, 255, true);
Motor motor2(2, 320, ENC_2, PWM_2, ISR_2, 600, 255, false);
Motor motor3(3, 320, ENC_3, PWM_3, ISR_3, 600, 255, true);

void stopAllMotors(){
  motor0.stop();
  motor1.stop();
  motor2.stop();
  motor3.stop();
  }

void calculatePosition(){
  positionAbsolute[0] = motor0.getPosition();
  positionAbsolute[1] = motor0.getPosition();
  positionAbsolute[2] = motor0.getPosition();
  positionAbsolute[3] = motor0.getPosition();
  positionRelative[0] = map(positionAbsolute[0], 0, motor0.range, 0, 255);
  positionRelative[1] = map(positionAbsolute[0], 0, motor0.range, 0, 255);
  positionRelative[2] = map(positionAbsolute[0], 0, motor0.range, 0, 255);
  positionRelative[3] = map(positionAbsolute[0], 0, motor0.range, 0, 255);
  }

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
