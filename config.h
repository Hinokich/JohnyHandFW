/* Configuration file
 * Most of the pins defined here
 * Pins initialization also defined here
 */
#pragma once
#include <FlexCAN_T4.h>
#include "Wire.h"

#define CURRENT_SENSOR_OFFSET 1650
#define CURRENT_SENSOR_RATIO 110

#define MOTOR_DEFAULT_RANGE 320
#define MOTOR_DEFAULT_CURRENT_LIMIT 600
#define MOTOR_MUX_ADDRESS 0x4c

//Encoder pins
#define ENC_0 12
#define ENC_1 11
#define ENC_2 23
#define ENC_3 9
#define ENC_4 10

//PWM pins
#define PWM_0 2
#define PWM_1 3
#define PWM_2 4
#define PWM_3 5
#define PWM_4 6

//Current sensor pins
#define AIN_0 A3
#define AIN_1 A2
#define AIN_2 A1
#define AIN_3 A0
#define AIN_4 A6

//Thumb motor driver pins
#define THUMB_0 21
#define THUMB_1 22

//Used interfaces
#define CAN_2
#define I2C_0 
#define UART2

#define I2C_SDA 18
#define I2C_SCL 19

#define CAN_BAUDRATE 250000

#ifdef CAN_1
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
#else
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
#endif

void initPins(){
  pinMode(ENC_0, INPUT_PULLUP);
  pinMode(ENC_1, INPUT_PULLUP);
  pinMode(ENC_2, INPUT_PULLUP);
  pinMode(ENC_3, INPUT_PULLUP);
  pinMode(ENC_4, INPUT_PULLUP);

  pinMode(PWM_0, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

  pinMode(AIN_0, INPUT);
  pinMode(AIN_1, INPUT);
  pinMode(AIN_2, INPUT);
  pinMode(AIN_3, INPUT);
  pinMode(AIN_4, INPUT);

  pinMode(THUMB_0, OUTPUT);
  pinMode(THUMB_1, OUTPUT);
  }

void initCAN(){
  can.begin();
  can.setBaudRate(CAN_BAUDRATE);
  }

void initUART(){
  
  }

void initI2C(){
  Wire.begin();
  }

  
