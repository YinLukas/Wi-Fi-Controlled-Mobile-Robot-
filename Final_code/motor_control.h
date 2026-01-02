#pragma once
#include <Arduino.h>

struct MotorPins {
  int pinPWM;
  int pinF;
  int pinB;
  int pinA;
  int pinB_enc;
};

extern MotorPins FL_pins;
extern MotorPins FR_pins;
extern MotorPins RL_pins;
extern MotorPins RR_pins;

extern bool usePID; 

extern volatile long L_encoderCount, R_encoderCount, L2_encoderCount, R2_encoderCount;
extern float targetL, targetR, targetL2, targetR2;
extern float rpmL, rpmR, rpmL2, rpmR2;

void initMotors();
void updateMotorControl();

// ⭐ 修复点：添加这些声明
void setTargetRPM(float fl, float fr, float rl, float rr);
void brakeAll();
void releaseBrake();