#pragma once
#include <Arduino.h>
class MotorCHR {
public:
  MotorCHR(int C1_pin, int C2_pin, int DIR_pin, int PWM_pin, int wheel_ticks, bool is_reversed);
  void setVelocity(float new_velocity);
  float getVelocity();
  float calculateVelocity();
  float PID(float current_velocity, float Pk, float Ik, float Dk);
  void driverControl(float PWM_signal);
  void counter();
  int C1_pin, C2_pin, DIR_pin, PWM_pin, wheel_ticks;
  float current_velocity = 0;
  float goal_velocity = 0;
  int current_ticks = 0;
  int prev_ticks = 0;
  float current_error;
  float prev_error = 0;
  unsigned long timer = 0;
  float P = 0;
  float I = 0;
  float D = 0;
  float Pk = 0;
  float Ik = 0;
  float Dk = 0;
  float PWM_signal;
  bool is_reversed = false;
};
