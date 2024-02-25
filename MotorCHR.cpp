#pragma once
#include <MotorCHR.h>

MotorCHR::MotorCHR(int C1_pin, int C2_pin, int DIR_pin, int PWM_pin, int wheel_ticks, bool is_reversed) {
  this->C1_pin = C1_pin;
  this->C2_pin = C2_pin;
  this->DIR_pin = DIR_pin;
  this->PWM_pin = PWM_pin;
  this->wheel_ticks = wheel_ticks;
  this->is_reversed = is_reversed;
}

float MotorCHR::PID(float current_velocity, float Pk, float Ik, float Dk) {
  current_error = goal_velocity - current_velocity;
  P = current_error * Pk;
  I += (current_error * (50) / 1000) * Ik;
  D = (((current_error - prev_error) / (50)) / 1000) * Dk;
  if (I > 255) {
    I = 255;
  } else if (I < -255) {
    I = -255;
  }
  PWM_signal = P + I + D;
  if (PWM_signal > 255) {
    PWM_signal = 255;
  } else if (PWM_signal < -255) {
    PWM_signal = -255;
  }
  prev_error = current_error;
  return PWM_signal;
}

void MotorCHR::driverControl(float PWM_signal) {
  if (PWM_signal >= 0) {
    if (is_reversed) {
      digitalWrite(DIR_pin, LOW);
    } else {
      digitalWrite(DIR_pin, HIGH);
    }
  } else {
    if (is_reversed) {
      digitalWrite(DIR_pin, HIGH);
    } else {
      digitalWrite(DIR_pin, LOW);
    }
  }
  analogWrite(PWM_pin, abs(PWM_signal));
}

void MotorCHR::setVelocity(float new_velocity) {
  goal_velocity = new_velocity;
  driverControl(PID(calculateVelocity(), Pk, Ik, Dk));
}

float MotorCHR::getVelocity() {
  return current_velocity;
}

float MotorCHR::calculateVelocity() {
  if (millis() - timer >= 50) {
    current_velocity = ((float)(current_ticks - prev_ticks) / (float)((millis() - timer) * wheel_ticks)) * 6.28 * 1000;
    timer = millis();
    prev_ticks = current_ticks;
  }
  return current_velocity;
}

void MotorCHR::counter() {
  if (digitalRead(C1_pin) == digitalRead(C2_pin)) {
    if (is_reversed) {
      current_ticks--;
    } else {
      current_ticks++;
    }
  } else {
    if (is_reversed) {
      current_ticks++;
    } else {
      current_ticks--;
    }
  }
}
