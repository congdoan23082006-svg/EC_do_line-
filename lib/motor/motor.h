#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// ================================================================
// ĐỘNG CƠ TB6612
// ================================================================
// IC TRÁI (AIN1 nối BIN1, AIN2 nối BIN2, PWMA nối PWMB)
#define L_PWM 25
#define L_IN1 26
#define L_IN2 27
// IC PHẢI
#define R_PWM 14
#define R_IN1 12 // GPIO12 là chân strapping
#define R_IN2 13
// Chân STBY chung (nối cứng lên 3.3V thì để -1)
#define TB6612_STBY -1

#define PWM_FREQ 2000
#define PWM_RES 8
#define L_PWM_CH 0
#define R_PWM_CH 1

#define REVERSE_LEFT 0
#define REVERSE_RIGHT 0

void tb6612_init();
void tb6612_coast();
void tb6612_short_brake();
void speed_run(int left, int right);
void move(int left, int right);
void stopMotor();

#endif // MOTOR_H
