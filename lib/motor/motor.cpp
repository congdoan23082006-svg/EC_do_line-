#include "motor.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_RES)
#define PWM_WRITE(pin, ch, d) ledcWrite(pin, d)
#else
#define PWM_ATTACH(pin, ch)                                                    \
    do {                                                                       \
        ledcSetup(ch, PWM_FREQ, PWM_RES);                                      \
        ledcAttachPin(pin, ch);                                                \
    } while (0)
#define PWM_WRITE(pin, ch, d) ledcWrite(ch, d)
#endif

void tb6612_coast() {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    PWM_WRITE(L_PWM, L_PWM_CH, 0);
    PWM_WRITE(R_PWM, R_PWM_CH, 0);
}

void tb6612_short_brake() {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, HIGH);
    PWM_WRITE(L_PWM, L_PWM_CH, 255);
    PWM_WRITE(R_PWM, R_PWM_CH, 255);
}

void tb6612_init() {
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    PWM_ATTACH(L_PWM, L_PWM_CH);
    PWM_ATTACH(R_PWM, R_PWM_CH);
#if TB6612_STBY >= 0
    pinMode(TB6612_STBY, OUTPUT);
    digitalWrite(TB6612_STBY, HIGH);
#endif
    tb6612_coast();
}

static void driveOne(int in1, int in2, int pin, int ch, int spd, bool rev) {
    spd = constrain(spd, -255, 255);
    if (rev)
        spd = -spd;
    if (spd > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (spd < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    PWM_WRITE(pin, ch, abs(spd));
}

void speed_run(int left, int right) {
    driveOne(L_IN1, L_IN2, L_PWM, L_PWM_CH, left, REVERSE_LEFT);
    driveOne(R_IN1, R_IN2, R_PWM, R_PWM_CH, right, REVERSE_RIGHT);
}

void move(int left, int right) {
    speed_run(constrain(left, -255, 255), constrain(right, -255, 255));
}

void stopMotor() {
    tb6612_coast();
}
