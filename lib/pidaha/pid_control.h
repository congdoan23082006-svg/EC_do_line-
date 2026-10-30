#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

// ================================================================
// CÁC THÔNG SỐ CHUẨN TỪ BLE LOG
// ================================================================
extern float KP;
extern float KD;
extern float KI;
extern int BASE;

extern int SPEED_MIN;
extern int E_SLOW;

extern int TURN_OUT;
extern int TURN_IN;
extern int TURN_BASE;

extern const int BOOST;
extern const int BOOST_THRESHOLD;
extern float integralError;

extern int lastError;
extern bool runLine;
extern const char *currentCaseName;

void movePID(int base, int adjust);
void lineFollowPID();
void endLine();

#endif // PID_CONTROL_H
