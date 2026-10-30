#include "pid_control.h"
#include "ble_comm.h"
#include "motor.h"
#include "qtr_sensor.h"

// ================================================================
// PID & THAM SỐ CHUẨN TỪ BLE
// ================================================================
float KP = 0.04f;
float KD = 0.4f;
float KI = 0.00005f;
int BASE = 150;

int SPEED_MIN = 110;
int E_SLOW = 1800;

int TURN_OUT = 60;
int TURN_IN = 60;
int TURN_BASE = 120;

const int BOOST = 20;
const int BOOST_THRESHOLD = 150;
float integralError = 0.0f;

int lastError = 0;
bool runLine = false;
const char *currentCaseName = "-";

// Xử lý nét đứt & vạch kết thúc
#define GAP_STRAIGHT_MS 100
#define GAP_MAX_COUNT 5
#define GAP_ERR_MAX 1500
#define END_BLACK_COUNT 1
#define END_CONFIRM 3

static int cnt = 0;
static int cntDen = 0;
static bool inGap = false;
static bool inFullBlack = false;
static int fullBlackFrames = 0;
static unsigned long gapStart = 0;

void movePID(int base, int adjust) {
  int L = base + adjust, R = base - adjust;
  if (adjust > BOOST_THRESHOLD)
    L += BOOST;
  if (adjust < -BOOST_THRESHOLD)
    R += BOOST;
  move(L, R);
}

void endLine() {
  uint64_t finishTime = esp_timer_get_time();
  tb6612_short_brake();
  delay(150);
  tb6612_coast();
  runLine = false;
  integralError = 0.0f;
  currentModeName = "IDLE";
  currentCaseName = "END_LINE";

  if (raceRunning) {
    raceRunning = false;
    float elapsedSeconds = (float)(finishTime - raceStartTime) / 1000000.0f;
    char lapMsg[64];
    snprintf(lapMsg, sizeof(lapMsg), "LOG:RACE_FINISH:%.3f", elapsedSeconds);
    bleSend(lapMsg);
  } else {
    bleSend("LOG:END_LINE");
  }
}

void lineFollowPID() {
  int blackCount = docSensor();

  if (currentPattern == 0b11111111) {
    fullBlackFrames++;
    if (fullBlackFrames >= END_CONFIRM && !inFullBlack) {
      inFullBlack = true;
      cntDen++;
      if (cntDen >= END_BLACK_COUNT) {
        endLine();
        return;
      }
    }
  } else {
    fullBlackFrames = 0;
    inFullBlack = false;
  }
  if (blackCount != 0)
    inGap = false;

  // Cua góc vuông 90 độ (sử dụng TURN_BASE)
  if ((currentPattern & 0b00000001) && blackCount < 5) {
    currentCaseName = "TURN_RIGHT";
    move(TURN_BASE + TURN_OUT, TURN_BASE - TURN_IN);
    integralError = 0.0f;
    return;
  }
  if ((currentPattern & 0b10000000) && blackCount < 5) {
    currentCaseName = "TURN_LEFT";
    move(TURN_BASE - TURN_IN, TURN_BASE + TURN_OUT);
    integralError = 0.0f;
    return;
  }

  // Mất line
  if (blackCount == 0) {
    if (!inGap && cnt < GAP_MAX_COUNT && abs(lastError) <= GAP_ERR_MAX) {
      inGap = true;
      gapStart = millis();
      cnt++;
    }
    if (inGap && millis() - gapStart < GAP_STRAIGHT_MS) {
      currentCaseName = "GAP_STRAIGHT";
      move(BASE, BASE);
      return;
    }
    currentCaseName = "LOST_LINE";
    if (lastError > 0)
      move(BASE + BOOST, -BASE);
    else
      move(-BASE, BASE + BOOST);
    integralError = 0.0f;
    return;
  }

  // Bám line PID kết hợp Tốc độ theo đường (PROF)
  currentCaseName = "PID_FOLLOW";
  int error = (int)lastLinePos - SETPOINT;

  // Tích phân sai số có Anti-windup
  integralError += (float)error;
  integralError = constrain(integralError, -20000.0f, 20000.0f);

  int adjust = (int)(error * KP) + (int)(integralError * KI) +
               (int)(KD * (error - lastError));
  lastError = error;

  // Tính tốc độ theo sai số đường (PROF)
  int currentBase = BASE;
  if (E_SLOW > 0) {
    int errAbs = abs(error);
    if (errAbs > E_SLOW)
      errAbs = E_SLOW;
    currentBase = BASE - ((BASE - SPEED_MIN) * errAbs) / E_SLOW;
  }

  movePID(currentBase, adjust);
}
