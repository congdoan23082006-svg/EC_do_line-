#include "pid_control.h"
#include "ble_comm.h"
#include "motor.h"
#include "qtr_sensor.h"

// ================================================================
// PID & THAM SỐ CHUẨN TỪ BLE (CHẾ ĐỘ TEST AN TOÀN - BASE 120)
// ================================================================
float KP = 0.04f;
float KD = 0.45f;
float KI = 0.00005f;
int BASE = 120;

int SPEED_MIN = 90;
int E_SLOW = 1500;

int TURN_OUT = 50;
int TURN_IN = 145;
int TURN_BASE = 95;

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
#define START_BLIND_MS 1500 // 1.5s đầu tiên bỏ qua vạch đích để thoát khỏi vạch xuất phát

static int cnt = 0;
static int cntDen = 0;
static bool inGap = false;
static bool inFullBlack = false;
static int fullBlackFrames = 0;
static unsigned long gapStart = 0;

// Trạng thái khóa cua vuông 90 độ (State Machine không chặn)
// 0: Không cua
// 1: Bắt đầu xoay, chờ tâm cảm biến (S4, S5) thoát khỏi vạch cũ
// 2: Tiếp tục xoay, chờ tâm cảm biến (S4 hoặc S5) chạm vào vạch mới
static uint8_t turnRightPhase = 0;
static uint8_t turnLeftPhase = 0;
static unsigned long turnStartTime = 0;
#define TURN_90_TIMEOUT 450 // Giới hạn tối đa 450ms bảo hiểm chống kẹt

void resetTurnState() {
  turnRightPhase = 0;
  turnLeftPhase = 0;
  cnt = 0;
  inGap = false;
}

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
  resetTurnState();
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

  if (!runLine) {
    resetTurnState();
    return;
  }

  // ================================================================
  // 1. MÁY TRẠNG THÁI CUA VUÔNG 90 ĐỘ (STATE MACHINE)
  // ================================================================
  if (turnRightPhase > 0) {
    currentCaseName = "TURN_RIGHT_90";
    move(TURN_BASE + TURN_OUT, TURN_BASE - TURN_IN);

    // Bảo hiểm kẹt thời gian
    if (millis() - turnStartTime > TURN_90_TIMEOUT) {
      turnRightPhase = 0;
      tb6612_short_brake();
      delay(10);
      integralError = 0.0f;
      lastError = 0;
      return;
    }

    if (turnRightPhase == 1) {
      // Chờ tâm xe (S4, S5) thoát khỏi vạch cũ (hoặc sau 45ms xoay)
      if (!(currentPattern & 0b00011000) || (millis() - turnStartTime > 45)) {
        turnRightPhase = 2;
      }
    } else if (turnRightPhase == 2) {
      // Chờ tâm xe (S4 hoặc S5) chạm vào vạch mới!
      if (currentPattern & 0b00011000) {
        turnRightPhase = 0;
        tb6612_short_brake(); // Phanh hãm dập quán tính xoay
        delay(15);
        integralError = 0.0f;
        lastError = 0;
        return;
      }
    }
    return;
  }

  if (turnLeftPhase > 0) {
    currentCaseName = "TURN_LEFT_90";
    move(TURN_BASE - TURN_IN, TURN_BASE + TURN_OUT);

    // Bảo hiểm kẹt thời gian
    if (millis() - turnStartTime > TURN_90_TIMEOUT) {
      turnLeftPhase = 0;
      tb6612_short_brake();
      delay(10);
      integralError = 0.0f;
      lastError = 0;
      return;
    }

    if (turnLeftPhase == 1) {
      // Chờ tâm xe (S4, S5) thoát khỏi vạch cũ (hoặc sau 45ms xoay)
      if (!(currentPattern & 0b00011000) || (millis() - turnStartTime > 45)) {
        turnLeftPhase = 2;
      }
    } else if (turnLeftPhase == 2) {
      // Chờ tâm xe (S4 hoặc S5) chạm vào vạch mới!
      if (currentPattern & 0b00011000) {
        turnLeftPhase = 0;
        tb6612_short_brake(); // Phanh hãm dập quán tính xoay
        delay(15);
        integralError = 0.0f;
        lastError = 0;
        return;
      }
    }
    return;
  }

  // ================================================================
  // 2. VẠCH ĐÍCH (TOÀN BỘ 8 MẮT ĐỀU ĐEN)
  // Bỏ qua trong START_BLIND_MS đầu để xe thoát khỏi vạch xuất phát
  // ================================================================
  bool canDetectFinish = (raceStartTime == 0) || ((esp_timer_get_time() - raceStartTime) > ((uint64_t)START_BLIND_MS * 1000ULL));
  if (currentPattern == 0b11111111 && canDetectFinish) {
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

  // ================================================================
  // 3. NHẬN DIỆN GÓC VUÔNG 90 ĐỘ (2 HOẶC 3 MẮT BIÊN CHỐNG SÓT LINE)
  // ================================================================
  // Cua phải: Ít nhất 2 mắt S7, S8 cùng đen VÀ S1, S2 trắng VÀ không phải ngã tư/vạch đích
  if (((currentPattern & 0b00000011) == 0b00000011) &&
      ((currentPattern & 0b11000000) == 0) && (blackCount <= 6)) {
    turnRightPhase = 1;
    turnStartTime = millis();
    currentCaseName = "TURN_RIGHT_90";
    move(TURN_BASE + TURN_OUT, TURN_BASE - TURN_IN);
    integralError = 0.0f;
    return;
  }

  // Cua trái: Ít nhất 2 mắt S1, S2 cùng đen VÀ S7, S8 trắng VÀ không phải ngã tư/vạch đích
  if (((currentPattern & 0b11000000) == 0b11000000) &&
      ((currentPattern & 0b00000011) == 0) && (blackCount <= 6)) {
    turnLeftPhase = 1;
    turnStartTime = millis();
    currentCaseName = "TURN_LEFT_90";
    move(TURN_BASE - TURN_IN, TURN_BASE + TURN_OUT);
    integralError = 0.0f;
    return;
  }

  // ================================================================
  // 4. MẤT LINE & NÉT ĐỨT
  // ================================================================
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

  // ================================================================
  // 5. BÁM LINE PID KẾT HỢP PROFILE TỐC ĐỘ (PROF)
  // ================================================================
  currentCaseName = "PID_FOLLOW";
  int error = (int)lastLinePos - SETPOINT;

  // Khi xe bám vạch ổn định ở giữa, reset bộ đếm nét đứt cho chặng tiếp theo
  if (blackCount >= 2 && abs(error) < 600) {
    cnt = 0;
  }

  // Triệt tiêu trễ tích phân (Anti-windup): reset khi đổi hướng cua hoặc xe vào giữa tâm
  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0) || abs(error) < 60) {
    integralError = 0.0f;
  } else if (abs(error) < 1200) {
    integralError += (float)error;
    integralError = constrain(integralError, -10000.0f, 10000.0f);
  }

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
