#include "Motion.h"
#include "Sensor.h" // Chứa mảng sensor và sensorValue

// Khởi tạo các biến
int servoPwm = 0;
int vitri = 0;
int baseSpeed = 200;  // Dải PWM 0-255
volatile int calPID = 0;
volatile long lastPos = 0;
volatile float Kp = 3.0;
volatile float Ki = 0;
volatile float Kd = 13;
volatile int isLeft = 0;
volatile int isRight = 0;
volatile long posPID = 0;

void motion_init() {
  // Cài đặt các chân IN làm OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Mặc định cho xe đứng im
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Cài đặt băm xung PWM cho chân ENA và ENB (8-bit để dùng 0-255 dễ hơn)
  ledcSetup(ENA_CH, 1000, 8);
  ledcAttachPin(ENA, ENA_CH);

  ledcSetup(ENB_CH, 1000, 8);
  ledcAttachPin(ENB, ENB_CH);

  speed_run(0, 0);
}

// --------------------------------------------------------
// HÀM ĐIỀU KHIỂN CHIỀU VÀ TỐC ĐỘ CHUẨN L298N
// --------------------------------------------------------
void speed_run(int speedDC_left, int speedDC_right) {
  // 1. Giới hạn dải tốc độ không vượt quá 255
  if (speedDC_left < -255) speedDC_left = -255;
  if (speedDC_right < -255) speedDC_right = -255;
  if (speedDC_left > 255) speedDC_left = 255;
  if (speedDC_right > 255) speedDC_right = 255;

  // 2. Điều khiển bánh TRÁI (Motor A)
  if (speedDC_left == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);   // Phanh động cơ
    ledcWrite(ENA_CH, 0);
  } 
  else if (speedDC_left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);   // Chạy tới
    ledcWrite(ENA_CH, speedDC_left);
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  // Chạy lùi (khi phanh gắt)
    ledcWrite(ENA_CH, -speedDC_left); // Lấy số dương của tốc độ
  }

  // 3. Điều khiển bánh PHẢI (Motor B)
  if (speedDC_right == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);   // Phanh động cơ
    ledcWrite(ENB_CH, 0);
  } 
  else if (speedDC_right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);   // Chạy tới
    ledcWrite(ENB_CH, speedDC_right);
  } 
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);  // Chạy lùi (khi phanh gắt)
    ledcWrite(ENB_CH, -speedDC_right); // Lấy số dương của tốc độ
  }
}

// ========================================================
// CÁC HÀM BÊN DƯỚI GIỮ NGUYÊN 100% LOGIC CỦA BẠN
// ========================================================

void handleAndSpeed(int angle, int speed1) {
  int speedLeft  = speed1 + angle;
  int speedRight = speed1 - angle;

  // Co tỉ lệ 2 bánh xuống nếu vượt giới hạn 255
  // Cách này giữ nguyên tỉ lệ speedLeft:speedRight → xe vẫn rẽ đúng góc
  int maxVal = max(abs(speedLeft), abs(speedRight));
  if (maxVal > 255) {
    speedLeft  = (int)((long)speedLeft  * 255 / maxVal);
    speedRight = (int)((long)speedRight * 255 / maxVal);
  }

  speed_run(speedLeft, speedRight);
}

void PID() {
  if (calPID == 1) {
    // ============================================================
    // Tính vị trí line từ sensor byte nhị phân (ĐEN = 1)
    // Sensor vật lý j nằm ở bit (7-j) trong byte `sensor`
    // Trọng số vị trí: j * 3000 → dải 0 (trái) đến 21000 (phải)
    // Tâm line = 10500
    // ============================================================
    long weightedSum = 0;
    long activeCount = 0;

    for (int j = 0; j < 8; j++) {
      if (bitRead(sensor, 7 - j)) {
        weightedSum += (long)j * 30;  // dải 0→210, tâm = 105
        activeCount++;
      }
    }

    long i;
    if (activeCount == 0) {
      i = lastPos;
    } else {
      long position = weightedSum / activeCount; // 0..210
      i = position - 105;                        // lỗi: -105..+105
    }

    posPID = i;

    // Anti-windup
    static long sumError = 0;
    if (abs(i) < 30) sumError += i;
    if (sumError >  5000) sumError =  5000;
    if (sumError < -5000) sumError = -5000;

    long iP = (long)(Kp * i);
    long iI = (long)(Ki * sumError);
    long iD = (long)(Kd * (lastPos - i));
    long iRet = iP + iI - iD;

    // không cần chia 5 nữa vì error đã vừ tầm (±105 × Kp ≈ ±210)
    servoPwm = (int)iRet;
    lastPos = i;
    calPID = 0;
  }
}

void runforwardline(int tocdo) {
  PID();

  switch (sensor) {
    case 0b00011000:
    case 0b00111100:
      handleAndSpeed(servoPwm, tocdo);
      vitri = 0;
      break;

    case 0b00011100:
    case 0b00001000:
      handleAndSpeed(servoPwm, tocdo);
      vitri = 1;
      break;

    case 0b00010000:
    case 0b00111000:
      if (vitri >= 7) { speed_run(255, 0); break; }
      else { handleAndSpeed(servoPwm, tocdo); }
      vitri = -1;
      break;

    case 0b00001100:
    case 0b00011110:
      if (vitri < -7) { speed_run(0, 255); break; }
      else { handleAndSpeed(servoPwm, baseSpeed); vitri = 2; }
      break;

    case 0b00000100:
    case 0b00001110:
      if (vitri < -7) { speed_run(0, 255); break; }
      else { handleAndSpeed(servoPwm, (baseSpeed * 70 / 100)); vitri = 3; }
      break;

    case 0b00110000:
    case 0b01111000:
      if (vitri >= 7) { speed_run(255, 0); break; }
      else { handleAndSpeed(servoPwm, baseSpeed); vitri = -2; }
      break;

    case 0b00100000:
    case 0b01110000:
      if (vitri >= 7) { speed_run(255, 0); break; }
      else { handleAndSpeed(servoPwm, (baseSpeed * 70 / 100)); vitri = -3; }
      break;

    case 0b00000110:
    case 0b00001111:
    case 0b00000010:
    case 0b00000111:
    case 0b00000011:
    case 0b00000001:
    case 0b00011111:  // 5 sensor xa phải
    case 0b00111111:  // 6 sensor xa phải
    case 0b01111111:  // 7 sensor xa phải
      if (vitri < -7) { speed_run(0, 255); break; }
      else { handleAndSpeed(servoPwm, (baseSpeed * 50 / 100)); vitri = 7; }
      break;

    case 0b01100000:
    case 0b11110000:
    case 0b01000000:
    case 0b11100000:
    case 0b11000000:
    case 0b10000000:
    case 0b11111000:  // 5 sensor xa trái
    case 0b11111100:  // 6 sensor xa trái
    case 0b11111110:  // 7 sensor xa trái
      if (vitri >= 7) { speed_run(255, 0); break; }
      else { handleAndSpeed(servoPwm, (baseSpeed * 50 / 100)); vitri = -7; }
      break;

    case 0b00000000:
      if (vitri <= -7) { speed_run(0, 255); vitri = -8; }
      else if (vitri >= 7) { speed_run(255, 0); vitri = 8; }
      else speed_run(0, 0);
      break;

    default:
      // Pattern không nhận ra (nhiễu/transition): giữ nguyên lái cũ, không phanh đột ngột
      handleAndSpeed(servoPwm, tocdo);
      break;
  }
}