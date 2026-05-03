#include "Sensor.h"

// Khởi tạo các biến toàn cục
unsigned int sensorValue[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
volatile unsigned char sensor;
int mode = 2; // Khởi động lên mặc định là Mode 2

// Khởi tạo các chân GPIO và EEPROM
void sensor_init() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  EEPROM.begin(EEPROM_SIZE);
  readEeprom();
  
  beep(100); 
}

void setMuxChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
}

void beep(int timer) {
  digitalWrite(BUZZER, 1);
  delay(timer);
  digitalWrite(BUZZER, 0);
}

void readEeprom() {
  for (int i = 0; i < 8; i++) {
    black_value[i] = (EEPROM.read(i * 2) << 8) | EEPROM.read((i * 2) + 1);
  }
  for (int i = 0; i < 8; i++) {
    white_value[i] = (EEPROM.read(16 + (2 * i)) << 8) | EEPROM.read(17 + (2 * i));
  }
  for (int i = 0; i < 8; i++) {
    compare_value[i] = ((black_value[i] + white_value[i]) / 2);
    if (compare_value[i] == 0) compare_value[i] = 2000; 
  }
}

bool isBtn1Clicked() {
  if (digitalRead(BUTTON1) == LOW) {
    delay(50);
      while (digitalRead(BUTTON1) == LOW) delay(1); 
      delay(50);
      return true;
    }
  
  return false;
}

void read_sensor_74hc4067() {
  unsigned char temp = 0;
  for (int j = 0; j < 8; j++) {
    setMuxChannel(j);
    delayMicroseconds(5); 
    sensorValue[j] = 4095 - analogRead(SIG_PIN);

    temp = temp << 1; 
    // Logic cảm biến infrared (sensorValue = 4095 - raw, nên ĐEN có giá trị CAO hơn):
    // - Khi phát hiện ĐEN  : sensorValue >= compare_value → bit = 1
    // - Khi phát hiện TRẮNG: sensorValue <  compare_value → bit = 0
    if (sensorValue[j] >= compare_value[j]) {
      temp |= 0x01;  // ĐEN = 1
    } else {
      temp &= 0xfe;  // TRẮNG = 0
    }
  }
  sensor = temp;
}