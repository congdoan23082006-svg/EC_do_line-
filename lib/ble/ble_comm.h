#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <Arduino.h>

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define CAL_TIME 5000
#define STATUS_INTERVAL 150
#define DEBUG_RAW 0

extern bool deviceConnected;
extern bool oldDeviceConnected;
extern int speed_dk;
extern bool calibrated;
extern bool calibrating;
extern unsigned long calEnd;
extern const char *currentModeName;
extern volatile uint64_t raceStartTime;
extern volatile bool raceRunning;

#if DEBUG_RAW
extern bool testMode;
extern unsigned long lastRawTime;
#endif

void ble_init();
void ble_check_connection();
void bleSend(const char *msg);
void sendStatus();
void handleBLECommand(String cmd);

#endif // BLE_COMM_H
