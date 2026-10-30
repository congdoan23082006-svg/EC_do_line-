#include "ble_comm.h"
#include "motor.h"
#include "qtr_sensor.h"
#include "pid_control.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;

bool deviceConnected = false;
bool oldDeviceConnected = false;
int speed_dk = 120;
bool calibrated = false;
bool calibrating = false;
unsigned long calEnd = 0;

const char *currentModeName = "IDLE";
volatile uint64_t raceStartTime = 0;
volatile bool raceRunning = false;

static unsigned long lastStatusTime = 0;

#if DEBUG_RAW
bool testMode = false;
unsigned long lastRawTime = 0;
#endif

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String v = pChar->getValue().c_str();
        if (v.length() > 0)
            handleBLECommand(v);
    }
};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *p) override {
        deviceConnected = true;
        bleSend("LOG:BLE_CONNECTED");
    }
    void onDisconnect(BLEServer *p) override {
        deviceConnected = false;
        stopMotor();
        runLine = false;
        raceRunning = false;
        integralError = 0.0f;
        BLEDevice::startAdvertising();
    }
};

void ble_init() {
    BLEDevice::init("ROBOT_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pSvc = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRx = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE |
                                    BLECharacteristic::PROPERTY_WRITE_NR);
    pRx->setCallbacks(new MyCallbacks());
    pSvc->start();
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising... ROBOT_BLE");
}

void ble_check_connection() {
    if (!deviceConnected && oldDeviceConnected) {
        delay(300);
        BLEDevice::startAdvertising();
        oldDeviceConnected = false;
    }
    if (deviceConnected && !oldDeviceConnected)
        oldDeviceConnected = true;
}

void bleSend(const char *msg) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue((uint8_t *)msg, strlen(msg));
        pTxCharacteristic->notify();
        delay(5);
    }
    Serial.println(msg);
}

void sendStatus() {
    if (!deviceConnected)
        return;
    if (millis() - lastStatusTime < STATUS_INTERVAL)
        return;
    lastStatusTime = millis();

    char patStr[9];
    for (int i = 7; i >= 0; i--)
        patStr[7 - i] = (currentPattern & (1 << i)) ? '1' : '0';
    patStr[8] = '\0';

    float curLapTime = 0.0f;
    if (raceRunning) {
        curLapTime = (float)(esp_timer_get_time() - raceStartTime) / 1000000.0f;
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"%s\",\"case\":\"%s\","
             "\"pos\":%u,\"pat\":\"%s\","
             "\"spd\":%d,\"err\":%d,\"cal\":%d,\"time\":%.2f}",
             currentModeName, currentCaseName, lastLinePos,
             patStr, speed_dk, lastError,
             calibrating ? 2 : (calibrated ? 1 : 0), curLapTime);
    pTxCharacteristic->setValue((uint8_t *)buf, strlen(buf));
    pTxCharacteristic->notify();
}

void handleBLECommand(String cmd) {
    cmd.trim();

    if (cmd.startsWith("SPD:")) {
        speed_dk = constrain(cmd.substring(4).toInt(), 30, 255);
        char buf[30];
        snprintf(buf, sizeof(buf), "LOG:SPD=%d", speed_dk);
        bleSend(buf);
        return;
    }

#if DEBUG_RAW
    if (cmd == "TEST") {
        testMode = !testMode;
        runLine = false;
        stopMotor();
        bleSend(testMode ? "LOG:TEST_ON" : "LOG:TEST_OFF");
        return;
    }
#endif

    if (cmd == "CAL") {
        runLine = false;
        raceRunning = false;
        stopMotor();
        calibrated = false;
        calibrating = true;
        calEnd = millis() + CAL_TIME;
        qtr.resetCalibration();
        currentPattern = 0;
        integralError = 0.0f;
        currentModeName = "CALIB";
        currentCaseName = "CALIBRATING";
        bleSend("LOG:CAL_START");
        return;
    }

    // PID:kp,kd,ki,base
    if (cmd.startsWith("PID:")) {
        float kp, kd, ki = 0.0f;
        int base;
        if (sscanf(cmd.c_str() + 4, "%f,%f,%f,%d", &kp, &kd, &ki, &base) == 4) {
            KP = kp;
            KD = kd;
            KI = ki;
            BASE = constrain(base, 0, 255);
            integralError = 0.0f;
            bleSend("LOG:PID_OK");
        } else if (sscanf(cmd.c_str() + 4, "%f,%f,%d", &kp, &kd, &base) == 3) {
            KP = kp;
            KD = kd;
            BASE = constrain(base, 0, 255);
            integralError = 0.0f;
            bleSend("LOG:PID_OK");
        } else {
            bleSend("LOG:PID_ERR");
        }
        return;
    }

    // PROF:s_min,e_slow
    if (cmd.startsWith("PROF:")) {
        int s_min, e_slow;
        if (sscanf(cmd.c_str() + 5, "%d,%d", &s_min, &e_slow) == 2) {
            SPEED_MIN = constrain(s_min, 0, 255);
            E_SLOW = constrain(e_slow, 100, 10000);
            bleSend("LOG:PROF_OK");
        } else {
            bleSend("LOG:PROF_ERR");
        }
        return;
    }

    // TURN:out,in,base
    if (cmd.startsWith("TURN:")) {
        int o, i, b = TURN_BASE;
        if (sscanf(cmd.c_str() + 5, "%d,%d,%d", &o, &i, &b) == 3) {
            TURN_OUT = constrain(o, 0, 255);
            TURN_IN = constrain(i, -255, 400);
            TURN_BASE = constrain(b, 0, 255);
            bleSend("LOG:TURN_OK");
        } else if (sscanf(cmd.c_str() + 5, "%d,%d", &o, &i) == 2) {
            TURN_OUT = constrain(o, 0, 255);
            TURN_IN = constrain(i, -255, 400);
            bleSend("LOG:TURN_OK");
        } else {
            bleSend("LOG:TURN_ERR");
        }
        return;
    }

    // Đọc tất cả thông số
    if (cmd == "GETPID" || cmd == "GETALL" || cmd == "READ") {
        char buf[96];
        snprintf(buf, sizeof(buf), "PID:%.4f,%.4f,%.5f,%d", KP, KD, KI, BASE);
        bleSend(buf);
        snprintf(buf, sizeof(buf), "PROF:%d,%d", SPEED_MIN, E_SLOW);
        bleSend(buf);
        snprintf(buf, sizeof(buf), "TURN:%d,%d,%d", TURN_OUT, TURN_IN, TURN_BASE);
        bleSend(buf);
        return;
    }

    if (cmd == "AUTO") {
        if (!calibrated) {
            bleSend("LOG:NEED_CAL");
            return;
        }
        runLine = true;
        lastError = 0;
        integralError = 0.0f;
        lastLinePos = SETPOINT;
        currentModeName = "AUTO";
        currentCaseName = "PID_FOLLOW";
        raceStartTime = esp_timer_get_time();
        raceRunning = true;
        bleSend("LOG:AUTO_START");
        return;
    }

    if (cmd == "MANUAL") {
        runLine = false;
        raceRunning = false;
        stopMotor();
        integralError = 0.0f;
        currentModeName = "MANUAL";
        currentCaseName = "IDLE";
        bleSend("LOG:MODE_MANUAL");
        return;
    }

    if (!runLine) {
        if (cmd == "FWD")
            move(speed_dk, speed_dk);
        else if (cmd == "BWD")
            move(-speed_dk, -speed_dk);
        else if (cmd == "LFT")
            move(-speed_dk, speed_dk);
        else if (cmd == "RGT")
            move(speed_dk, -speed_dk);
        else if (cmd == "STP")
            stopMotor();
    }
}
