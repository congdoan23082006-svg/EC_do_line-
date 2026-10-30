#include <Arduino.h>
#include "motor.h"
#include "qtr_sensor.h"
#include "pid_control.h"
#include "ble_comm.h"

void setup() {
    Serial.begin(115200);
    
    tb6612_init();
    ble_init();
    qtr.begin();

    if (qtr.loadCalibration()) {
        calibrated = true;
        Serial.println("Da nap calib tu EEPROM");
    }
    
    Serial.println("READY! Gui CAL de calibrate, sau do AUTO de chay");
    bleSend("LOG:ROBOT_READY");
}

void loop() {
    ble_check_connection();

    // Xử lý quá trình Calibrate cảm biến
    if (calibrating) {
        qtr.calibrate();
        if (millis() >= calEnd) {
            calibrating = false;
            calibrated  = true;
            currentModeName = "IDLE";
            currentCaseName = "-";
            
            char bad[48] = "LOG:CAL_WARN mat:";
            bool anyBad = false;
            for (int c = 0; c < 8; c++) {
                if (!qtr.channelOk(c)) {
                    char t[4];
                    snprintf(t, sizeof(t), " %d", c + 1);
                    strncat(bad, t, sizeof(bad) - strlen(bad) - 1);
                    anyBad = true;
                }
            }
            if (anyBad) bleSend(bad);
            
            qtr.saveCalibration();
            bleSend("LOG:CAL_DONE");
        }
    }

    // Chế độ chạy tự động hoặc đọc cảm biến chờ
    if (runLine) {
        lineFollowPID();
    } else if (calibrated && !calibrating) {
        docSensor();
    } else {
        currentPattern = 0;
    }

#if DEBUG_RAW
    if (testMode && !calibrating && !runLine) {
        docSensor();
        if (deviceConnected && (millis() - lastRawTime > 200)) {
            lastRawTime = millis();
            char raw[64];
            snprintf(raw, sizeof(raw), "RAW:%u,%u,%u,%u,%u,%u,%u,%u",
                sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
                sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);
            bleSend(raw);
        }
    }
#endif

    // Gửi trạng thái định kỳ qua BLE
    sendStatus();
}