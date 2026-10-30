#ifndef QTR_SENSOR_H
#define QTR_SENSOR_H

#include <Arduino.h>
#include <EEPROM.h>

#define QTR_USE_GRAY 1
#if QTR_USE_GRAY
#define QTR_ORDER {0, 1, 3, 2, 6, 7, 5, 4}
#else
#define QTR_ORDER {0, 1, 2, 3, 4, 5, 6, 7}
#endif

#define QTR_SETTLE_US 80
#define QTR_DUMMY 1
#define QTR_SAMPLES 2
#define QTR_MIN_RANGE 200

#define QTR_EEPROM_SIZE 64
#define QTR_EEPROM_ADDR 0
#define QTR_EEPROM_MAGIC 0x51A8

class MyQTR8C {
  public:
    MyQTR8C(const uint8_t *selPins, uint8_t sigPin, bool invert = false);
    void begin();
    void resetCalibration();
    void calibrate();
    bool channelOk(uint8_t i) const;
    void saveCalibration();
    bool loadCalibration();
    unsigned int readLine(uint16_t *sensorValues);

  private:
    uint8_t _sel[3];
    uint8_t _sig;
    bool _invert;
    uint16_t _min[8], _max[8];
    uint16_t _last;
    uint8_t _cur;
    const uint8_t GRAY[8] = QTR_ORDER;

    void selectChannel(uint8_t ch);
    uint16_t readRaw(uint8_t ch);
    uint16_t normalize(uint8_t i, uint16_t raw);
};

#define REVERSE_SENSOR 0
#define INVERT_COLOR true
#define SETPOINT 3500
#define SENSOR_THRESHOLD 500

extern MyQTR8C qtr;
extern uint16_t sensorValues[8];
extern uint8_t currentPattern;
extern unsigned int lastLinePos;

int docSensor();

#endif // QTR_SENSOR_H
