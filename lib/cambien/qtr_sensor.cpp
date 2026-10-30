#include "qtr_sensor.h"

const uint8_t S_PINS[] = {18, 19, 23}; // S0, S1, S2
const uint8_t SIG_PIN = 32;

MyQTR8C qtr(S_PINS, SIG_PIN, INVERT_COLOR);
uint16_t sensorValues[8];
uint8_t currentPattern = 0;
unsigned int lastLinePos = 3500;

MyQTR8C::MyQTR8C(const uint8_t *selPins, uint8_t sigPin, bool invert)
    : _sig(sigPin), _invert(invert), _last(3500), _cur(0) {
    for (int i = 0; i < 3; i++)
        _sel[i] = selPins[i];
    resetCalibration();
}

void MyQTR8C::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(_sel[i], OUTPUT);
        digitalWrite(_sel[i], LOW);
    }
    pinMode(_sig, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(_sig, ADC_11db);
    _cur = 0;
    EEPROM.begin(QTR_EEPROM_SIZE);
}

void MyQTR8C::resetCalibration() {
    for (int i = 0; i < 8; i++) {
        _min[i] = 4095;
        _max[i] = 0;
    }
}

void MyQTR8C::calibrate() {
    for (int i = 0; i < 8; i++) {
        uint16_t v = readRaw(GRAY[i]);
        if (v < _min[i])
            _min[i] = v;
        if (v > _max[i])
            _max[i] = v;
    }
}

bool MyQTR8C::channelOk(uint8_t i) const {
    return (_max[i] > _min[i]) && ((_max[i] - _min[i]) >= QTR_MIN_RANGE);
}

void MyQTR8C::saveCalibration() {
    EEPROM.put(QTR_EEPROM_ADDR, (uint16_t)QTR_EEPROM_MAGIC);
    EEPROM.put(QTR_EEPROM_ADDR + 2, _min);
    EEPROM.put(QTR_EEPROM_ADDR + 2 + sizeof(_min), _max);
    EEPROM.commit();
}

bool MyQTR8C::loadCalibration() {
    uint16_t magic = 0;
    EEPROM.get(QTR_EEPROM_ADDR, magic);
    if (magic != QTR_EEPROM_MAGIC)
        return false;
    EEPROM.get(QTR_EEPROM_ADDR + 2, _min);
    EEPROM.get(QTR_EEPROM_ADDR + 2 + sizeof(_min), _max);
    for (int i = 0; i < 8; i++) {
        if (!channelOk(i))
            return false;
    }
    return true;
}

void MyQTR8C::selectChannel(uint8_t ch) {
    uint8_t diff = ch ^ _cur;
    for (int b = 0; b < 3; b++)
        if (diff & (1 << b))
            digitalWrite(_sel[b], (ch >> b) & 1);
    _cur = ch;
}

uint16_t MyQTR8C::readRaw(uint8_t ch) {
    selectChannel(ch);
    delayMicroseconds(QTR_SETTLE_US);
    for (int d = 0; d < QTR_DUMMY; d++)
        analogRead(_sig);
    uint32_t acc = 0;
    for (int k = 0; k < QTR_SAMPLES; k++)
        acc += analogRead(_sig);
    return acc / QTR_SAMPLES;
}

uint16_t MyQTR8C::normalize(uint8_t i, uint16_t raw) {
    if (!channelOk(i))
        return 0;
    uint16_t range = _max[i] - _min[i];
    if (range < QTR_MIN_RANGE)
        return 0;
    long v = ((long)(raw - min(raw, _min[i])) * 1000) / range;
    v = constrain(v, 0, 1000);
    return _invert ? 1000 - v : v;
}

unsigned int MyQTR8C::readLine(uint16_t *sensorValues) {
    uint32_t avg = 0;
    uint32_t sum = 0;
    bool onLine = false;

    for (int i = 0; i < 8; i++) {
        uint16_t val = normalize(i, readRaw(GRAY[i]));
        sensorValues[i] = val;
        if (val > 200)
            onLine = true;
        if (val > 50) {
            avg += (uint32_t)val * (i * 1000);
            sum += val;
        }
    }

    if (!onLine) {
        if (_last < 3500)
            return 0;
        else
            return 7000;
    }

    if (sum == 0)
        return _last;
    _last = avg / sum;
    return _last;
}

int docSensor() {
    unsigned int pos = qtr.readLine(sensorValues);
#if REVERSE_SENSOR
    lastLinePos = 7000 - pos;
#else
    lastLinePos = pos;
#endif
    currentPattern = 0;
    int black = 0;
    for (int i = 0; i < 8; i++) {
        if (sensorValues[i] > SENSOR_THRESHOLD) {
#if REVERSE_SENSOR
            currentPattern |= (1 << i);
#else
            currentPattern |= (1 << (7 - i));
#endif
            black++;
        }
    }
    return black;
}
