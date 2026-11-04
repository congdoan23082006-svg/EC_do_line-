#include "sensor.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

// Định nghĩa các biến toàn cục
uint16_t sensorRaw[N_SENSORS];
uint16_t minValues[N_SENSORS];
uint16_t maxValues[N_SENSORS];
uint16_t thresholds[N_SENSORS];
bool blackIsHigher[N_SENSORS];
bool isCalibrating = false;
bool calibrationReady = false;

// Biến quản lý thời gian quét hiệu chuẩn
static unsigned long calibStartTime = 0;
static uint32_t calibDurationMs = 4000; // Mặc định lia trong 4 giây

// Biến nội bộ phục vụ chống rung nút nhấn
static int lastCalibReading = LOW;
static int calibButtonState = LOW;
static unsigned long lastCalibChangeTime = 0;

void sensor_init(void) {
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    pinMode(PIN_MUX_D, OUTPUT);
    pinMode(PIN_CALIB, INPUT);

    // GPIO36 trên ESP32 là chân input-only
    analogReadResolution(12); // Độ phân giải 12-bit (0 - 4095)

    // Khởi tạo giá trị ban đầu cho các mắt
    for (int i = 0; i < N_SENSORS; i++) {
        minValues[i] = 4095;
        maxValues[i] = 0;
        thresholds[i] = 2048;
        blackIsHigher[i] = true;
    }
}

void sensor_mux_select(uint8_t channel) {
    digitalWrite(PIN_MUX_A, channel & 0x01);
    digitalWrite(PIN_MUX_B, (channel >> 1) & 0x01);
    digitalWrite(PIN_MUX_C, (channel >> 2) & 0x01);
    digitalWrite(PIN_MUX_D, (channel >> 3) & 0x01);
}

uint16_t sensor_read_mux(uint8_t channel) {
    sensor_mux_select(channel);
    delayMicroseconds(6); // Đợi MUX chốt kênh ổn định (tối ưu từ 20us -> 6us)

    analogRead(PIN_LINE_SIG); // Bỏ mẫu đầu tiên để xả điện dung ký sinh

    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(PIN_LINE_SIG);
        delayMicroseconds(3); // Tối ưu từ 10us -> 3us
    }

    return (uint16_t)(sum / ADC_SAMPLES);
}

void sensor_read_all(void) {
    // Chỉ đọc các mắt cần dùng (từ mắt 1 đến 14)
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        sensorRaw[i] = sensor_read_mux(i);
    }
}

// Bắt đầu chế độ lia cảm biến qua lại
void sensor_start_sweep_calibration(uint32_t duration_ms) {
    isCalibrating = true;
    calibrationReady = false;
    calibDurationMs = duration_ms;
    calibStartTime = millis();

    // Reset lại min/max của các mắt 1..14
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }

    printf("\n======================================================\n");
    printf("[CALIB] BAT DAU LIA CAM BIEN QUA LAI TREN VACH LINE!\n");
    printf(">> Hay lia thanh cam bien qua lai trong %u giay...\n", (unsigned int)(duration_ms / 1000));
    printf("======================================================\n");
}

// Cập nhật Min/Max liên tục khi đang lia (gọi trong loop)
void sensor_update_calibration_loop(void) {
    if (!isCalibrating) return;

    // Đọc toàn bộ mắt
    sensor_read_all();

    // Cập nhật giá trị cực tiểu (Min) và cực đại (Max) cho từng mắt
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        uint16_t val = sensorRaw[i];
        if (val < minValues[i]) minValues[i] = val;
        if (val > maxValues[i]) maxValues[i] = val;
    }

    // Kiểm tra nếu hết thời gian lia
    if (millis() - calibStartTime >= calibDurationMs) {
        sensor_finish_sweep_calibration();
    }
}

// Chốt kết thúc quá trình lia và tính ngưỡng trung bình (Min + Max) / 2
void sensor_finish_sweep_calibration(void) {
    isCalibrating = false;
    bool valid = true;

    printf("\n=== KET QUA HIEU CHUAN (LIA CAM BIEN QUA LAI) ===\n");
    printf("Mat  |  Min ADC  |  Max ADC  | Delta ADC | Nguong TB\n");
    printf("--------------------------------------------------\n");

    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        uint16_t diff = maxValues[i] - minValues[i];

        if (diff < MIN_CALIBRATION_DELTA) {
            printf(" %2d  |   %4u    |   %4u    |   %4u    | CHUA QUA VACH!\n",
                   i, minValues[i], maxValues[i], diff);
            valid = false;
            continue;
        }

        // Tính ngưỡng trung bình của từng mắt
        thresholds[i] = (minValues[i] + maxValues[i]) / 2;
        blackIsHigher[i] = true; // Giá trị cao hơn ngưỡng là vạch đen

        printf(" %2d  |   %4u    |   %4u    |   %4u    |   %4u (OK)\n",
               i, minValues[i], maxValues[i], diff, thresholds[i]);
    }
    printf("--------------------------------------------------\n");


    if (valid) {
        calibrationReady = true;
        printf("[CALIB THANH CONG] Da tinh xong nguong trung binh tung mat! (0 = Trang, 1 = Den)\n\n");
    } else {
        printf("[CALIB CHUA DAT] Mot so mat chua lia qua vach den. Hay bam CALIB va lia lai!\n\n");
    }
}

bool sensor_is_calibrating(void) {
    return isCalibrating;
}

bool sensor_is_button_pressed(void) {
    int reading = digitalRead(PIN_CALIB);
    if (reading != lastCalibReading) {
        lastCalibChangeTime = millis();
    }

    bool pressed = false;
    if (millis() - lastCalibChangeTime > BUTTON_DEBOUNCE_MS &&
        reading != calibButtonState) {
        calibButtonState = reading;
        pressed = (calibButtonState == HIGH);
    }
    lastCalibReading = reading;
    return pressed;
}

void sensor_handle_calibration_step(void) {
    if (!isCalibrating) {
        // Bắt đầu lia trong 4 giây
        sensor_start_sweep_calibration(4000);
    } else {
        // Nếu đang lia mà bấm nút lần nữa -> Chốt kết thúc sớm
        sensor_finish_sweep_calibration();
    }
}

uint16_t sensor_get_calibrated(uint8_t index) {
    if (index < SENSOR_START_IDX || index > SENSOR_END_IDX) return 0;
    uint16_t minVal = minValues[index];
    uint16_t maxVal = maxValues[index];
    if (maxVal <= minVal + MIN_CALIBRATION_DELTA) return 0;

    uint16_t raw = sensorRaw[index];
    if (raw <= minVal) raw = minVal;
    if (raw >= maxVal) raw = maxVal;

    // Hỗ trợ cực tính: Nếu blackIsHigher là true thì ADC cao là đen (0 -> 1000)
    // Nếu false thì ADC thấp là đen (đảo ngược lại để 1000 luôn luôn đại diện cho vạch đen)
    if (blackIsHigher[index]) {
        return (uint16_t)(((uint32_t)(raw - minVal) * 1000UL) / (maxVal - minVal));
    } else {
        return (uint16_t)(((uint32_t)(maxVal - raw) * 1000UL) / (maxVal - minVal));
    }
}

void sensor_set_polarity(bool black_is_higher) {
    for (int i = 0; i < N_SENSORS; i++) {
        blackIsHigher[i] = black_is_higher;
    }
    printf("[SENSOR] Da doi cuc tinh: %s\n", black_is_higher ? "DEN_ADC_CAO" : "DEN_ADC_THAP");
}

bool sensor_get_polarity(void) {
    return blackIsHigher[SENSOR_START_IDX];
}

bool sensor_is_black(uint8_t index) {
    // Bỏ qua mắt 0 và 15
    if (index < SENSOR_START_IDX || index > SENSOR_END_IDX) return false;

    // Ngưỡng 500/1000 tương đương với giá trị trung bình chính xác (min + max) / 2
    return (sensor_get_calibrated(index) >= 500);
}

uint16_t sensor_get_line_bits(void) {
    uint16_t bits = 0;
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        if (sensor_is_black(i)) {
            bits |= (1 << i);
        }
    }
    return bits;
}

void sensor_print_states(void) {
    for (int i = 0; i < N_SENSORS; i++) {
        if (i < SENSOR_START_IDX || i > SENSOR_END_IDX) {
            printf("- "); // Mắt không dùng
        } else {
            printf("%c ", sensor_is_black(i) ? '1' : '0');
        }
    }
    printf("\n");
}
