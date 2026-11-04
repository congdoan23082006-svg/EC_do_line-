#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// Cấu hình dải mắt cảm biến sử dụng (Đầy đủ 16 mắt: từ mắt 0 đến 15)
#define SENSOR_START_IDX   0      // Bắt đầu từ mắt 0
#define SENSOR_END_IDX     15     // Kết thúc ở mắt 15

// Trọng số vị trí đối xứng cho 16 mắt (Tâm vạch ở giữa mắt 7 và 8: từ -75 đến +75)
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 8 mắt bên trái (0..7)
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f  // 8 mắt bên phải (8..15)
};

// Cấu hình PID Dòng Chảy Liên Tục (Êm Ru, Mềm Mại, Không Xoay Khựng Bánh):
// Kp=1.65, Ki=0.0, Kd=3.20, baseSpeed=125, minSpeed=20, maxSpeed=220
static PID_Config_t pidConfig = {
    .Kp = 1.65f,
    .Ki = 0.0f,
    .Kd = 3.20f,
    .baseSpeed = 125,
    .maxSpeed = 220,
    .minSpeed = 20    // Bánh trong luôn giữ tốc độ tiến >= 20, cả 2 bánh cùng lăn mượt mà, KHÔNG khựng xoay tại chỗ
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static float smoothedDTerm = 0.0f;  // Bộ lọc Low-Pass làm mượt D-Term chống giật lắc
static unsigned long lastComputeTime = 0;
static unsigned long lostLineStartTime = 0;
static bool isLineCurrentlyLost = false;
static int8_t lastKnownDirection = 0; // -1: Trái, +1: Phải
static float lastKnownPosition = 0.0f; // Bộ nhớ vị trí vạch cuối cùng

#define DASHED_LINE_TIMEOUT_MS  60    // Thời gian lướt qua nét đứt

static inline int16_t clamp_val(int16_t v, int16_t min_v, int16_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = baseSpeed;
    lastError = 0.0f;
    integralError = 0.0f;
    smoothedDTerm = 0.0f;
    lastComputeTime = millis();
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    isLineCurrentlyLost = false;
    lastKnownPosition = 0.0f;
    lastKnownDirection = 0;
}

void pid_set_tunings(float kp, float ki, float kd) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}

void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = clamp_val(speed, 0, 255);
}

void pid_set_auto_mode(bool enable) {
    autoMode = enable;
    isLineCurrentlyLost = false;
    lastKnownPosition = 0.0f;
    lastKnownDirection = 0;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
    } else {
        integralError = 0.0f;
        lastError = 0.0f;
        smoothedDTerm = 0.0f;
        lastComputeTime = millis();
    }
}

bool pid_is_auto_mode(void) {
    return autoMode;
}

bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    float sumWeights = 0.0f;
    uint32_t sumCalibrated = 0;
    uint8_t count = 0;

    // Quét toàn bộ 16 mắt bằng giá trị chuẩn hóa Analog 0 - 1000
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        uint16_t cal = sensor_get_calibrated(i);

        // Lọc nhiễu nền: chỉ lấy tín hiệu thực sự có vạch đen (cal >= 150)
        if (cal >= 150) {
            sumCalibrated += cal;
            sumWeights += (float)cal * SENSOR_WEIGHTS[i];
            if (cal >= 500) {
                count++;
            }
        }
    }

    *outBlackCount = count;

    // Nếu tổng độ đen trên thanh cảm biến >= 300 (tức là có vạch line)
    if (sumCalibrated >= 300) {
        float pos = sumWeights / (float)sumCalibrated; // Trọng tâm liên tục (Smooth Centroid)
        *outPosition = pos;

        // Ghi nhớ vị trí và hướng vạch cuối cùng (Line Memory)
        lastKnownPosition = pos;
        if (pos < -15.0f) {
            lastKnownDirection = -1; // Vạch ở bên trái
        } else if (pos > 15.0f) {
            lastKnownDirection = 1;  // Vạch ở bên phải
        } else {
            lastKnownDirection = 0;  // Vạch đang ở chính giữa tâm
        }
        return true;
    }

    // Mất vạch hoàn toàn (cả 16 mắt đều trắng tinh) -> Giữ vị trí cuối cùng
    *outPosition = lastKnownPosition;
    return false;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady) {
        return LINE_CASE_STOPPED;
    }

    uint16_t mask = sensor_get_line_bits();
    uint8_t totalBlack = __builtin_popcount(mask & 0b1111111111111111);

    // 1. VẠCH NGANG / NGÃ TƯ / VẠCH ĐÍCH (Từ 11 mắt đen trở lên cùng lúc)
    if (totalBlack >= 11) {
        return LINE_CASE_CROSS_LINE;
    }

    // 2. MẤT LINE HOÀN TOÀN (Cả 16 mắt đều trắng: blackCount == 0)
    if (blackCount == 0) {
        if (!isLineCurrentlyLost) {
            isLineCurrentlyLost = true;
            lostLineStartTime = millis();
        }

        unsigned long lostElapsed = millis() - lostLineStartTime;

        // Nếu trước đó đang bám thẳng ở giữa tâm trong thời gian ngắn (<= 60ms) -> Nét đứt
        if (lastKnownDirection == 0 && fabsf(lastKnownPosition) < 15.0f && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        // Mất line -> Kích hoạt tìm lại line êm ái
        return LINE_CASE_LOST_LINE;
    }

    // Đã nhìn thấy vạch đen -> Reset cờ mất line
    isLineCurrentlyLost = false;

    // 3. TOÀN BỘ CÁC TRƯỜNG HỢP CÒN LẠI (Đường thẳng, cua nhẹ, cua vừa, cua gắt, cua chữ S, cua 90 độ):
    // -> ĐỀU ĐƯỢC ĐIỀU KHIỂN BẰNG THUẬT TOÁN PID DÒNG CHẢY LIÊN TỤC (CONTINUOUS FLOW SMOOTH PID)
    // -> BÁNH XE LUÔN LĂN TIẾN MƯỢT MÀ, KHÔNG BAO GIỜ DỪNG KHỰNG HAY XOAY XOAY TẠI CHỖ!
    return LINE_CASE_NORMAL_PID;
}

void pid_line_follow_step(void) {
    if (!autoMode) {
        return;
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);

    // Nhận diện Case hiện tại
    currentCase = pid_detect_case(position, blackCount);

    unsigned long now = millis();
    float dt = (float)(now - lastComputeTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    lastComputeTime = now;

    // =========================================================================
    // XỬ LÝ TỪNG CASE TRƯỜNG HỢP CỤ THỂ
    // =========================================================================
    switch (currentCase) {

        // ---------------------------------------------------------------------
        // CASE 1: BÁM LINE CHUẨN PID DÒNG CHẢY LIÊN TỤC (MỀM MẠI, ÊM RU, LƯỚT NHANH)
        // ---------------------------------------------------------------------
        case LINE_CASE_NORMAL_PID: {
            float error = position; // Sai số liên tục (-75.0 đến +75.0)
            float absErr = fabsf(error);

            // 1. Khâu P mềm mại, mượt mà (Smooth Proportional):
            // Giữ lái cực êm ở dải tâm và tăng lực lái vừa phải ở góc cua sâu
            float pTerm = pidConfig.Kp * error;
            if (absErr > 20.0f) {
                float extra = (absErr - 20.0f) * 0.40f;
                pTerm += (error > 0.0f) ? extra : -extra;
            }

            // 2. Khâu vi phân D giảm chấn thủy lực (Low-Pass Filter):
            // Dập tắt hoàn toàn rung lắc và hiện tượng lắc đuôi cá, thân xe êm ru
            float rawDelta = error - lastError;
            lastError = error;
            smoothedDTerm = smoothedDTerm * 0.65f + (pidConfig.Kd * rawDelta) * 0.35f;
            float dTerm = smoothedDTerm;

            // 3. Khâu tích phân I (Anti-windup: kẹp trong khoảng [-80, 80])
            integralError += error * dt;
            if (integralError > 80.0f) integralError = 80.0f;
            if (integralError < -80.0f) integralError = -80.0f;
            float iTerm = pidConfig.Ki * integralError;

            // Tổng bù lái
            lastCorrection = pTerm + iTerm + dTerm;

            // 4. Giảm ga ôm cua nhẹ nhàng (Tối đa chỉ giảm 20 PWM để duy trì trớn lướt, KHÔNG BỊ CHẬM):
            float dragFactor = absErr / 75.0f;
            if (dragFactor > 1.0f) dragFactor = 1.0f;
            int16_t currentBase = pidConfig.baseSpeed - (int16_t)(dragFactor * 20.0f);

            int16_t leftSpeed  = currentBase + (int16_t)lastCorrection;
            int16_t rightSpeed = currentBase - (int16_t)lastCorrection;

            // 5. NGUYÊN TẮC VÀNG ĐỂ XE CHẠY ÊM RU, KHÔNG BAO GIỜ BỊ XOAY BÁNH TẠI CHỖ:
            // Bánh trong luôn giữ tốc độ TIẾN tối thiểu (>= 20) làm điểm tựa lăn êm ái
            // Cả hai bánh luôn quay tiến, xe uốn lượn mềm mại qua khúc cua với tốc độ cao!
            leftSpeed  = clamp_val(leftSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
            rightSpeed = clamp_val(rightSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 4: MẤT LINE (16 mắt đều trắng) -> Tìm lại line êm ái, KHÔNG xoay tít mù
        // ---------------------------------------------------------------------
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Nếu mất line quá 1.5 giây mà không thấy -> Dừng xe để an toàn
            if (lostElapsed > 1500) {
                motor_stop();
                break;
            }

            // Xe lăn bánh êm ái hướng đầu xe về phía vạch cuối cùng vừa thấy
            if (lastKnownDirection < 0) {
                motor_set_speeds(20, 105); // Bánh trái lăn nhẹ, bánh phải đẩy xe về lại vạch
            } else if (lastKnownDirection > 0) {
                motor_set_speeds(105, 20); // Bánh phải lăn nhẹ, bánh trái đẩy xe về lại vạch
            } else {
                motor_set_speeds(70, 70);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 5: VẠCH NGANG / NGÃ TƯ / VẠCH ĐÍCH
        // ---------------------------------------------------------------------
        case LINE_CASE_CROSS_LINE: {
            // Giữ hướng chạy thẳng lướt qua giao lộ với tốc độ cơ bản
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 9: ĐOẠN NÉT ĐỨT (DASHED LINE)
        // ---------------------------------------------------------------------
        case LINE_CASE_DASHED_LINE: {
            // Xe duy trì chạy thẳng với tốc độ an toàn để lướt qua khoảng trắng
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 8: DỪNG XE HOẶC CHƯA HIỆU CHUẨN
        // ---------------------------------------------------------------------
        case LINE_CASE_STOPPED:
        default: {
            motor_stop();
            break;
        }
    }
}

LineCase_t pid_get_current_case(void) {
    return currentCase;
}

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID: return "CASE 1: Bam Line PID";
        case LINE_CASE_SHARP_LEFT: return "CASE 2: Cua Gat Trai (90 deg)";
        case LINE_CASE_SHARP_RIGHT: return "CASE 3: Cua Gat Phai (90 deg)";
        case LINE_CASE_LOST_LINE: return "CASE 4: Mat Line (Tim Lai)";
        case LINE_CASE_CROSS_LINE: return "CASE 5: Vach Ngang / Nga Tu";
        case LINE_CASE_T_LEFT: return "CASE 6: Nga 3 Trai";
        case LINE_CASE_T_RIGHT: return "CASE 7: Nga 3 Phai";
        case LINE_CASE_STOPPED: return "CASE 8: Dung Xe";
        case LINE_CASE_DASHED_LINE: return "CASE 9: Doan Net Dut (Dashed Line)";
        default: return "Khong Xac Dinh";
    }
}

float pid_get_last_error(void) {
    return lastError;
}

float pid_get_correction(void) {
    return lastCorrection;
}

float pid_get_kp(void) {
    return pidConfig.Kp;
}

float pid_get_ki(void) {
    return pidConfig.Ki;
}

float pid_get_kd(void) {
    return pidConfig.Kd;
}

int16_t pid_get_base_speed(void) {
    return pidConfig.baseSpeed;
}
