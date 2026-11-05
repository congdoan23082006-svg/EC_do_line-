#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// Cấu hình dải mắt cảm biến sử dụng (Đầy đủ 16 mắt: từ mắt 0 đến 15)
#define SENSOR_START_IDX   0      // Bắt đầu từ mắt 0
#define SENSOR_END_IDX     15     // Kết thúc ở mắt 15

// =============================================================================
// BẢNG TRỌNG SỐ VỊ TRÍ HÌNH HỌC CÁNH ÉN (SWEPT-BACK WING GEOMETRY):
// - 8 mắt giữa (4..11) thẳng hàng ở mũi trước: khoảng cách tâm hẹp, bám vạch siêu êm
// - 4 mắt cánh trái (0..3) & 4 mắt cánh phải (12..15) vát chéo dốc lùi về phía sau:
//   trọng số mở rộng lũy tiến (-128..-62 và +62..+128) để bẻ góc dứt khoát khi ôm cua 90°
// =============================================================================
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -128.0f, -104.0f, -82.0f, -62.0f, // 4 mắt Cánh Trái (0, 1, 2, 3) - vát chéo dốc sau
     -44.0f,  -30.0f, -18.0f,  -6.0f, // 4 mắt Nửa Trái hàng thẳng (4, 5, 6, 7)
       6.0f,   18.0f,  30.0f,  44.0f, // 4 mắt Nửa Phải hàng thẳng (8, 9, 10, 11)
      62.0f,   82.0f, 104.0f, 128.0f  // 4 mắt Cánh Phải (12, 13, 14, 15) - vát chéo dốc sau
};

// Cấu hình PID Tối Ưu Cho Bo Mạch Cánh Én & Cần Dài 9cm:
// Kp=1.35, Ki=0.0, Kd=2.80, baseSpeed=125, minSpeed=20, maxSpeed=220
static PID_Config_t pidConfig = {
    .Kp = 1.35f,
    .Ki = 0.0f,
    .Kd = 2.80f,
    .baseSpeed = 125,
    .maxSpeed = 220,
    .minSpeed = 20    // Bánh trong luôn giữ tốc độ lăn tiến >= 20, xe uốn cua êm ru không khựng xoay
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static float smoothedDTerm = 0.0f;  // Bộ lọc Low-Pass 70/30 dập chấn động học cánh én
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
        if (pos < -20.0f) {
            lastKnownDirection = -1; // Vạch ở bên trái (cánh trái)
        } else if (pos > 20.0f) {
            lastKnownDirection = 1;  // Vạch ở bên phải (cánh phải)
        } else {
            lastKnownDirection = 0;  // Vạch đang ở cụm giữa thẳng
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
    
    // Phân tích hình học Cánh Én:
    // Cánh trái (mắt 0..3): 0b0000000000001111
    // Giữa thẳng (mắt 4..11): 0b0000111111110000
    // Cánh phải (mắt 12..15): 0b1111000000000000
    uint8_t leftWingCount  = __builtin_popcount(mask & 0b0000000000001111);
    uint8_t centerCount    = __builtin_popcount(mask & 0b0000111111110000);
    uint8_t rightWingCount = __builtin_popcount(mask & 0b1111000000000000);
    uint8_t totalBlack     = leftWingCount + centerCount + rightWingCount;

    // 1. VẠCH NGANG / NGÃ TƯ / CHỮ T (Phù hợp 100% với bo mạch Cánh Én):
    // Khi lướt qua vạch ngang: hoặc cụm 8 mắt giữa đen đậm (>=5),
    // hoặc cả 2 cánh ngoài cùng đen (leftWing >= 2 && rightWing >= 2), hoặc tổng số mắt đen >= 8
    if (centerCount >= 5 || (leftWingCount >= 2 && rightWingCount >= 2) || totalBlack >= 8) {
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
        if (lastKnownDirection == 0 && fabsf(lastKnownPosition) < 20.0f && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        // Mất line -> Kích hoạt tìm lại line êm ái
        return LINE_CASE_LOST_LINE;
    }

    // Đã nhìn thấy vạch đen -> Reset cờ mất line
    isLineCurrentlyLost = false;

    // 3. TOÀN BỘ CÁC TRƯỜNG HỢP CÒN LẠI (Đường thẳng, cua nhẹ, cua chữ S, cua 90 độ):
    // -> ĐỀU ĐƯỢC ĐIỀU KHIỂN BẰNG THUẬT TOÁN PID DÒNG CHẢY LIÊN TỤC VỚI HỆ THỐNG TRỌNG SỐ CÁNH ÉN!
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
        // CASE 1: BÁM LINE CHUẨN PID DÒNG CHẢY HÌNH HỌC CÁNH ÉN
        // ---------------------------------------------------------------------
        case LINE_CASE_NORMAL_PID: {
            float error = position; // Sai số liên tục (-128.0 đến +128.0)
            float absErr = fabsf(error);

            // 1. Khâu P thích ứng hình học Cánh Én (Adaptive Progressive P):
            // - Dải 8 mắt giữa (|error| <= 35.0): Lái cực êm, xe chạy thẳng phẳng lì không lắc đầu.
            // - Khi tràn sang 4 mắt cánh én (|error| > 35.0): Tự động mớm thêm lực cua dứt khoát bo góc 90°.
            float pTerm = pidConfig.Kp * error;
            if (absErr > 35.0f) {
                float extra = (absErr - 35.0f) * 0.45f;
                pTerm += (error > 0.0f) ? extra : -extra;
            }

            // 2. Khâu vi phân D lọc thông thấp 70% / 30% dập tắt hoàn toàn rung lắc đòn bẩy 9cm:
            float rawDelta = error - lastError;
            lastError = error;
            smoothedDTerm = smoothedDTerm * 0.70f + (pidConfig.Kd * rawDelta) * 0.30f;
            float dTerm = smoothedDTerm;

            // 3. Khâu tích phân I (Anti-windup: kẹp trong khoảng [-80, 80])
            integralError += error * dt;
            if (integralError > 80.0f) integralError = 80.0f;
            if (integralError < -80.0f) integralError = -80.0f;
            float iTerm = pidConfig.Ki * integralError;

            // Tổng bù lái vi sai
            lastCorrection = pTerm + iTerm + dTerm;

            // 4. Giảm ga ôm cua thích ứng cánh én (Tối đa giảm 25 PWM ở góc cua cực đại 128.0 để ghìm trớn):
            float dragFactor = absErr / 128.0f;
            if (dragFactor > 1.0f) dragFactor = 1.0f;
            int16_t currentBase = pidConfig.baseSpeed - (int16_t)(dragFactor * 25.0f);

            int16_t leftSpeed  = currentBase + (int16_t)lastCorrection;
            int16_t rightSpeed = currentBase - (int16_t)lastCorrection;

            // 5. Thuật toán vi sai bám sàn cánh én:
            // - Khi cua thường (|error| <= 70): Bánh trong luôn giữ minSpeed = 20, cả hai bánh cùng lăn tiến êm ru.
            // - Khi cua vuông 90° gắt chạm cánh ngoài (|error| > 70): Bánh trong tự động thu hẹp về 5-10 PWM
            //   để làm tâm xoay hẹp (pivot tightening), giúp đầu xe 9cm uốn trọn góc cua mà không văng sàn!
            int16_t dynamicMin = pidConfig.minSpeed;
            if (absErr > 70.0f) {
                float wingTight = (absErr - 70.0f) / 58.0f;
                if (wingTight > 1.0f) wingTight = 1.0f;
                dynamicMin = (int16_t)(pidConfig.minSpeed - (wingTight * 15.0f)); // 20 -> 5
            }

            leftSpeed  = clamp_val(leftSpeed, dynamicMin, pidConfig.maxSpeed);
            rightSpeed = clamp_val(rightSpeed, dynamicMin, pidConfig.maxSpeed);

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 4: MẤT LINE (16 mắt đều trắng) -> Tìm lại line êm ái theo cánh én
        // ---------------------------------------------------------------------
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Nếu mất line quá 1.5 giây mà không thấy -> Dừng xe để an toàn
            if (lostElapsed > 1500) {
                motor_stop();
                break;
            }

            // Xe lăn bánh êm ái hướng đầu xe về phía cánh én vừa thấy vạch cuối cùng
            if (lastKnownDirection < 0) {
                motor_set_speeds(15, 120); // Bánh trái lăn nhẹ, bánh phải đẩy xe về lại vạch
            } else if (lastKnownDirection > 0) {
                motor_set_speeds(120, 15); // Bánh phải lăn nhẹ, bánh trái đẩy xe về lại vạch
            } else {
                motor_set_speeds(70, 70);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 5: VẠCH NGANG / NGÃ TƯ / CHỮ T
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
        case LINE_CASE_NORMAL_PID: return "CASE 1: Bam Line Canh En";
        case LINE_CASE_SHARP_LEFT: return "CASE 2: Cua Gat Trai (90 deg)";
        case LINE_CASE_SHARP_RIGHT: return "CASE 3: Cua Gat Phai (90 deg)";
        case LINE_CASE_LOST_LINE: return "CASE 4: Mat Line (Tim Lai)";
        case LINE_CASE_CROSS_LINE: return "CASE 5: Vach Ngang / Chu T";
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
