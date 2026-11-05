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
     128.0f,  104.0f,   82.0f,   62.0f, // 4 mắt Cánh Phải (0, 1, 2, 3) - ngoài cùng -> trong
      44.0f,   30.0f,   18.0f,    6.0f, // 4 mắt Nửa Phải hàng thẳng (4, 5, 6, 7)
      -6.0f,  -18.0f,  -30.0f,  -44.0f, // 4 mắt Nửa Trái hàng thẳng (8, 9, 10, 11)
     -62.0f,  -82.0f, -104.0f, -128.0f  // 4 mắt Cánh Trái (12, 13, 14, 15) - trong -> ngoài cùng
};

// Cấu hình PID Tối Ưu Cho Bo Mạch Cánh Én & Cần Dài 9cm:
// Kp=1.35, Ki=0.0, Kd=2.80, baseSpeed=125, minSpeed=-60, maxSpeed=220
static PID_Config_t pidConfig = {
    .Kp = 1.15f,
    .Ki = 0.0f,
    .Kd = 2.40f,
    .baseSpeed = 125,
    .maxSpeed = 220,
    .minSpeed = -60   // Cho phép bánh trong đảo chiều nhẹ (-55 PWM) khi gặp cua vuông 90° thật sự
};

static bool autoMode = false;
static bool steerReverse = false;
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

void pid_set_steer_reverse(bool reverse) {
    steerReverse = reverse;
}

bool pid_get_steer_reverse(void) {
    return steerReverse;
}

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = baseSpeed;
    pidConfig.minSpeed = -60;
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

        // Lọc nhiễu nền thông minh: chỉ lấy tín hiệu thực sự có vạch đen (cal >= 180)
        // và trừ nền 140 để triệt tiêu hoàn toàn nhiễu phản xạ sàn ở các mắt chóp cánh én
        if (cal >= 180) {
            uint32_t eff = cal - 140;
            sumCalibrated += eff;
            sumWeights += (float)eff * SENSOR_WEIGHTS[i];
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
    // Mắt 0..3: Cánh Phải (0b0000000000001111)
    // Mắt 4..11: Giữa thẳng (0b0000111111110000)
    // Mắt 12..15: Cánh Trái (0b1111000000000000)
    uint8_t rightWingCount = __builtin_popcount(mask & 0b0000000000001111);
    uint8_t centerCount    = __builtin_popcount(mask & 0b0000111111110000);
    uint8_t leftWingCount  = __builtin_popcount(mask & 0b1111000000000000);
    uint8_t totalBlack     = leftWingCount + centerCount + rightWingCount;

    // 1. VẠCH NGANG / NGÃ TƯ (Chỉ nhận khi vạch cắt ngang TOÀN BỘ thân xe):
    // Phải có CẢ cánh trái, cụm giữa VÀ cánh phải cùng thấy vạch đen!
    // Hoặc từ 12 mắt đen trở lên cùng lúc.
    // LƯU Ý SỐNG CÒN: Góc 90° chỉ có 1 cánh đen (cánh kia trắng tinh). Nếu chỉ có 1 cánh đen,
    // TUYỆT ĐỐI KHÔNG ĐƯỢC coi là ngã tư, mà phải để hệ thống rẽ 90°!
    if ((leftWingCount >= 2 && rightWingCount >= 2 && centerCount >= 2) || totalBlack >= 12) {
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

        // Mất line -> Kích hoạt tìm lại line dứt khoát
        return LINE_CASE_LOST_LINE;
    }

    // Đã nhìn thấy vạch đen -> Reset cờ mất line
    isLineCurrentlyLost = false;

    // 3. TOÀN BỘ CÁC TRƯỜNG HỢP CÒN LẠI (Đường thẳng, cua nhẹ, cua chữ S, cua 90 độ):
    // -> ĐỀU ĐƯỢC ĐIỀU KHIỂN BẰNG THUẬT TOÁN PID DÒNG CHẢY HÌNH HỌC CÁNH ÉN!
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
            // 1. Vùng đệm êm trung tâm (Soft-Core Center Zone):
            // - Dải bám tâm (|error| <= 18.0): giảm Kp nhẹ xuống 60% để xe chạy thẳng êm ru phẳng lì, không lắc đầu
            // - Dải cua vừa (18.0 < |error| <= 35.0): Kp đạt 100% để giữ xe bám sát tâm đường cong
            // - Dải cua gắt cánh én (|error| > 35.0): Mớm thêm lực cua bẻ góc 90°
            float effectiveKp = pidConfig.Kp;
            if (absErr <= 18.0f) {
                effectiveKp = pidConfig.Kp * (0.60f + 0.40f * (absErr / 18.0f));
            }
            float pTerm = effectiveKp * error;
            if (absErr > 35.0f) {
                float extra = (absErr - 35.0f) * 0.50f;
                pTerm += (error > 0.0f) ? extra : -extra;
            }

            // 2. Khâu vi phân D lọc thông thấp 85% / 15% dập tắt triệt để rung lắc đòn bẩy 9cm:
            float rawDelta = error - lastError;
            lastError = error;
            smoothedDTerm = smoothedDTerm * 0.85f + (pidConfig.Kd * rawDelta) * 0.15f;
            float dTerm = smoothedDTerm;

            // 3. Khâu tích phân I (Anti-windup: kẹp trong khoảng [-80, 80])
            integralError += error * dt;
            if (integralError > 80.0f) integralError = 80.0f;
            if (integralError < -80.0f) integralError = -80.0f;
            float iTerm = pidConfig.Ki * integralError;

            // 4. Bộ lọc dập sốc gia tốc lái vi sai (Slew-Rate Limiter):
            // Giới hạn tốc độ thay đổi bù lái tối đa 4.5 PWM mỗi chu kỳ 5ms,
            // triệt tiêu hoàn toàn hiện tượng rung giật đổi tốc độ bánh xe đột ngột
            float rawCorrection = pTerm + iTerm + dTerm;
            float maxDelta = 4.5f;
            if (rawCorrection - lastCorrection > maxDelta) {
                lastCorrection += maxDelta;
            } else if (rawCorrection - lastCorrection < -maxDelta) {
                lastCorrection -= maxDelta;
            } else {
                lastCorrection = rawCorrection;
            }

            // 4. Giảm ga ôm cua thích ứng cánh én:
            float dragFactor = absErr / 128.0f;
            if (dragFactor > 1.0f) dragFactor = 1.0f;
            int16_t currentBase = pidConfig.baseSpeed - (int16_t)(dragFactor * 35.0f);

            if (steerReverse) {
                lastCorrection = -lastCorrection;
            }

            // Logic 2 bánh PID đồng bộ chuẩn xác với điều khiển tay (Left: trừ bù lái, Right: cộng bù lái)
            int16_t leftSpeed  = currentBase - (int16_t)lastCorrection;
            int16_t rightSpeed = currentBase + (int16_t)lastCorrection;

            // 5. Thuật toán vi sai bám sàn cánh én:
            // - Khi đi thẳng & cua nhẹ (|error| <= 40): dynamicMin = 20 (bánh trong luôn lăn tiến, êm ru tuyệt đối).
            // - Khi cua vừa (40 < |error| <= 70): dynamicMin giảm mượt từ 20 -> 0 (thả trôi ôm cua).
            // - Khi cua vuông 90° gắt (|error| > 70): dynamicMin cho phép đảo chiều nhẹ (-55 PWM)
            //   tạo tâm quay vi sai dứt khoát đưa đầu xe 9cm bẻ góc 90° trong 0.1s!
            int16_t dynamicMin = 20;
            if (absErr > 70.0f) {
                float wingFactor = (absErr - 70.0f) / 58.0f; // 0.0 -> 1.0
                if (wingFactor > 1.0f) wingFactor = 1.0f;
                dynamicMin = -(int16_t)(wingFactor * 55.0f); // 0 -> -55 PWM
            } else if (absErr > 40.0f) {
                float midFactor = (absErr - 40.0f) / 30.0f; // 0.0 -> 1.0
                dynamicMin = (int16_t)(20.0f - midFactor * 20.0f); // 20 -> 0 PWM
            }

            leftSpeed  = clamp_val(leftSpeed, dynamicMin, pidConfig.maxSpeed);
            rightSpeed = clamp_val(rightSpeed, dynamicMin, pidConfig.maxSpeed);

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 4: MẤT LINE (16 mắt đều trắng) -> Tìm lại line dứt khoát
        // ---------------------------------------------------------------------
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Nếu mất line quá 1.5 giây mà không thấy -> Dừng xe để an toàn
            if (lostElapsed > 1500) {
                motor_stop();
                break;
            }

            // Xe quay đầu dứt khoát về phía cánh én vừa thấy vạch cuối cùng
            int8_t searchDir = lastKnownDirection;
            if (steerReverse) searchDir = -searchDir;

            if (searchDir < 0) {
                motor_set_speeds(140, -50); // Cánh Trái thấy vạch cuối -> Lái đầu xe bẻ hướng ngược lại theo logic điều khiển
            } else if (searchDir > 0) {
                motor_set_speeds(-50, 140); // Cánh Phải thấy vạch cuối -> Lái đầu xe bẻ hướng ngược lại theo logic điều khiển
            } else {
                motor_set_speeds(50, 50);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 5: VẠCH NGANG / NGÃ TƯ
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
