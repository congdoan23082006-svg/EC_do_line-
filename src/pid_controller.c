#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// Cấu hình dải mắt cảm biến sử dụng (Đầy đủ 16 mắt: từ mắt 0 đến 15)
#define SENSOR_START_IDX   0      // Bắt đầu từ mắt 0
#define SENSOR_END_IDX     15     // Kết thúc ở mắt 15 (Tổng cộng 16 mắt hoạt động)

// Trọng số vị trí đối xứng cho 16 mắt (Tâm vạch ở giữa mắt 7 và 8)
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 8 mắt bên trái (0..7)
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f  // 8 mắt bên phải (8..15)
};

// Cấu hình PID Siêu Bám Line Mềm Mại & Đầm Chắc: Kp=1.80, Ki=0.0, Kd=3.9, baseSpeed=120
static PID_Config_t pidConfig = {
    .Kp = 1.80f,
    .Ki = 0.0f,
    .Kd = 3.90f,
    .baseSpeed = 120,
    .maxSpeed = 220,
    .minSpeed = -30   // Cua thường: bánh trong hãm chậm hoặc lùi nhẹ để xe bám lăn, triệt tiêu văng trượt
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static float smoothedDTerm = 0.0f;  // Bộ lọc Low-Pass làm mượt D-Term chống giật lắc
static float smoothedBase = 120.0f; // Bộ lọc làm trơn tốc độ ga ôm cua
static unsigned long lastComputeTime = 0;
#define DASHED_LINE_TIMEOUT_MS  60    // Rút ngắn xuống 60ms để tránh phóng mù ra ngoài khi văng góc cua
#define SHARP_TURN_TIMEOUT_MS   450   // Thời gian tối đa giữ cua 90 độ trước khi tự thoát an toàn
#define SHARP_TURN_COOLDOWN_MS  80    // Thời gian miễn nhiễm sau khi thoát cua (tránh quay vòng tròn lặp lại)
#define CROSS_LINE_COOLDOWN_MS  120   // Thời gian miễn nhiễm sau ngã tư (tránh mép vạch ngang kích hoạt nhầm cua 90 độ)
#define SHARP_BRAKE_MS          35    // Thời gian phanh giật ngược (Active Brake) khử trớn quán tính khi bắt đầu cua
static unsigned long lostLineStartTime = 0;
static bool isLineCurrentlyLost = false;
static int8_t lastKnownDirection = 0; // -1: Trái, +1: Phải

// Biến chốt trạng thái rẽ 90 độ
static bool isTurningSharpLeft = false;
static bool isTurningSharpRight = false;
static bool hasClearedOldLine = false;
static bool isSharpCounterBraking = false;
static unsigned long sharpCounterBrakeStartTime = 0;
static int8_t sharpTurnSide = 0; // -1: Trái, +1: Phải
static float lastKnownPosition = 0.0f; // Bộ nhớ vị trí vạch cuối cùng (Line Memory)
static unsigned long sharpTurnStartTime = 0;
static unsigned long lastSharpTurnExitTime = 0;
static unsigned long lastCrossLineExitTime = 0;

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
    smoothedBase = (float)baseSpeed;
    lastComputeTime = millis();
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    isLineCurrentlyLost = false;
    isTurningSharpLeft = false;
    isTurningSharpRight = false;
    hasClearedOldLine = false;
    isSharpCounterBraking = false;
    sharpTurnSide = 0;
    lastKnownPosition = 0.0f;
    lastSharpTurnExitTime = 0;
    lastCrossLineExitTime = 0;
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
    isTurningSharpLeft = false;
    isTurningSharpRight = false;
    hasClearedOldLine = false;
    isSharpCounterBraking = false;
    sharpTurnSide = 0;
    lastKnownPosition = 0.0f;
    lastSharpTurnExitTime = 0;
    lastCrossLineExitTime = 0;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
    } else {
        integralError = 0.0f;
        lastError = 0.0f;
        smoothedDTerm = 0.0f;
        smoothedBase = (float)pidConfig.baseSpeed;
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

    // Quét toàn bộ mắt từ 1 đến 14 bằng giá trị chuẩn hóa Analog 0 - 1000
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        uint16_t cal = sensor_get_calibrated(i); // Giá trị 0 - 1000 chuẩn hóa

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

    // Mất vạch hoàn toàn (cả 14 mắt đều trắng tinh) -> Giữ vị trí cuối cùng
    *outPosition = lastKnownPosition;
    return false;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady) {
        return LINE_CASE_STOPPED;
    }

    uint16_t mask = sensor_get_line_bits();
    uint8_t totalBlack = __builtin_popcount(mask & 0b1111111111111111);

    // =========================================================================
    // 1. KIỂM TRA TRẠNG THÁI ĐANG GIỮ CUA 90 ĐỘ (LATCHED TURN)
    // =========================================================================
    // Kiểm tra pha 3: Phanh dập quán tính quay (Counter-Brake 22ms) để khóa cứng tâm vạch mới
    if (isSharpCounterBraking) {
        if (millis() - sharpCounterBrakeStartTime >= 22) {
            isSharpCounterBraking = false;
            lastSharpTurnExitTime = millis();
            lastKnownDirection = 0;
            lastError = 0.0f;
            integralError = 0.0f;
            return LINE_CASE_NORMAL_PID; // Khóa vào line mới siêu ổn định!
        }
        return (sharpTurnSide < 0) ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
    }

    if (isTurningSharpLeft) {
        unsigned long turnElapsed = millis() - sharpTurnStartTime;

        // Bước 1: Xác nhận thanh cảm biến đã quét hoàn toàn ra khỏi vạch cũ (tránh thoát cua non)
        if (!hasClearedOldLine) {
            if ((mask & 0b0000001111000000) == 0 && turnElapsed >= 14) {
                hasClearedOldLine = true;
            }
        }

        // Bước 2: Khi đã rời vạch cũ (hoặc sau 22ms an toàn), đón bắt vạch line mới
        if (hasClearedOldLine || turnElapsed >= 22) {
            if ((mask & 0b0000001111000000) != 0 || (turnElapsed >= 45 && (mask & 0b0000011111100000) != 0)) {
                isTurningSharpLeft = false;
                // Kích hoạt Pha 3: Xung phanh ngược dập tắt 100% quán tính xoay
                isSharpCounterBraking = true;
                sharpCounterBrakeStartTime = millis();
                sharpTurnSide = -1;
                return LINE_CASE_SHARP_LEFT;
            }
        }

        if (turnElapsed > SHARP_TURN_TIMEOUT_MS) {
            isTurningSharpLeft = false;
            lastSharpTurnExitTime = millis();
            return LINE_CASE_NORMAL_PID;
        } else {
            return LINE_CASE_SHARP_LEFT;
        }
    }

    if (isTurningSharpRight) {
        unsigned long turnElapsed = millis() - sharpTurnStartTime;

        // Bước 1: Xác nhận thanh cảm biến đã quét hoàn toàn ra khỏi vạch cũ (tránh thoát cua non)
        if (!hasClearedOldLine) {
            if ((mask & 0b0000001111000000) == 0 && turnElapsed >= 14) {
                hasClearedOldLine = true;
            }
        }

        // Bước 2: Khi đã rời vạch cũ (hoặc sau 22ms an toàn), đón bắt vạch line mới
        if (hasClearedOldLine || turnElapsed >= 22) {
            if ((mask & 0b0000001111000000) != 0 || (turnElapsed >= 45 && (mask & 0b0000011111100000) != 0)) {
                isTurningSharpRight = false;
                // Kích hoạt Pha 3: Xung phanh ngược dập tắt 100% quán tính xoay
                isSharpCounterBraking = true;
                sharpCounterBrakeStartTime = millis();
                sharpTurnSide = 1;
                return LINE_CASE_SHARP_RIGHT;
            }
        }

        if (turnElapsed > SHARP_TURN_TIMEOUT_MS) {
            isTurningSharpRight = false;
            lastSharpTurnExitTime = millis();
            return LINE_CASE_NORMAL_PID;
        } else {
            return LINE_CASE_SHARP_RIGHT;
        }
    }

    // =========================================================================
    // 2. VẠCH NGANG / NGÃ TƯ / VẠCH ĐÍCH (Từ 11 mắt đen trở lên cùng lúc)
    // =========================================================================
    if (totalBlack >= 11) {
        lastCrossLineExitTime = millis(); // Ghi nhận để tránh mép vạch ngang kích hoạt nhầm cua 90 độ
        return LINE_CASE_CROSS_LINE;
    }

    // Cờ kiểm tra xem có được phép kích hoạt cua 90 độ không (chống kích hoạt lặp & chống nhầm sau ngã tư)
    bool canTriggerSharp = (millis() - lastSharpTurnExitTime >= SHARP_TURN_COOLDOWN_MS) &&
                           (millis() - lastCrossLineExitTime >= CROSS_LINE_COOLDOWN_MS);

    // =========================================================================
    // 3. MẤT LINE HOÀN TOÀN (Cả 14 mắt đều trắng tinh: blackCount == 0)
    // SỬ DỤNG BỘ NHỚ VỊ TRÍ CUỐI (LINE MEMORY) - BÍ KÍP ĐỘI VÔ ĐỊCH!
    // =========================================================================
    if (blackCount == 0) {
        if (!isLineCurrentlyLost) {
            isLineCurrentlyLost = true;
            lostLineStartTime = millis();
        }

        unsigned long lostElapsed = millis() - lostLineStartTime;

        // Nếu vừa mất vạch ở bên trái (lastKnownPosition < -22.0f)
        // -> Đây là CUA VUÔNG 90 ĐỘ TRÁI! Kích hoạt xoay đón vạch ngay!
        if (canTriggerSharp && lastKnownPosition < -22.0f) {
            isTurningSharpLeft = true;
            isTurningSharpRight = false;
            hasClearedOldLine = false;
            isSharpCounterBraking = false;
            sharpTurnStartTime = millis();
            lastKnownDirection = -1;
            return LINE_CASE_SHARP_LEFT;
        }

        // Nếu vừa mất vạch ở bên phải (lastKnownPosition > 22.0f)
        // -> Đây là CUA VUÔNG 90 ĐỘ PHẢI! Kích hoạt xoay đón vạch ngay!
        if (canTriggerSharp && lastKnownPosition > 22.0f) {
            isTurningSharpRight = true;
            isTurningSharpLeft = false;
            hasClearedOldLine = false;
            isSharpCounterBraking = false;
            sharpTurnStartTime = millis();
            lastKnownDirection = 1;
            return LINE_CASE_SHARP_RIGHT;
        }

        // Nếu trước đó đang bám thẳng ở giữa tâm (lastKnownDirection == 0) trong thời gian ngắn (<= 60ms)
        // -> Đây là ĐOẠN NÉT ĐỨT (Dashed line) -> Tiếp tục lướt thẳng qua
        if (lastKnownDirection == 0 && fabsf(lastKnownPosition) < 15.0f && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        // Mất line bất thường -> Kích hoạt tìm lại line
        return LINE_CASE_LOST_LINE;
    }

    // Đã nhìn thấy vạch đen -> Reset cờ mất line
    isLineCurrentlyLost = false;

    // =========================================================================
    // 4. PHÁT HIỆN SỚM CUA 90 ĐỘ KHI VẠCH CẮT NGANG MÉP NGOÀI
    // =========================================================================
    uint8_t leftBits  = __builtin_popcount(mask & 0b0000000011111111); // 8 mắt trái (0..7)
    uint8_t rightBits = __builtin_popcount(mask & 0b1111111100000000); // 8 mắt phải (8..15)

    // Cua 90° Trái: Mép ngoài cùng (0..3) đen VÀ nửa phải trắng (cho phép nhiễu <= 1 mắt dính mép cũ)
    if (canTriggerSharp && (mask & 0b0000000000001111) != 0 && leftBits >= 3 && rightBits <= 1) {
        isTurningSharpLeft = true;
        isTurningSharpRight = false;
        hasClearedOldLine = false;
        isSharpCounterBraking = false;
        sharpTurnStartTime = millis();
        lastKnownDirection = -1;
        return LINE_CASE_SHARP_LEFT;
    }

    // Cua 90° Phải: Mép ngoài cùng (12..15) đen VÀ nửa trái trắng (cho phép nhiễu <= 1 mắt dính mép cũ)
    if (canTriggerSharp && (mask & 0b1111000000000000) != 0 && rightBits >= 3 && leftBits <= 1) {
        isTurningSharpRight = true;
        isTurningSharpLeft = false;
        hasClearedOldLine = false;
        isSharpCounterBraking = false;
        sharpTurnStartTime = millis();
        lastKnownDirection = 1;
        return LINE_CASE_SHARP_RIGHT;
    }

    // =========================================================================
    // 5. PHÁT HIỆN NGÃ 3 CHỮ T (CASE 6 & CASE 7)
    // =========================================================================
    // Ngã 3 chữ T Trái: Tâm có vạch thẳng VÀ mép trái (0..3) có vạch rẽ ngang VÀ nửa phải trắng
    if ((mask & 0b0000001111000000) != 0 && (mask & 0b0000000000001111) != 0 && rightBits <= 1) {
        return LINE_CASE_T_LEFT;
    }

    // Ngã 3 chữ T Phải: Tâm có vạch thẳng VÀ mép phải (12..15) có vạch rẽ ngang VÀ nửa trái trắng
    if ((mask & 0b0000001111000000) != 0 && (mask & 0b1111000000000000) != 0 && leftBits <= 1) {
        return LINE_CASE_T_RIGHT;
    }

    // =========================================================================
    // 6. TOÀN BỘ CÁC TRƯỜNG HỢP CÒN LẠI (Đường thẳng, cua nhẹ, cua chữ S, cua gấp):
    // -> ĐỀU ĐƯỢC ĐIỀU KHIỂN BẰNG THUẬT TOÁN PID TRỌNG TÂM LIÊN TỤC!
    // =========================================================================
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
        // CASE 1: BÁM LINE CHUẨN PID (Chạy mềm mại, đầm chắc, triệt tiêu 100% văng xe)
        // ---------------------------------------------------------------------
        case LINE_CASE_NORMAL_PID: {
            float error = position; // Sai số liên tục (-65.0 đến +65.0)
            float absErr = fabsf(error);

            // 1. Khâu P mượt mà (Smooth Proportional):
            // Giữ lái cực êm ở dải tâm (|error| <= 14) và tăng lực lái vừa phải ở góc cua
            float pTerm = pidConfig.Kp * error;
            if (absErr > 14.0f) {
                float extra = (absErr - 14.0f) * 0.35f;
                pTerm += (error > 0.0f) ? extra : -extra;
            }

            // 2. Khâu vi phân D thông minh (Dynamic Asymmetric Damping - Bí kíp chống văng lố):
            // - Khi xe đang lệch ra xa line (error * rawDelta > 0): D bình thường để xe bẻ lái nhanh, không bị đơ lái
            // - Khi xe đang lao đầu quay về tâm line (error * rawDelta < 0): D tự động tăng mạnh để dập tắt quán tính xoay,
            //   ghìm chết đầu xe ngay giữa tâm vạch, KHÔNG CHO XE VĂNG LỐ SANG PHÍA BÊN KIA!
            float rawDelta = error - lastError;
            lastError = error;

            float effectiveKd = pidConfig.Kd;
            if (error * rawDelta < 0.0f) {
                // Đang trên đà quay về tâm -> Tăng 75% lực giảm chấn dập tắt đà xoay văng lố
                effectiveKd *= 1.75f;
            }
            smoothedDTerm = smoothedDTerm * 0.50f + (effectiveKd * rawDelta) * 0.50f;
            float dTerm = smoothedDTerm;

            // 3. Khâu tích phân I (Anti-windup: kẹp trong khoảng [-80, 80])
            integralError += error * dt;
            if (integralError > 80.0f) integralError = 80.0f;
            if (integralError < -80.0f) integralError = -80.0f;
            float iTerm = pidConfig.Ki * integralError;

            // Tổng bù lái
            lastCorrection = pTerm + iTerm + dTerm;

            // 4. Giảm tốc ôm cua chủ động sâu hơn (Deep Adaptive Corner Drag):
            // Xe tự động hạ ga sâu khi vào khúc cua để dồn trọng tâm xe cắn chặt mặt sàn,
            // triệt tiêu hoàn toàn lực ly tâm làm trượt văng xe ra ngoài lề
            float targetBase = (float)pidConfig.baseSpeed;
            if (absErr > 8.0f) {
                float drag = (absErr - 8.0f) * 1.25f; // Giảm ga chủ động theo độ gắt của khúc cua
                targetBase -= drag;
                float minCornerSpeed = (float)pidConfig.baseSpeed * 0.55f;
                if (minCornerSpeed < 65.0f) minCornerSpeed = 65.0f;
                if (targetBase < minCornerSpeed) targetBase = minCornerSpeed;
            }
            // Bộ lọc làm trơn ga chuyển tiếp êm ái
            smoothedBase = smoothedBase * 0.80f + targetBase * 0.20f;
            int16_t currentBase = (int16_t)smoothedBase;

            int16_t leftSpeed  = currentBase + (int16_t)lastCorrection;
            int16_t rightSpeed = currentBase - (int16_t)lastCorrection;

            // Giới hạn tốc độ trong dải an toàn (Bánh trong giữ độ bám lăn, không lùi gắt làm trượt lốp)
            leftSpeed  = clamp_val(leftSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
            rightSpeed = clamp_val(rightSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 2: CUA GẮT TRÁI 90 ĐỘ (3 Pha Thích Ứng Theo baseSpeed)
        // ---------------------------------------------------------------------
        case LINE_CASE_SHARP_LEFT: {
            int16_t base = pidConfig.baseSpeed;
            int16_t brakeHard = -clamp_val((int16_t)(base * 1.25f), 130, 215);
            int16_t brakeSoft = clamp_val((int16_t)(base * 0.22f), 20, 45);
            int16_t spinFwd = clamp_val((int16_t)(base * 1.10f), 120, 210);
            int16_t spinRev = -clamp_val((int16_t)(base * 0.95f), 110, 195);
            int16_t counterBrake = clamp_val((int16_t)(base * 0.55f), 55, 95);

            if (isSharpCounterBraking) {
                // Pha 3 (22ms): Xung phanh ngược ghìm chết đà quay thích ứng theo tốc độ
                motor_set_speeds(counterBrake, -counterBrake);
            } else {
                unsigned long turnElapsed = millis() - sharpTurnStartTime;
                if (turnElapsed < 30) {
                    // Pha 1 (0-30ms): Phanh giật bánh trong khử trớn lao thẳng, mớm góc quay
                    motor_set_speeds(brakeHard, brakeSoft);
                } else {
                    // Pha 2 (>30ms): Xoay tâm nhanh dứt khoát theo tốc độ, không trượt lốp
                    motor_set_speeds(spinRev, spinFwd);
                }
            }
            integralError = 0.0f;
            lastError = -10.0f;
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 3: CUA GẮT PHẢI 90 ĐỘ (3 Pha Thích Ứng Theo baseSpeed)
        // ---------------------------------------------------------------------
        case LINE_CASE_SHARP_RIGHT: {
            int16_t base = pidConfig.baseSpeed;
            int16_t brakeHard = -clamp_val((int16_t)(base * 1.25f), 130, 215);
            int16_t brakeSoft = clamp_val((int16_t)(base * 0.22f), 20, 45);
            int16_t spinFwd = clamp_val((int16_t)(base * 1.10f), 120, 210);
            int16_t spinRev = -clamp_val((int16_t)(base * 0.95f), 110, 195);
            int16_t counterBrake = clamp_val((int16_t)(base * 0.55f), 55, 95);

            if (isSharpCounterBraking) {
                // Pha 3 (22ms): Xung phanh ngược ghìm chết đà quay thích ứng theo tốc độ
                motor_set_speeds(-counterBrake, counterBrake);
            } else {
                unsigned long turnElapsed = millis() - sharpTurnStartTime;
                if (turnElapsed < 30) {
                    // Pha 1 (0-30ms): Phanh giật bánh trong khử trớn lao thẳng, mớm góc quay
                    motor_set_speeds(brakeSoft, brakeHard);
                } else {
                    // Pha 2 (>30ms): Xoay tâm nhanh dứt khoát theo tốc độ, không trượt lốp
                    motor_set_speeds(spinFwd, spinRev);
                }
            }
            integralError = 0.0f;
            lastError = 10.0f;
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 4: MẤT LINE (16 mắt đều trắng)
        // ---------------------------------------------------------------------
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Nếu mất line quá 1.2 giây mà không thấy -> Dừng xe để an toàn
            if (lostElapsed > 1200) {
                motor_stop();
                break;
            }

            // Xoay êm ái tại chỗ tìm lại line với tốc độ vừa phải
            if (lastKnownDirection < 0) {
                motor_set_speeds(-75, 70);
            } else if (lastKnownDirection > 0) {
                motor_set_speeds(70, -75);
            } else {
                motor_set_speeds(-65, -65);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 5: VẠCH NGANG / NGÃ TƯ / VẠCH XUẤT PHÁT
        // ---------------------------------------------------------------------
        case LINE_CASE_CROSS_LINE: {
            // Giữ hướng chạy thẳng vượt qua giao lộ với tốc độ cơ bản
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 6: NGÃ 3 CHỮ T BÊN TRÁI
        // ---------------------------------------------------------------------
        case LINE_CASE_T_LEFT: {
            // Ưu tiên rẽ trái ôm cua
            int16_t slowSpd = -(int16_t)(pidConfig.baseSpeed * 0.4f);
            motor_set_speeds(slowSpd, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 7: NGÃ 3 CHỮ T BÊN PHẢI
        // ---------------------------------------------------------------------
        case LINE_CASE_T_RIGHT: {
            // Ưu tiên rẽ phải ôm cua
            int16_t slowSpd = -(int16_t)(pidConfig.baseSpeed * 0.4f);
            motor_set_speeds(pidConfig.baseSpeed, slowSpd);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 9: ĐOẠN NÉT ĐỨT (DASHED LINE)
        // ---------------------------------------------------------------------
        case LINE_CASE_DASHED_LINE: {
            // Xe duy trì chạy thẳng với tốc độ an toàn để lướt qua khoảng trắng, tránh phóng mù quá nhanh
            int16_t dashSpeed = clamp_val(pidConfig.baseSpeed, 80, 120);
            motor_set_speeds(dashSpeed, dashSpeed);
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
