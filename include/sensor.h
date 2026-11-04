#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =================== CẤU HÌNH CHÂN (PIN CONFIG) ===================
#define PIN_LINE_SIG          36      // Chân đọc Analog (ADC1_CH0)
#define PIN_MUX_A             17      // S0
#define PIN_MUX_B             18      // S1
#define PIN_MUX_C             19      // S2
#define PIN_MUX_D             23      // S3
#define PIN_CALIB             34      // Nút nhấn hiệu chuẩn (Calibration)

// =================== THÔNG SỐ CẢM BIẾN ===================
#define N_SENSORS             16      // Tổng số mắt
#define SENSOR_START_IDX      0       // Bắt đầu từ mắt 0 (sử dụng trọn vẹn 16 mắt)
#define SENSOR_END_IDX        15      // Kết thúc ở mắt 15 (sử dụng trọn vẹn 16 mắt)
#define ADC_SAMPLES           4       // Số lần lấy mẫu ADC trung bình
#define MIN_CALIBRATION_DELTA 150     // Chênh lệch tối thiểu giữa Min và Max (tránh nhiễu ADC)
#define BUTTON_DEBOUNCE_MS    50      // Thời gian chống rung phím (ms)

// =================== CÁC BIẾN TOÀN CỤC ===================
extern uint16_t sensorRaw[N_SENSORS];
extern uint16_t minValues[N_SENSORS];
extern uint16_t maxValues[N_SENSORS];
extern uint16_t thresholds[N_SENSORS];
extern bool blackIsHigher[N_SENSORS];
extern bool isCalibrating;
extern bool calibrationReady;

// =================== CÁC HÀM XỬ LÝ (FUNCTIONS) ===================

/**
 * @brief Khởi tạo các chân GPIO cho MUX, chân nút bấm và cấu hình ADC 12-bit
 */
void sensor_init(void);

/**
 * @brief Chọn kênh từ 0 đến 15 trên MUX CD74HC4067
 * @param channel Kênh cần chọn (0 - 15)
 */
void sensor_mux_select(uint8_t channel);

/**
 * @brief Đọc giá trị Analog từ kênh đã chọn (có lấy trung bình mẫu)
 * @param channel Kênh cần đọc (0 - 15)
 * @return Giá trị ADC 12-bit (0 - 4095)
 */
uint16_t sensor_read_mux(uint8_t channel);

/**
 * @brief Đọc toàn bộ các mắt cảm biến và cập nhật vào mảng sensorRaw
 */
void sensor_read_all(void);

/**
 * @brief Bắt đầu quá trình hiệu chuẩn LIA CẢM BIẾN QUA LẠI
 * @param duration_ms Thời gian cho người dùng lia (mặc định 4000ms = 4 giây)
 */
void sensor_start_sweep_calibration(uint32_t duration_ms);

/**
 * @brief Cập nhật quá trình quét Min/Max liên tục (gọi trong hàm loop)
 */
void sensor_update_calibration_loop(void);

/**
 * @brief Chốt kết thúc quá trình lia và tính ngưỡng trung bình = (Min + Max) / 2
 */
void sensor_finish_sweep_calibration(void);

/**
 * @brief Kiểm tra xem hệ thống có đang trong thời gian lia calib hay không
 */
bool sensor_is_calibrating(void);

/**
 * @brief Kiểm tra nút nhấn hiệu chuẩn (có chống rung phím)
 * @return true nếu nút vừa được nhấn nhả hợp lệ
 */
bool sensor_is_button_pressed(void);

/**
 * @brief Xử lý khi nhấn nút calib:
 *        - Nếu chưa calib: Bắt đầu lia trong 4 giây.
 *        - Nếu đang lia: Bấm nút để chốt kết thúc sớm.
 */
void sensor_handle_calibration_step(void);

/**
 * @brief Lấy giá trị cảm biến đã chuẩn hóa theo thang đo 0 - 1000 (0: Trắng, 1000: Đen)
 * @param index Chỉ số mắt cảm biến (1 - 14)
 * @return Giá trị từ 0 đến 1000
 */
uint16_t sensor_get_calibrated(uint8_t index);

/**
 * @brief Kiểm tra xem một mắt cảm biến đang ở vạch đen (1) hay nền trắng (0)
 * @param index Chỉ số mắt cảm biến (0 - 15)
 * @return true nếu là vạch đen (1), false nếu là nền trắng (0)
 */
bool sensor_is_black(uint8_t index);

/**
 * @brief Lấy trạng thái các mắt cảm biến dưới dạng bitmask 16-bit
 */
uint16_t sensor_get_line_bits(void);

/**
 * @brief Cài đặt cực tính cảm biến (true: Đen có ADC cao hơn trắng, false: Trắng có ADC cao hơn đen)
 */
void sensor_set_polarity(bool black_is_higher);

/**
 * @brief Lấy trạng thái cực tính cảm biến hiện tại
 */
bool sensor_get_polarity(void);

/**
 * @brief In trạng thái các mắt cảm biến (0/1) ra Serial
 */
void sensor_print_states(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_H
