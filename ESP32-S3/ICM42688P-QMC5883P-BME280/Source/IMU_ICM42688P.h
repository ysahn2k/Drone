// ============================================================================
//  IMU_ICM42688P.h  —  ICM-42688-P SPI 고성능 IMU 드라이버
//
//  특징:
//    - SPI 인터페이스 (최대 24MHz) → I2C 버그 완전 회피
//    - 자이로: 32kHz 최대 ODR, ±2000 dps / 노이즈 밀도 0.0028°/s/√Hz
//    - 가속도: 32kHz 최대 ODR, ±16g
//    - 동결(Freeze) 현상 없음 (SPI 직접 레지스터 접근, 외부 펌웨어 의존 없음)
//    - Betaflight ICM-42688-P 동일 칩 (FC 표준)
//
//  인터페이스:
//    GY87 호환 — IMU_GY87와 동일한 getData() / calibrate() / getCalData() 사용
//    AHRS: 6축 모드 (ahrs.updateIMU 사용, 자력계 없음)
//
//  Phase 4 하드웨어 연결 (ESP32-S3 FSPI):
//    SCK  → PIN_SPI_SCK  (DroneConfig: GPIO 36)
//    MISO → PIN_SPI_MISO (DroneConfig: GPIO 37)
//    MOSI → PIN_SPI_MOSI (DroneConfig: GPIO 35)
//    CS   → PIN_CS_ICM42688P (DroneConfig: GPIO 34)
// ============================================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

// ── ICM-42688-P 레지스터 맵 (Bank 0) ─────────────────────────────────────
#define ICM_DEVICE_CONFIG        0x11   // 소프트 리셋 (bit0 = 1)
#define ICM_INT_CONFIG           0x14
#define ICM_TEMP_DATA1           0x1D   // 온도 상위 바이트
#define ICM_TEMP_DATA0           0x1E   // 온도 하위 바이트
#define ICM_ACCEL_DATA_X1        0x1F   // 가속도 X 상위 (총 6바이트: XH,XL,YH,YL,ZH,ZL)
#define ICM_GYRO_DATA_X1         0x25   // 자이로 X 상위 (총 6바이트)
#define ICM_PWR_MGMT0            0x4E   // 전원 관리
#define ICM_GYRO_CONFIG0         0x4F   // 자이로 FS/ODR
#define ICM_ACCEL_CONFIG0        0x50   // 가속도 FS/ODR
#define ICM_GYRO_CONFIG1         0x51   // 자이로 LPF 설정
#define ICM_GYRO_ACCEL_CONFIG0   0x52   // 가속도 LPF 설정
#define ICM_WHO_AM_I             0x75   // 칩 ID → 0x47 반환 확인
#define ICM_BANK_SEL             0x76   // 레지스터 뱅크 선택

#define ICM42688P_WHO_AM_I_VAL   0x47   // 예상 WHO_AM_I 값

// PWR_MGMT0: 가속도+자이로 저잡음 모드
#define ICM_GYRO_ACCEL_LN        0x0F

// ── 자이로 FS 설정 (GYRO_CONFIG0 bits[7:5]) ──────────────────────────────
#define ICM_GYRO_FS_2000DPS      (0 << 5)   // ±2000 dps (16.4 LSB/dps)
#define ICM_GYRO_FS_1000DPS      (1 << 5)   // ±1000 dps (32.8 LSB/dps) ← 권장
#define ICM_GYRO_FS_500DPS       (2 << 5)   // ±500  dps (65.5 LSB/dps)
#define ICM_GYRO_FS_250DPS       (3 << 5)   // ±250  dps (131  LSB/dps)

// ── 자이로 ODR 설정 (GYRO_CONFIG0 bits[3:0]) ─────────────────────────────
#define ICM_ODR_8KHZ             0x05       // 8 kHz
#define ICM_ODR_4KHZ             0x06       // 4 kHz
#define ICM_ODR_2KHZ             0x07       // 2 kHz
#define ICM_ODR_1KHZ             0x08       // 1 kHz ← 우리 루프 주기와 동기화

// ── 가속도 FS 설정 (ACCEL_CONFIG0 bits[7:5]) ─────────────────────────────
#define ICM_ACCEL_FS_16G         (0 << 5)   // ±16g (2048 LSB/g)
#define ICM_ACCEL_FS_8G          (1 << 5)   // ±8g  (4096 LSB/g) ← 권장
#define ICM_ACCEL_FS_4G          (2 << 5)   // ±4g  (8192 LSB/g)
#define ICM_ACCEL_FS_2G          (3 << 5)   // ±2g  (16384 LSB/g)

// ── 데이터 구조체 ─────────────────────────────────────────────────────────
struct ICM_Data {
    float ax = 0.f, ay = 0.f, az = 0.f;   // 가속도 [m/s²]
    float gx = 0.f, gy = 0.f, gz = 0.f;   // 자이로  [rad/s]
    float temp = 0.f;                       // 온도    [°C]
};

struct ICM_Cal {
    int16_t axOfs = 0, ayOfs = 0, azOfs = 0;  // 가속도 오프셋 [LSB]
    int16_t gxOfs = 0, gyOfs = 0, gzOfs = 0;  // 자이로 오프셋 [LSB]
};

// ============================================================================
class IMU_ICM42688P {
public:
    IMU_ICM42688P();

    // ── 초기화 ──────────────────────────────────────────────────────────────
    // csPin    : SPI Chip Select 핀 (DroneConfig: PIN_CS_ICM42688P)
    // spi      : SPI 객체 (기본: SPI, FSPI 사용 시 SPIClass spiIMU(FSPI))
    // spiClock : SPI 클럭 [Hz] (기본 8MHz, 최대 24MHz)
    bool begin(uint8_t csPin,
               SPIClass& spi      = SPI,
               uint32_t  spiClock = 8000000UL);

    // ── 매 루프 호출 (1kHz) ─────────────────────────────────────────────────
    void update();

    // ── 캘리브레이션 ────────────────────────────────────────────────────────
    // 드론을 수평 정지 상태로 놓고 호출
    void calibrate(uint16_t samples = 2000);
    bool loadCalibration();   // NVS에서 읽기
    bool saveCalibration();   // NVS에 저장

    // ── 데이터 접근 ─────────────────────────────────────────────────────────
    const ICM_Data& getData()    const { return data_; }
    const ICM_Cal&  getCalData() const { return cal_;  }

    float getAccelX()  const { return data_.ax; }
    float getAccelY()  const { return data_.ay; }
    float getAccelZ()  const { return data_.az; }
    float getGyroX()   const { return data_.gx; }   // rad/s
    float getGyroY()   const { return data_.gy; }
    float getGyroZ()   const { return data_.gz; }
    float getTemp()    const { return data_.temp; }

    // ── AHRS 6축 업데이트용 래퍼 (GY87 호환 인터페이스) ──────────────────
    // ahrs.updateIMU(imu.getGyroX(), imu.getGyroY(), imu.getGyroZ(),
    //               imu.getAccelX(), imu.getAccelY(), imu.getAccelZ(), dt);

private:
    uint8_t   csPin_;
    SPIClass* spi_;
    uint32_t  spiClock_;
    ICM_Data  data_;
    ICM_Cal   cal_;
    float     accelScale_;   // m/s² per LSB
    float     gyroScale_;    // rad/s per LSB

    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg (uint8_t reg);
    void    readRegs(uint8_t startReg, uint8_t* buf, uint8_t len);
};
