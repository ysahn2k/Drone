// ============================================================================
//  IMU_ICM42688P.cpp  —  ICM-42688-P SPI 드라이버 구현
// ============================================================================
#include "IMU_ICM42688P.h"
#include "DroneConfig.h"

#ifdef IMU_SELECT_ICM42688P

#include <math.h>
#include <string.h>
#include <Preferences.h>

#define GRAVITY_MS2      9.80665f
#define DEG_TO_RAD_F     (float)(M_PI / 180.0)

// NVS 네임스페이스 (GY87와 충돌 방지)
#define NVS_NS_ICM       "icm42688cal"

// ============================================================================
//  생성자
// ============================================================================
IMU_ICM42688P::IMU_ICM42688P()
    : csPin_(255), spi_(&SPI), spiClock_(8000000UL),
      accelScale_(0.f), gyroScale_(0.f) {}

// ============================================================================
//  begin() — SPI 초기화 및 센서 설정
// ============================================================================
bool IMU_ICM42688P::begin(uint8_t csPin, SPIClass& spi, uint32_t spiClock)
{
    csPin_    = csPin;
    spi_      = &spi;
    spiClock_ = spiClock;

    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    delayMicroseconds(100);

    // ── WHO_AM_I 확인 ─────────────────────────────────────────────────────
    uint8_t whoami = readReg(ICM_WHO_AM_I);
    if (whoami != ICM42688P_WHO_AM_I_VAL) {
        Serial.printf("[ICM42688P] WHO_AM_I 불일치: 0x%02X (기대: 0x%02X)\n",
                      whoami, ICM42688P_WHO_AM_I_VAL);
        Serial.println("  → SPI 배선(SCK/MOSI/MISO/CS) 및 핀 번호를 확인하세요.");
        return false;
    }

    // ── 소프트 리셋 ───────────────────────────────────────────────────────
    writeReg(ICM_DEVICE_CONFIG, 0x01);
    delay(2);   // 리셋 후 최소 1ms 대기

    // ── Accel + Gyro 저잡음(Low Noise) 모드 활성 ─────────────────────────
    // 0x0F = accel LN (bits[1:0]=11) + gyro LN (bits[3:2]=11)
    writeReg(ICM_PWR_MGMT0, ICM_GYRO_ACCEL_LN);
    delayMicroseconds(300);  // 전원 안정화 (최소 200µs 필요)

    // ── 자이로 설정: ±1000 dps, 1kHz ODR ─────────────────────────────────
    // ±1000 dps: 일반적인 드론 비행 최대 각속도(~500 dps)를 안전하게 커버
    writeReg(ICM_GYRO_CONFIG0, ICM_GYRO_FS_1000DPS | ICM_ODR_1KHZ);

    // ── 가속도 설정: ±8g, 1kHz ODR ───────────────────────────────────────
    // ±8g: 급기동 시 최대 5g 내외 → 충분한 여유
    writeReg(ICM_ACCEL_CONFIG0, ICM_ACCEL_FS_8G | ICM_ODR_1KHZ);

    delay(5);   // 센서 안정화

    // ── 스케일 계수 계산 ──────────────────────────────────────────────────
    // ±1000 dps: 32.8 LSB/dps → 1/32.8 dps/LSB → rad/s/LSB
    gyroScale_  = DEG_TO_RAD_F / 32.8f;

    // ±8g: 4096 LSB/g → m/s²/LSB
    accelScale_ = GRAVITY_MS2 / 4096.0f;

    // ── NVS에서 캘리브레이션 로드 (없으면 기본값 0) ──────────────────────
    loadCalibration();

    Serial.printf("[ICM42688P] 초기화 완료  자이로:±1000dps  가속도:±8g  ODR:1kHz\n");
    return true;
}

// ============================================================================
//  update() — 매 1kHz 루프에서 호출
//  온도(2B) + 가속도(6B) + 자이로(6B) = 14바이트 버스트 읽기
// ============================================================================
void IMU_ICM42688P::update()
{
    // 0x1D(TEMP_DATA1)부터 14바이트 연속 읽기
    // [0,1]   TEMP_DATA1/0   (온도 MSB/LSB)
    // [2,3]   ACCEL_DATA_X1/0
    // [4,5]   ACCEL_DATA_Y1/0
    // [6,7]   ACCEL_DATA_Z1/0
    // [8,9]   GYRO_DATA_X1/0
    // [10,11] GYRO_DATA_Y1/0
    // [12,13] GYRO_DATA_Z1/0
    uint8_t buf[14];
    readRegs(ICM_TEMP_DATA1, buf, 14);

    // 온도: T[°C] = raw / 132.48 + 25  (ICM42688P 데이터시트)
    int16_t rawTemp = (int16_t)((buf[0] << 8) | buf[1]);
    data_.temp = (float)rawTemp / 132.48f + 25.0f;

    // 가속도 (오프셋 보정 후 변환)
    int16_t rawAx = (int16_t)((buf[2]  << 8) | buf[3])  - cal_.axOfs;
    int16_t rawAy = (int16_t)((buf[4]  << 8) | buf[5])  - cal_.ayOfs;
    int16_t rawAz = (int16_t)((buf[6]  << 8) | buf[7])  - cal_.azOfs;

    // 자이로 (오프셋 보정 후 변환)
    int16_t rawGx = (int16_t)((buf[8]  << 8) | buf[9])  - cal_.gxOfs;
    int16_t rawGy = (int16_t)((buf[10] << 8) | buf[11]) - cal_.gyOfs;
    int16_t rawGz = (int16_t)((buf[12] << 8) | buf[13]) - cal_.gzOfs;

    data_.ax = (float)rawAx * accelScale_;
    data_.ay = (float)rawAy * accelScale_;
    data_.az = (float)rawAz * accelScale_;
    data_.gx = (float)rawGx * gyroScale_;
    data_.gy = (float)rawGy * gyroScale_;
    // Z축 부호 반전: ICM42688P 기본 Z축 방향이 GY87(MPU6050)와 반대일 경우 대비
    // 실제 비행 후 Yaw 방향이 반대이면 아래 부호를 제거
    data_.gz = -(float)rawGz * gyroScale_;
}

// ============================================================================
//  calibrate() — 드론 수평 정지 상태에서 호출 (2000 샘플 평균)
// ============================================================================
void IMU_ICM42688P::calibrate(uint16_t samples)
{
    Serial.println("[ICM42688P 캘리브레이션] 시작...");
    Serial.println("  드론을 수평 정지 상태로 유지하세요. (약 2초)");
    delay(500);  // 준비 대기

    // 임시로 오프셋 0으로 설정하여 원시값 수집
    memset(&cal_, 0, sizeof(cal_));

    // ── ★ Betaflight 분석 적용: 표준편차 기반 흔들림 감지 + 재시도 ──
    for (uint8_t attempt = 1; attempt <= GYRO_CAL_MAX_RETRY; attempt++) {
        long sumAx=0, sumAy=0, sumAz=0;
        long sumGx=0, sumGy=0, sumGz=0;
        int64_t sumGx2=0, sumGy2=0, sumGz2=0;

        for (uint16_t i = 0; i < samples; i++) {
            // 가속도(6B) + 자이로(6B) = 12바이트
            uint8_t buf[12];
            readRegs(ICM_ACCEL_DATA_X1, buf, 12);

            int16_t gx = (int16_t)((buf[6] << 8) | buf[7]);
            int16_t gy = (int16_t)((buf[8] << 8) | buf[9]);
            int16_t gz = (int16_t)((buf[10]<< 8) | buf[11]);

            sumAx += (int16_t)((buf[0] << 8) | buf[1]);
            sumAy += (int16_t)((buf[2] << 8) | buf[3]);
            sumAz += (int16_t)((buf[4] << 8) | buf[5]);
            sumGx += gx; sumGy += gy; sumGz += gz;
            sumGx2 += (int64_t)gx*gx; sumGy2 += (int64_t)gy*gy; sumGz2 += (int64_t)gz*gz;

            delayMicroseconds(1000);  // 1kHz ODR 동기화 (1ms)
        }

        float meanGx = (float)sumGx/samples, meanGy = (float)sumGy/samples, meanGz = (float)sumGz/samples;
        float varGx = (float)sumGx2/samples - meanGx*meanGx;
        float varGy = (float)sumGy2/samples - meanGy*meanGy;
        float varGz = (float)sumGz2/samples - meanGz*meanGz;
        float stdLSB = sqrtf(max(varGx, max(varGy, varGz)));
        float stdDps = stdLSB * gyroScale_ * RAD_TO_DEG;

        if (stdDps > GYRO_CAL_STDDEV_THRESHOLD_DPS) {
            Serial.printf("[ICM42688P] 캘리브레이션 중 흔들림 감지 (표준편차 %.2f deg/s) → 재시도 %d/%d\n",
                          stdDps, attempt, GYRO_CAL_MAX_RETRY);
            delay(500);
            continue;
        }

        // 가속도: X/Y → 0 목표, Z → +1g 목표
        int16_t oneG = (int16_t)(GRAVITY_MS2 / accelScale_);  // ±8g에서 ≈4096
        cal_.axOfs = (int16_t)(sumAx / samples);
        cal_.ayOfs = (int16_t)(sumAy / samples);
        cal_.azOfs = (int16_t)(sumAz / samples) - oneG;
        cal_.gxOfs = (int16_t)meanGx;
        cal_.gyOfs = (int16_t)meanGy;
        cal_.gzOfs = (int16_t)meanGz;

        Serial.printf("[ICM42688P] 캘리브레이션 안정성 확인 완료 (표준편차 %.2f deg/s)\n", stdDps);
        saveCalibration();
        Serial.println("[ICM42688P 캘리브레이션 완료] ──────────────────────");
        Serial.printf("  가속도(acc): axOfs=%6d  ayOfs=%6d  azOfs=%6d\n",
                      cal_.axOfs, cal_.ayOfs, cal_.azOfs);
        Serial.printf("  자이로(gyr): gxOfs=%6d  gyOfs=%6d  gzOfs=%6d\n",
                      cal_.gxOfs, cal_.gyOfs, cal_.gzOfs);
        Serial.println("────────────────────────────────────────────────────");
        return;
    }

    Serial.println("[ICM42688P][경고] 최대 재시도 후에도 흔들림 지속 — 마지막 측정값으로 저장합니다");
    saveCalibration();
}

// ============================================================================
//  NVS 저장/로드
// ============================================================================
bool IMU_ICM42688P::loadCalibration()
{
    Preferences prefs;
    prefs.begin(NVS_NS_ICM, true);  // 읽기 전용
    bool ok = prefs.isKey("axOfs");
    if (ok) {
        cal_.axOfs = prefs.getShort("axOfs", 0);
        cal_.ayOfs = prefs.getShort("ayOfs", 0);
        cal_.azOfs = prefs.getShort("azOfs", 0);
        cal_.gxOfs = prefs.getShort("gxOfs", 0);
        cal_.gyOfs = prefs.getShort("gyOfs", 0);
        cal_.gzOfs = prefs.getShort("gzOfs", 0);
        Serial.printf("[ICM42688P] NVS 캘리브레이션 로드: "
                      "ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                      cal_.axOfs, cal_.ayOfs, cal_.azOfs,
                      cal_.gxOfs, cal_.gyOfs, cal_.gzOfs);
    } else {
        Serial.println("[ICM42688P] NVS 캘리브레이션 없음 → CH8로 캘리브레이션 실행 권장");
    }
    prefs.end();
    return ok;
}

bool IMU_ICM42688P::saveCalibration()
{
    // ★ Betaflight 분석 적용 (PG 변경감지 해시 패턴): 값이 실제로 바뀐
    // 경우에만 NVS에 쓴다. 플래시 쓰기 수명(통상 10만회) 보호.
    Preferences prefs;
    prefs.begin(NVS_NS_ICM, true);
    bool hasExisting = prefs.isKey("axOfs");
    bool unchanged = false;
    if (hasExisting) {
        unchanged = (prefs.getShort("axOfs", 0) == cal_.axOfs) &&
                    (prefs.getShort("ayOfs", 0) == cal_.ayOfs) &&
                    (prefs.getShort("azOfs", 0) == cal_.azOfs) &&
                    (prefs.getShort("gxOfs", 0) == cal_.gxOfs) &&
                    (prefs.getShort("gyOfs", 0) == cal_.gyOfs) &&
                    (prefs.getShort("gzOfs", 0) == cal_.gzOfs);
    }
    prefs.end();

    if (unchanged) {
        Serial.println("[ICM42688P] 캘리브레이션 값 변경 없음 → NVS 쓰기 생략");
        return true;
    }

    prefs.begin(NVS_NS_ICM, false);  // 읽기-쓰기
    prefs.putShort("axOfs", cal_.axOfs);
    prefs.putShort("ayOfs", cal_.ayOfs);
    prefs.putShort("azOfs", cal_.azOfs);
    prefs.putShort("gxOfs", cal_.gxOfs);
    prefs.putShort("gyOfs", cal_.gyOfs);
    prefs.putShort("gzOfs", cal_.gzOfs);
    prefs.end();
    return true;
}

// ============================================================================
//  SPI 저수준 함수
// ============================================================================
void IMU_ICM42688P::writeReg(uint8_t reg, uint8_t val)
{
    spi_->beginTransaction(SPISettings(spiClock_, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin_, LOW);
    spi_->transfer(reg & 0x7F);   // bit7 = 0: 쓰기
    spi_->transfer(val);
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
}

uint8_t IMU_ICM42688P::readReg(uint8_t reg)
{
    spi_->beginTransaction(SPISettings(spiClock_, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin_, LOW);
    spi_->transfer(reg | 0x80);   // bit7 = 1: 읽기
    uint8_t val = spi_->transfer(0x00);
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    return val;
}

void IMU_ICM42688P::readRegs(uint8_t startReg, uint8_t* buf, uint8_t len)
{
    spi_->beginTransaction(SPISettings(spiClock_, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin_, LOW);
    spi_->transfer(startReg | 0x80);   // bit7 = 1: 읽기
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_->transfer(0x00);
    }
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
}

#endif // IMU_SELECT_ICM42688P
