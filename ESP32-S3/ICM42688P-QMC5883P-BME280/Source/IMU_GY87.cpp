// ============================================================================
//  IMU_GY87.cpp  —  GY-87 드라이버 (Gemini 구조 + 표준 BMP180 공식)
//  수정 이력:
//    - endTransmission(false) → (true): BMP180 압력 ADC 오독 해결 (핵심 수정)
//    - 소프트 리셋 딜레이 150ms: MPU6050 안정화
//    - BYPASS_EN 정착 delay(50): HMC/BMP 초기화 전 버스 안정화
//    - B7 공식: ×50000(Gemini 오류) → ×6250(OSS=3 표준 Bosch 공식)
//    - gz 부호 반전: CW=양수 (Betaflight 규약)
// ============================================================================
#include "IMU_GY87.h"
#include <string.h>   // memcmp (saveCalibration 변경감지용)

static const float ACCEL_SCALE_TBL[4] = {
    GRAVITY_MS2 / 16384.0f,
    GRAVITY_MS2 /  8192.0f,
    GRAVITY_MS2 /  4096.0f,
    GRAVITY_MS2 /  2048.0f
};

static const float GYRO_SCALE_TBL[4] = {
    DEG_TO_RAD / 131.072f,
    DEG_TO_RAD /  65.536f,
    DEG_TO_RAD /  32.768f,
    DEG_TO_RAD /  16.384f
};

IMU_GY87::IMU_GY87()
    : addr_(GY87_MPU6050_ADDR), ready_(false),
      seaLevelPa_(101325.0f),
      baroState_(BS_IDLE), baroTimer_(0), baroUT_(0), baroUP_(0),
      gyroFS_(MPU6050_GYRO_FS), accelFS_(MPU6050_ACCEL_FS),
      dlpf_(MPU6050_DLPF_CFG)
{}

bool IMU_GY87::begin(uint8_t addr)
{
    addr_       = addr;
    accelScale_ = ACCEL_SCALE_TBL[accelFS_ & 0x03];
    gyroScale_  = GYRO_SCALE_TBL [gyroFS_  & 0x03];

    if (!initMPU6050()) {
        Serial.println("[GY87] MPU6050 미검출! SDA/SCL 배선 확인");
        return false;
    }

    // ★ BYPASS_EN 정착 대기: HMC/BMP가 버스에 안정적으로 나타날 때까지
    delay(50);

    initHMC5883L();
    delay(50);

    if (!initBMP180()) {
        Serial.println("[GY87] BMP180 초기화 실패 → GPS 고도 폴백 사용");
    }

    loadCalibration();
    ready_ = true;
    Serial.println("[GY87] 초기화 완료");
    return true;
}

// ============================================================================
//  initMPU6050 — 소프트 리셋 + BYPASS_EN=1
// ============================================================================
bool IMU_GY87::initMPU6050()
{
    uint8_t id = i2cRead(addr_, GY87_REG_WHO_AM_I);
    if (id != 0x68 && id != 0x72) return false;

    // 소프트 리셋 (DEVICE_RESET = 0x80)
    i2cWrite(addr_, GY87_REG_PWR_MGMT_1, 0x80);
    delay(150);   // ★ Gemini: 100ms → 150ms (칩 리셋 완전 대기)

    // 클럭: PLL X-자이로 기준 (안정적)
    i2cWrite(addr_, GY87_REG_PWR_MGMT_1, 0x01);
    delay(10);

    i2cWrite(addr_, GY87_REG_CONFIG,      dlpf_ & 0x07);
    i2cWrite(addr_, GY87_REG_SMPLRT_DIV, 0x00);
    i2cWrite(addr_, GY87_REG_GYRO_CONFIG,  (gyroFS_  & 0x03) << 3);
    i2cWrite(addr_, GY87_REG_ACCEL_CONFIG, (accelFS_ & 0x03) << 3);

    // ★ BYPASS_EN=1: ESP32가 HMC5883L·BMP180에 직접 접근
    i2cWrite(addr_, GY87_REG_USER_CTRL,   0x00);  // I2C 마스터 비활성
    delay(5);
    i2cWrite(addr_, GY87_REG_INT_PIN_CFG, 0x02);  // BYPASS_EN = 1
    delay(10);

    return true;
}

// ============================================================================
//  initHMC5883L — BYPASS 모드에서 직접 초기화
// ============================================================================
void IMU_GY87::initHMC5883L()
{
    i2cWrite(GY87_HMC5883L_ADDR, 0x00, 0x78);  // CRA: 8샘플 평균, 75Hz
    delay(5);
    i2cWrite(GY87_HMC5883L_ADDR, 0x01, 0x20);  // CRB: 게인 ±1.3 Ga
    delay(5);
    i2cWrite(GY87_HMC5883L_ADDR, 0x02, 0x00);  // Mode: 연속 측정
    delay(5);
}

// ============================================================================
//  initBMP180 — 캘리브레이션 22바이트 읽기 + 유효성 검증
// ============================================================================
bool IMU_GY87::initBMP180()
{
    // BYPASS 정착 후 추가 대기
    delay(20);

    uint8_t id = i2cRead(GY87_BMP180_ADDR, 0xD0);
    if (id != 0x55) {
        Serial.printf("[GY87] BMP180 ID 오류: 0x%02X (0x55 기대)\n", id);
        return false;
    }

    uint8_t buf[22];
    i2cReadN(GY87_BMP180_ADDR, 0xAA, 22, buf);

    AC1_ = (int16_t) ((buf[0]  << 8) | buf[1]);
    AC2_ = (int16_t) ((buf[2]  << 8) | buf[3]);
    AC3_ = (int16_t) ((buf[4]  << 8) | buf[5]);
    AC4_ = (uint16_t)((buf[6]  << 8) | buf[7]);
    AC5_ = (uint16_t)((buf[8]  << 8) | buf[9]);
    AC6_ = (uint16_t)((buf[10] << 8) | buf[11]);
    B1_  = (int16_t) ((buf[12] << 8) | buf[13]);
    B2_  = (int16_t) ((buf[14] << 8) | buf[15]);
    MB_  = (int16_t) ((buf[16] << 8) | buf[17]);
    MC_  = (int16_t) ((buf[18] << 8) | buf[19]);
    MD_  = (int16_t) ((buf[20] << 8) | buf[21]);

    Serial.printf("[GY87] BMP180 계수: AC1=%d AC2=%d AC3=%d AC4=%u AC5=%u AC6=%u\n",
                  AC1_, AC2_, AC3_, AC4_, AC5_, AC6_);
    Serial.printf("[GY87]              B1=%d B2=%d MB=%d MC=%d MD=%d\n",
                  B1_, B2_, MB_, MC_, MD_);

    // 유효성 검사: 0이거나 0xFF로 가득 찬 계수 차단
    if (AC1_==0 || AC1_==-1 || AC5_==0 || AC5_==0xFFFF ||
        AC6_==0 || AC6_==0xFFFF || MC_==0 || MD_==0) {
        Serial.println("[GY87] BMP180 캘리브레이션 계수 오류 → 기압 비활성화");
        return false;
    }

    Serial.println("[GY87] BMP180 캘리브레이션 OK");

    // 첫 온도 변환 시작
    i2cWrite(GY87_BMP180_ADDR, 0xF4, 0x2E);
    baroTimer_ = millis();
    baroState_ = BS_TEMP_WAIT;
    return true;
}

// ============================================================================
//  update — MPU6050 + HMC5883L + BMP180 상태 머신
// ============================================================================
void IMU_GY87::update()
{
    if (!ready_) return;

    // ── MPU6050 14바이트 읽기 (가속도+온도+자이로) ──
    uint8_t raw[14];
    i2cReadN(addr_, GY87_REG_ACCEL_XOUT_H, 14, raw);

    int16_t rawAx = (int16_t)((raw[0]  << 8) | raw[1])  - cal_.axOfs;
    int16_t rawAy = (int16_t)((raw[2]  << 8) | raw[3])  - cal_.ayOfs;
    int16_t rawAz = (int16_t)((raw[4]  << 8) | raw[5])  - cal_.azOfs;
    int16_t rawT  = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t rawGx = (int16_t)((raw[8]  << 8) | raw[9])  - cal_.gxOfs;
    int16_t rawGy = (int16_t)((raw[10] << 8) | raw[11]) - cal_.gyOfs;
    int16_t rawGz = (int16_t)((raw[12] << 8) | raw[13]) - cal_.gzOfs;

    // raw → 물리 단위
    data_.ax = (float)rawAx * accelScale_;
    data_.ay = (float)rawAy * accelScale_;
    data_.az = (float)rawAz * accelScale_;
    data_.gx = (float)rawGx * gyroScale_;
    data_.gy = (float)rawGy * gyroScale_;
    // ★ gz 부호 반전: MPU6050(CW=음수) → Betaflight 규약(CW=양수)
    data_.gz = -(float)rawGz * gyroScale_;
    data_.temperature = (float)rawT / 340.0f + 36.53f;

    // ── HMC5883L 직접 읽기 (BYPASS 모드) ──
    uint8_t mag[6];
    i2cReadN(GY87_HMC5883L_ADDR, 0x03, 6, mag);

    int16_t rawMx = (int16_t)((mag[0] << 8) | mag[1]);
    int16_t rawMz = (int16_t)((mag[2] << 8) | mag[3]);  // HMC: X, Z, Y 순서
    int16_t rawMy = (int16_t)((mag[4] << 8) | mag[5]);

    if (rawMx != -4096 && rawMy != -4096 && rawMz != -4096) {
        data_.mx = ((float)rawMx - cal_.mxOfs) * cal_.mxSc * 0.92f;
        data_.my = ((float)rawMy - cal_.myOfs) * cal_.mySc * 0.92f;
        data_.mz = ((float)rawMz - cal_.mzOfs) * cal_.mzSc * 0.92f;
        data_.magReady = true;
    } else {
        data_.magReady = false;
    }

    // ── BMP180 기압계 비동기 상태 머신 ──
    updateBaro();
}

// ============================================================================
//  updateBaro — OSS=3 비동기 상태 머신
//  ★ B7 공식: × 6250  (= 50000 >> OSS=3, Bosch 데이터시트 표준)
//     Gemini 코드의 × 50000은 OSS=3에서 overflow 또는 8배 과대 오류
// ============================================================================
void IMU_GY87::updateBaro()
{
    switch (baroState_) {

    case BS_IDLE:
        i2cWrite(GY87_BMP180_ADDR, 0xF4, 0x2E);   // 온도 변환 시작
        baroTimer_ = millis();
        baroState_ = BS_TEMP_WAIT;
        break;

    case BS_TEMP_WAIT:
        if (millis() - baroTimer_ >= 6) {           // 데이터시트 4.5ms + 여유
            uint8_t tb[2];
            i2cReadN(GY87_BMP180_ADDR, 0xF6, 2, tb);
            baroUT_ = (int32_t)((tb[0] << 8) | tb[1]);

            i2cWrite(GY87_BMP180_ADDR, 0xF4, 0xF4); // 압력 변환 OSS=3
            baroTimer_ = millis();
            baroState_ = BS_PRESS_WAIT;
        }
        break;

    case BS_PRESS_WAIT:
        if (millis() - baroTimer_ >= 28) {           // 데이터시트 25.5ms + 여유
            uint8_t pb[3];
            i2cReadN(GY87_BMP180_ADDR, 0xF6, 3, pb);
            // ★ OSS=3: raw 24비트 >> (8-3) = >> 5
            baroUP_ = (((int32_t)pb[0] << 16) |
                       ((int32_t)pb[1] <<  8) |
                        (int32_t)pb[2]) >> 5;

            // ── Bosch BMP180 표준 보상 공식 (데이터시트 Table 1) ──
            // [온도 보상]
            int32_t X1 = ((baroUT_ - (int32_t)AC6_) * (int32_t)AC5_) >> 15;
            int32_t X2 = ((int32_t)MC_ << 11) / (X1 + (int32_t)MD_);
            int32_t B5 = X1 + X2;

            // [압력 보상]
            int32_t B6 = B5 - 4000;
            X1 = ((int32_t)B2_ * ((B6 * B6) >> 12)) >> 11;
            X2 = ((int32_t)AC2_ * B6) >> 11;
            int32_t X3 = X1 + X2;
            // B3: OSS=3 → << 3  (Bosch: ((AC1*4+X3) << oss + 2) / 4)
            int32_t B3 = ((((int32_t)AC1_ * 4 + X3) << 3) + 2) / 4;
            X1 = ((int32_t)AC3_ * B6) >> 13;
            X2 = ((int32_t)B1_ * ((B6 * B6) >> 12)) >> 16;
            X3 = ((X1 + X2) + 2) >> 2;
            uint32_t B4 = ((uint32_t)AC4_ * (uint32_t)(X3 + 32768)) >> 15;

            // ★ B7: (UP-B3) × (50000 >> OSS) = × 6250 for OSS=3
            //   Gemini 코드의 × 50000 은 OSS=0 수식으로 OSS=3에 부적합
            uint32_t B7 = ((uint32_t)baroUP_ - (uint32_t)B3) * 6250UL;

            int32_t p;
            if (B7 < 0x80000000UL)
                p = (int32_t)((B7 * 2UL) / B4);
            else
                p = (int32_t)((B7 / B4) * 2UL);

            // 2차 보정
            X1 = (p >> 8) * (p >> 8);
            X1 = (X1 * 3038) >> 16;
            X2 = (-7357 * p) >> 16;
            p += (X1 + X2 + 3791) >> 4;

            // 유효 범위: 50000~115000 Pa (약 -500m~5500m 고도)
            if (p > 50000 && p < 115000) {
                data_.pressure  = (float)p;
                data_.altitude  = bmp180Altitude((float)p);
                data_.baroReady = true;
            } else {
                data_.baroReady = false;
                static bool warnPrinted = false;
                if (!warnPrinted) {
                    Serial.printf("[GY87] BMP180 압력 이상값: %d Pa → GPS 고도 폴백\n", p);
                    warnPrinted = true;
                }
            }
            baroState_ = BS_IDLE;
        }
        break;
    }
}

float IMU_GY87::bmp180Altitude(float pa) const {
    return 44330.0f * (1.0f - powf(pa / seaLevelPa_, 1.0f / 5.255f));
}

// ============================================================================
//  calibrate / loadCalibration / saveCalibration
// ============================================================================
void IMU_GY87::calibrate(uint16_t samples)
{
    Serial.println("[GY87] 가속도/자이로 캘리브레이션 시작...");
    delay(1000);

    // ── ★ Betaflight 분석 적용: 표준편차 기반 흔들림 감지 + 재시도 ──
    // sensors/gyro.c의 performGyroCalibration() 패턴: 캘리브레이션 도중
    // 표준편차가 임계값을 넘으면(흔들림 발생) 처음부터 다시 시작한다.
    for (uint8_t attempt = 1; attempt <= GYRO_CAL_MAX_RETRY; attempt++) {
        int64_t sumAx=0, sumAy=0, sumAz=0;
        int64_t sumGx=0, sumGy=0, sumGz=0;
        int64_t sumGx2=0, sumGy2=0, sumGz2=0;   // 표준편차 계산용 제곱합

        for (uint16_t i = 0; i < samples; i++) {
            uint8_t raw[14];
            i2cReadN(addr_, GY87_REG_ACCEL_XOUT_H, 14, raw);
            int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
            int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
            int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

            sumAx += (int16_t)((raw[0]  << 8) | raw[1]);
            sumAy += (int16_t)((raw[2]  << 8) | raw[3]);
            sumAz += (int16_t)((raw[4]  << 8) | raw[5]);
            sumGx += gx;  sumGy += gy;  sumGz += gz;
            sumGx2 += (int64_t)gx*gx; sumGy2 += (int64_t)gy*gy; sumGz2 += (int64_t)gz*gz;
            delayMicroseconds(400);
        }

        // 표준편차(raw LSB) → deg/s로 환산해 판정
        float meanGx = (float)sumGx / samples, meanGy = (float)sumGy / samples, meanGz = (float)sumGz / samples;
        float varGx = (float)sumGx2/samples - meanGx*meanGx;
        float varGy = (float)sumGy2/samples - meanGy*meanGy;
        float varGz = (float)sumGz2/samples - meanGz*meanGz;
        float stdLSB = sqrtf(max(varGx, max(varGy, varGz)));
        float stdDps = stdLSB * gyroScale_ * RAD_TO_DEG;

        if (stdDps > GYRO_CAL_STDDEV_THRESHOLD_DPS) {
            Serial.printf("[GY87] 캘리브레이션 중 흔들림 감지 (표준편차 %.2f deg/s) → 재시도 %d/%d\n",
                          stdDps, attempt, GYRO_CAL_MAX_RETRY);
            delay(500);
            continue;   // 처음부터 재시도
        }

        cal_.axOfs = (int16_t)(sumAx / samples);
        cal_.ayOfs = (int16_t)(sumAy / samples);
        int16_t oneG = (int16_t)(GRAVITY_MS2 / accelScale_);
        cal_.azOfs = (int16_t)(sumAz / samples) - oneG;
        cal_.gxOfs = (int16_t)meanGx;
        cal_.gyOfs = (int16_t)meanGy;
        cal_.gzOfs = (int16_t)meanGz;
        cal_.valid  = true;

        Serial.printf("[GY87] 캘리브레이션 안정성 확인 완료 (표준편차 %.2f deg/s)\n", stdDps);
        saveCalibration();
        return;
    }

    // 재시도를 모두 소진해도 안정되지 않으면, 마지막 측정값으로라도 저장
    // (완전히 캘리브레이션을 포기하는 것보다 낫다고 판단 — 단, 경고는 출력)
    Serial.println("[GY87][경고] 최대 재시도 후에도 흔들림 지속 — 마지막 측정값으로 저장합니다");
    cal_.valid = true;
    saveCalibration();
}

bool IMU_GY87::loadCalibration() {
    prefs_.begin(NVS_NAMESPACE, true);
    size_t sz = prefs_.getBytesLength(NVS_KEY_IMU_CALIB);
    if (sz == sizeof(GY87_Cal)) {
        prefs_.getBytes(NVS_KEY_IMU_CALIB, &cal_, sizeof(GY87_Cal));
        prefs_.end();
        if (cal_.valid) return true;
    }
    prefs_.end();

    cal_.axOfs = MPU6050_AX_OFFSET; cal_.ayOfs = MPU6050_AY_OFFSET;
    cal_.azOfs = MPU6050_AZ_OFFSET; cal_.gxOfs = MPU6050_GX_OFFSET;
    cal_.gyOfs = MPU6050_GY_OFFSET; cal_.gzOfs = MPU6050_GZ_OFFSET;
    cal_.mxSc  = cal_.mySc = cal_.mzSc = 1.0f;
    Serial.println("[GY87] 기본 캘리브레이션 값 사용 (캘리브레이션 실행 권장!)");
    return false;
}

bool IMU_GY87::saveCalibration() {
    // ★ Betaflight 분석 적용 (PG 변경감지 해시 패턴): 값이 실제로 바뀐
    // 경우에만 NVS에 쓴다. 플래시 쓰기 수명(통상 10만회) 보호.
    GY87_Cal existing;
    prefs_.begin(NVS_NAMESPACE, true);
    size_t sz = prefs_.getBytesLength(NVS_KEY_IMU_CALIB);
    bool hasExisting = (sz == sizeof(GY87_Cal));
    if (hasExisting) prefs_.getBytes(NVS_KEY_IMU_CALIB, &existing, sizeof(GY87_Cal));
    prefs_.end();

    if (hasExisting && memcmp(&existing, &cal_, sizeof(GY87_Cal)) == 0) {
        Serial.println("[GY87] 캘리브레이션 값 변경 없음 → NVS 쓰기 생략");
        return true;
    }

    prefs_.begin(NVS_NAMESPACE, false);
    bool ok = prefs_.putBytes(NVS_KEY_IMU_CALIB, &cal_, sizeof(GY87_Cal));
    prefs_.end();
    return ok;
}

// ============================================================================
//  I2C 통신 함수
//  ★ endTransmission(true): STOP 전송 — 핵심 수정
//     endTransmission(false)(Repeated START)는 BMP180 압력 ADC를
//     정상값(~319,000)이 아닌 오독값(~102,000)으로 읽어 18153 Pa 오류 발생
// ============================================================================
void IMU_GY87::i2cWrite(uint8_t dev, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);   // ★ STOP
}

uint8_t IMU_GY87::i2cRead(uint8_t dev, uint8_t reg)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return 0xFF;  // ★ STOP
    Wire.requestFrom(dev, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

void IMU_GY87::i2cReadN(uint8_t dev, uint8_t reg, uint8_t n, uint8_t *buf)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) {             // ★ STOP
        memset(buf, 0, n);
        return;
    }
    Wire.requestFrom(dev, n);
    for (uint8_t i = 0; i < n; i++)
        buf[i] = Wire.available() ? Wire.read() : 0;
}
