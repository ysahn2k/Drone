// ============================================================================
//  Mag_QMC5883P.cpp  —  QMC5883P I2C 자력계 드라이버 구현
// ============================================================================
#include "Mag_QMC5883P.h"
#include <math.h>

Mag_QMC5883P::Mag_QMC5883P()
    : addr_(QMC5883P_DEFAULT_ADDR), wire_(&Wire) {}

// ============================================================================
//  begin() — I2C 초기화 및 센서 설정
// ============================================================================
bool Mag_QMC5883P::begin(uint8_t addr, TwoWire& wire)
{
    addr_  = addr;
    wire_  = &wire;

    // ── 칩 연결 확인 ──────────────────────────────────────────────────────
    uint8_t chipId = readReg(QMC5883P_REG_CHIP_ID);

    // ★ QMC5883P 칩 ID: 0xFF 기대 (제조사/버전에 따라 다를 수 있음)
    // 다른 값이 나오면 경고만 출력하고 계속 진행
    if (chipId == 0xFF || chipId != 0x00) {
        Serial.printf("[QMC5883P] 칩 ID: 0x%02X %s\n",
                      chipId,
                      chipId == 0xFF ? "(정상)" : "(예상과 다름, 계속 진행)");
    } else {
        Serial.printf("[QMC5883P] I2C 0x%02X 응답 없음 (SDA/SCL 배선 확인)\n", addr_);
        connected_ = false;
        return false;
    }

    // ── 소프트 리셋 ───────────────────────────────────────────────────────
    writeReg(QMC5883P_REG_CTRL2, 0x80);
    delay(20);

    // ── SET/RESET 주기 설정 (반드시 필요! 누락 시 데이터 신뢰 불가) ──────
    // QMC5883P 데이터시트: SET/RESET period = 0x01
    writeReg(QMC5883P_REG_PERIOD, 0x01);

    // ── ★ XYZ_UNLOCK 워크어라운드 (Betaflight compass_qmc5883.c 참고) ──────
    // 일부 QMC5883 계열 실리콘에서 누락 시 축 데이터가 간헐적으로 멈추거나
    // 부호가 튀는 현상이 보고됨. RTH 헤딩처럼 가끔씩만 읽는 용도에서는
    // 증상이 드물게 나타나 발견이 어려우므로 사전에 적용해둔다.
    writeReg(QMC5883P_REG_XYZ_UNLOCK, QMC5883P_XYZ_SIGN_CONFIG);

    // ── 측정 모드 설정 ────────────────────────────────────────────────────
    // 연속 측정, ODR=100Hz, 범위=±8G, 오버샘플링=512
    // ★ 주의: QMC5883P의 범위 비트는 QMC5883L과 다를 수 있음
    //         실제 모듈 테스트 후 범위 비트 조정 필요
    writeReg(QMC5883P_REG_CTRL1, QMC5883P_CTRL1_CONT_100HZ_8G_OSR512);

    delay(20);
    connected_ = true;
    Serial.printf("[QMC5883P] 초기화 완료 (I2C 0x%02X, 100Hz, ±8G)\n", addr_);
    Serial.println("  [주의] 자력계는 RTH/GPS 항법에만 사용 (AHRS 제외)");
    return true;
}

// ============================================================================
//  update() — 새 데이터가 있으면 읽기
// ============================================================================
bool Mag_QMC5883P::update()
{
    if (!connected_) { newData_ = false; return false; }

    // 데이터 준비(DRDY) 비트 확인
    uint8_t status = readReg(QMC5883P_REG_STATUS);
    if (!(status & QMC5883P_STATUS_DRDY)) {
        newData_ = false;
        return false;
    }

    // 6바이트 버스트 읽기 (Little-Endian: LSB 먼저)
    wire_->beginTransmission(addr_);
    wire_->write(QMC5883P_REG_DATAX_LSB);
    if (wire_->endTransmission(false) != 0) {
        newData_ = false;
        return false;
    }

    uint8_t received = wire_->requestFrom(addr_, (uint8_t)6);
    if (received < 6) {
        newData_ = false;
        return false;
    }

    uint8_t buf[6];
    for (int i = 0; i < 6; i++) buf[i] = wire_->read();

    // QMC5883P는 Little-Endian (LSB 먼저)
    int16_t newX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t newY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t newZ = (int16_t)((buf[5] << 8) | buf[4]);

    // ── ★ Stuck-data 감지 (Betaflight compass_qmc5883.c 참고) ────────────
    // DRDY 비트는 정상적으로 서는데 실제 값이 갱신되지 않는 증상에 대응.
    // 10회 연속 동일값이면 센서가 내부적으로 멈춘 것으로 간주, 소프트 리셋.
    if (newX == mx_ && newY == my_ && newZ == mz_) {
        if (++stuckCount_ > 10) {
            stuckCount_ = 0;
            Serial.println("[QMC5883P] Stuck-data 감지 → 소프트 리셋");
            writeReg(QMC5883P_REG_CTRL2, 0x80);
            delay(20);
            writeReg(QMC5883P_REG_PERIOD, 0x01);
            writeReg(QMC5883P_REG_XYZ_UNLOCK, QMC5883P_XYZ_SIGN_CONFIG);
            writeReg(QMC5883P_REG_CTRL1, QMC5883P_CTRL1_CONT_100HZ_8G_OSR512);
            newData_ = false;
            return false;
        }
    } else {
        stuckCount_ = 0;
    }

    mx_ = newX; my_ = newY; mz_ = newZ;

    newData_ = true;
    return true;
}

// ============================================================================
//  getHeadingDeg() — 수평 기준 기수 방향 계산
// ============================================================================
float Mag_QMC5883P::getHeadingDeg(float declination_deg) const
{
    // XY 평면 헤딩 (드론이 수평일 때 유효)
    // ★ 축 배치는 실제 모듈 장착 방향에 따라 mx_/my_ 부호/순서 조정 필요
    float heading = atan2f((float)my_, (float)mx_) * (180.0f / (float)M_PI);
    heading += declination_deg;

    // 0~360° 범위로 정규화
    if (heading < 0.0f)    heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;

    return heading;
}

// ============================================================================
//  getHeadingTilted() — 틸트 보정 헤딩 (AHRS roll/pitch 활용)
// ============================================================================
float Mag_QMC5883P::getHeadingTilted(float rollRad, float pitchRad,
                                       float declination_deg) const
{
    // 자기 벡터 틸트 보정 (Tilt-compensated heading)
    float cosR = cosf(rollRad),  sinR = sinf(rollRad);
    float cosP = cosf(pitchRad), sinP = sinf(pitchRad);

    // 수평면으로 투영
    float Xh = (float)mx_ * cosP
              + (float)my_ * sinR * sinP
              + (float)mz_ * cosR * sinP;

    float Yh = (float)my_ * cosR
              - (float)mz_ * sinR;

    float heading = atan2f(-Yh, Xh) * (180.0f / (float)M_PI);
    heading += declination_deg;

    if (heading < 0.0f)    heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;

    return heading;
}

// ============================================================================
//  I2C 저수준 함수
// ============================================================================
void Mag_QMC5883P::writeReg(uint8_t reg, uint8_t val)
{
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(val);
    wire_->endTransmission();
}

uint8_t Mag_QMC5883P::readReg(uint8_t reg)
{
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return 0xFF;
    wire_->requestFrom(addr_, (uint8_t)1);
    return wire_->available() ? wire_->read() : 0xFF;
}
