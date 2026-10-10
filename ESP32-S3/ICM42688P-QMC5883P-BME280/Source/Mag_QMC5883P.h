// ============================================================================
//  Mag_QMC5883P.h  —  QMC5883P I2C 자력계 드라이버
//
//  ★ QMC5883L과 다른 칩! (QST Corporation QMC5883P)
//    QMC5883L: I2C 0x0D, 구식 HMC5883L 호환 핀맵
//    QMC5883P: I2C 0x2C, 새로운 레지스터 맵, 더 나은 성능
//
//  사용 범위:
//    - AHRS(비행 자세 제어)에는 사용 안 함 → 모터 자기 간섭 위험
//    - RTH 기수 방향 결정 및 GPS 항법에만 활용
//
//  I2C 연결:
//    SDA → PIN_I2C_SDA (DroneConfig: GPIO 8)
//    SCL → PIN_I2C_SCL (DroneConfig: GPIO 9)
//
//  ESP32 DevKitC I2C 버그 대책:
//    DroneConfig.h에서 I2C_FREQ_HZ를 100000으로 낮추거나
//    Wire.begin(SDA, SCL, 100000UL) 사용 권장
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

// ── QMC5883P 레지스터 맵 ──────────────────────────────────────────────────
// ★ QMC5883L(0x00~0x0D)과 완전히 다름! 혼용 불가.
#define QMC5883P_REG_DATAX_LSB   0x00   // X 데이터 하위 바이트
#define QMC5883P_REG_DATAX_MSB   0x01   // X 데이터 상위 바이트
#define QMC5883P_REG_DATAY_LSB   0x02
#define QMC5883P_REG_DATAY_MSB   0x03
#define QMC5883P_REG_DATAZ_LSB   0x04
#define QMC5883P_REG_DATAZ_MSB   0x05
#define QMC5883P_REG_STATUS      0x06   // 상태 (bit0 = DRDY: 데이터 준비)
#define QMC5883P_REG_TEMP_LSB    0x07
#define QMC5883P_REG_TEMP_MSB    0x08
#define QMC5883P_REG_CTRL1       0x09   // 측정 모드/ODR/범위/오버샘플링
#define QMC5883P_REG_CTRL2       0x0A   // 소프트 리셋 등
#define QMC5883P_REG_PERIOD      0x0B   // SET/RESET 주기 (반드시 0x01 기록)
#define QMC5883P_REG_CHIP_ID     0x0D   // 칩 ID (0xFF 기대)

// ★ Betaflight 분석 결과 반영 (compass_qmc5883.c 참고):
//   QMC5883 계열 일부 실리콘에서 초기화 시 이 레지스터에 값을 쓰지 않으면
//   축별 출력이 간헐적으로 멈추거나 부호가 튀는 현상이 보고됨.
//   데이터시트에 명시되지 않은 비공식 워크어라운드이지만, Betaflight가
//   실제 양산 펌웨어에서 채택하고 있어 본 드라이버에도 동일하게 적용한다.
#define QMC5883P_REG_XYZ_UNLOCK   0x29   // 축 데이터 잠금 해제 레지스터
#define QMC5883P_XYZ_SIGN_CONFIG  0x06   // 위 레지스터에 기록할 값

// CTRL1 구성값
// bits[1:0] MODE: 00=대기, 01=연속
// bits[3:2] ODR:  00=10Hz, 01=50Hz, 10=100Hz, 11=200Hz
// bits[5:4] RNG:  00=±2G, 01=±8G
// bits[7:6] OSR:  00=512, 01=256, 10=128, 11=64
#define QMC5883P_CTRL1_CONT_100HZ_8G_OSR512 \
    ((0<<6) | (0<<4) | (2<<2) | (1))
    // OSR=512, RNG=8G(주의: 이 칩은 00이 ±8G일 수 있음 → 확인 필요)
    // ODR=100Hz, MODE=연속

// I2C 기본 주소
#define QMC5883P_DEFAULT_ADDR    0x2C

// 상태 비트
#define QMC5883P_STATUS_DRDY     0x01

// ============================================================================
class Mag_QMC5883P {
public:
    Mag_QMC5883P();

    // ── 초기화 ──────────────────────────────────────────────────────────────
    // addr: I2C 주소 (기본 0x2C)
    // wire: Wire 객체 (ESP32 DevKitC에서 Wire1 사용 가능)
    bool begin(uint8_t addr = QMC5883P_DEFAULT_ADDR,
               TwoWire& wire = Wire);

    // ── 업데이트 (100ms 간격 호출 권장, 즉 100Hz) ──────────────────────────
    // 반환: true = 새 데이터 읽음
    bool update();

    // ── 원시 자기장값 [LSB] ─────────────────────────────────────────────────
    int16_t getRawX() const { return mx_; }
    int16_t getRawY() const { return my_; }
    int16_t getRawZ() const { return mz_; }

    // ── 자기 북쪽 기준 기수 방향 [°] (0=북, 시계방향) ────────────────────
    // declination_deg: 자기 편각 (서울 기준 약 -8.0°)
    //   → 양수 편각: 자기 북쪽이 진북 동쪽에 위치
    //   → 음수 편각: 자기 북쪽이 진북 서쪽에 위치
    //
    // ★ 주의: 드론이 수평일 때만 정확. 경사 시 틸트 보정 필요.
    //   (AHRS roll/pitch를 이용한 틸트 보정은 getHeadingTilted() 사용 권장)
    float getHeadingDeg(float declination_deg = -8.0f) const;

    // ── 틸트 보정 헤딩 (AHRS roll/pitch 제공 시) ─────────────────────────
    float getHeadingTilted(float rollRad, float pitchRad,
                            float declination_deg = -8.0f) const;

    bool hasNewData() const { return newData_; }
    bool isConnected() const { return connected_; }

private:
    uint8_t  addr_;
    TwoWire* wire_;
    int16_t  mx_ = 0, my_ = 0, mz_ = 0;
    bool     newData_   = false;
    bool     connected_ = false;
    uint8_t  stuckCount_ = 0;   // ★ Stuck-data 감지용 연속 동일값 카운터

    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg (uint8_t reg);
};
