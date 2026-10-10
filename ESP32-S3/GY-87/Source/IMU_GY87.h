// ============================================================================
//  IMU_GY87.h  —  GY-87 복합 IMU 모듈 드라이버  (v5)
//
//  ★ GY-87 모듈이란?
//  ─────────────────────────────────────────────────────────────────────────
//  하나의 작은 보드에 세 가지 센서가 통합된 모듈입니다:
//
//  1) MPU6050 — 자이로스코프 + 가속도계
//     - 자이로: 각속도 측정 (deg/s) — "얼마나 빨리 회전하는가"
//     - 가속도: 가속도 측정 (m/s²) — "어느 방향으로 힘이 가해지는가"
//     - I2C 주소: 0x68 (AD0=GND)
//
//  2) HMC5883L — 지자계(나침반)
//     - 지구 자기장을 측정하여 북쪽 방향(Yaw) 기준을 제공
//     - MPU6050의 I2C 보조 버스에 연결되어 자동으로 데이터를 읽어옴
//     - I2C 주소: 0x1E
//
//  3) BMP180 — 기압 센서
//     - 대기압으로 고도를 계산 (m)
//     - 변환에 시간이 걸려 비동기(Non-blocking) 방식으로 처리
//     - I2C 주소: 0x77
//
//  ★ I2C 연결 방식
//  ─────────────────────────────────────────────────────────────────────────
//  ESP32 ←→ MPU6050 (I2C 마스터 모드) ←→ HMC5883L (I2C 슬레이브)
//  ESP32 ←→ BMP180 (직접 I2C 통신)
//
//  MPU6050이 HMC5883L을 자동으로 읽어 EXT_SENS_DATA 레지스터에 저장합니다.
//  따라서 ESP32는 MPU6050만 읽어도 자이로+가속도+지자계 데이터를 얻습니다.
//
//  ★ 캘리브레이션
//  ─────────────────────────────────────────────────────────────────────────
//  센서마다 제조 편차가 있어 0 출력값이 정확히 0이 아닙니다.
//  이 오차(Offset)를 측정하여 보정하는 작업이 캘리브레이션입니다.
//  측정된 오프셋은 ESP32의 NVS(내부 플래시)에 저장됩니다.
//  조종기 AUX4 채널을 HIGH로 올리면 자동 캘리브레이션이 시작됩니다.
// ============================================================================
#pragma once
#ifndef IMU_GY87_H
#define IMU_GY87_H

#include <Arduino.h>
#include <Wire.h>          // I2C 통신 라이브러리
#include <Preferences.h>   // NVS 플래시 저장 라이브러리 (ESP32 전용)
#include "DroneConfig.h"

// ── I2C 주소 ──
#define GY87_MPU6050_ADDR  0x68   // MPU6050 기본 주소 (AD0=GND)
#define GY87_HMC5883L_ADDR 0x1E   // HMC5883L 지자계 주소
#define GY87_BMP180_ADDR   0x77   // BMP180 기압 센서 주소

// ── MPU6050 주요 레지스터 주소 ──
// 레지스터: 칩 내부의 설정·데이터 저장소 (8비트 주소로 접근)
#define GY87_REG_SMPLRT_DIV    0x19   // 샘플 레이트 분주기
#define GY87_REG_CONFIG        0x1A   // DLPF 설정
#define GY87_REG_GYRO_CONFIG   0x1B   // 자이로 풀 스케일 범위
#define GY87_REG_ACCEL_CONFIG  0x1C   // 가속도계 풀 스케일 범위
#define GY87_REG_I2C_MST_CTRL 0x24   // I2C 마스터 제어
#define GY87_REG_I2C_SLV0_ADDR 0x25  // 슬레이브 0 주소 (HMC5883L)
#define GY87_REG_I2C_SLV0_REG  0x26  // 슬레이브 0 레지스터
#define GY87_REG_I2C_SLV0_CTRL 0x27  // 슬레이브 0 제어
#define GY87_REG_INT_PIN_CFG   0x37   // 인터럽트 핀 설정 (I2C 바이패스)
#define GY87_REG_ACCEL_XOUT_H  0x3B   // 가속도계 X축 상위 바이트 (14바이트 버스트 시작)
#define GY87_REG_EXT_SENS_DATA 0x49   // 외부 센서 데이터 (HMC5883L 자동 저장 위치)
#define GY87_REG_USER_CTRL     0x6A   // I2C 마스터 모드 활성화
#define GY87_REG_PWR_MGMT_1    0x6B   // 전원 관리 / 리셋
#define GY87_REG_WHO_AM_I      0x75   // 칩 ID 확인 (MPU6050=0x68)

// ============================================================================
//  캘리브레이션 데이터 구조체
//  NVS에 이 구조체 전체를 저장/불러옵니다.
// ============================================================================
struct GY87_Cal {
    // 자이로·가속도계 오프셋 (정수 raw 값)
    int16_t axOfs = 0, ayOfs = 0, azOfs = 0;  // 가속도 오프셋
    int16_t gxOfs = 0, gyOfs = 0, gzOfs = 0;  // 자이로 오프셋

    // 지자계 하드 아이언(Hard-Iron) 보정
    // 주변 금속이 만드는 일정한 자기장 오프셋
    float mxOfs = 0, myOfs = 0, mzOfs = 0;

    // 지자계 소프트 아이언(Soft-Iron) 보정
    // 비등방성 왜곡 보정 스케일 (기본 1.0 = 보정 없음)
    float mxSc = 1, mySc = 1, mzSc = 1;

    bool valid = false;  // 유효한 캘리브레이션 데이터 여부
};

// ============================================================================
//  센서 출력 데이터 구조체
//  update() 호출 후 이 구조체에 최신 데이터가 채워집니다.
// ============================================================================
struct GY87_Data {
    float ax = 0, ay = 0, az = 0;  // 가속도계 [m/s²] — 오프셋·스케일 보정 후
    float gx = 0, gy = 0, gz = 0;  // 자이로스코프 [rad/s] — Mahony에 입력
    float mx = 0, my = 0, mz = 0;  // 지자계 [µT]
    float temperature = 0;          // MPU6050 내부 온도 [°C]
    float pressure    = 0;          // BMP180 기압 [Pa]
    float altitude    = 0;          // BMP180 고도 [m] (해수면 기준)
    bool  magReady  = false;  // HMC5883L 데이터 유효 여부
    bool  baroReady = false;  // BMP180 데이터 유효 여부
};

// ============================================================================
//  IMU_GY87 클래스
// ============================================================================
class IMU_GY87 {
public:
    IMU_GY87();

    // 초기화 — Wire.begin() 후 setup()에서 호출
    // 반환값: true=성공, false=MPU6050 미검출
    bool begin(uint8_t addr = GY87_MPU6050_ADDR);

    // 모든 센서 읽기 — 매 루프마다 호출 (약 200~300 µs 소요)
    void update();

    // IMU 캘리브레이션 — 드론을 수평 정지 상태로 놓고 호출
    // samples: 평균을 낼 샘플 수 (많을수록 정밀, 시간도 오래 걸림)
    void calibrate(uint16_t samples = 2000);

    // NVS에서 캘리브레이션 값 불러오기/저장하기
    bool loadCalibration();
    bool saveCalibration();

    // 데이터 접근
    const GY87_Data& getData() const { return data_; }
    const GY87_Cal&  getCalData() const { return cal_; }  // ★ 캘리브레이션 오프셋 반환
    float getAccelX() const { return data_.ax; }
    float getAccelY() const { return data_.ay; }
    float getAccelZ() const { return data_.az; }
    float getGyroX()  const { return data_.gx; }  // rad/s
    float getGyroY()  const { return data_.gy; }
    float getGyroZ()  const { return data_.gz; }
    float getMagX()   const { return data_.mx; }
    float getMagY()   const { return data_.my; }
    float getMagZ()   const { return data_.mz; }
    float getTemperature() const { return data_.temperature; }
    float getPressure()    const { return data_.pressure;    }
    float getAltitude()    const { return data_.altitude;    }

    // 해수면 기압 설정 (고도 계산 기준점)
    // 이륙 전 지상에서 측정한 기압으로 설정하면 상대 고도가 더 정확합니다
    void setSeaLevelPressure(float pa) { seaLevelPa_ = pa; }

    bool isReady() const { return ready_; }

    // 센서 범위 설정 (begin() 호출 전에 설정)
    void setGyroFS (uint8_t fs) { gyroFS_  = fs; }  // 0~3
    void setAccelFS(uint8_t fs) { accelFS_ = fs; }  // 0~3
    void setDLPF   (uint8_t bw) { dlpf_    = bw; }  // 0~6

private:
    uint8_t   addr_;       // MPU6050 I2C 주소
    bool      ready_;      // 초기화 성공 여부
    GY87_Data data_;       // 최신 센서 데이터
    GY87_Cal  cal_;        // 캘리브레이션 오프셋
    Preferences prefs_;    // NVS 접근 객체

    // 변환 스케일 팩터 (FS 설정에 따라 begin()에서 계산)
    float accelScale_;   // (m/s²) per raw LSB
    float gyroScale_;    // (rad/s) per raw LSB

    // BMP180 공장 캘리브레이션 계수 (데이터시트 AC1~MD)
    // 압력·온도 계산에 사용하는 센서별 고유 값
    int16_t  AC1_, AC2_, AC3_;
    uint16_t AC4_, AC5_, AC6_;
    int16_t  B1_, B2_, MB_, MC_, MD_;
    float    seaLevelPa_;  // 기준 해수면 기압 [Pa]

    // BMP180 비동기 상태 머신
    // 기압 측정은 수십 ms가 필요하므로 여러 루프에 나누어 처리합니다
    enum BaroState { BS_IDLE, BS_TEMP_WAIT, BS_PRESS_WAIT };
    BaroState baroState_;  // 현재 상태
    uint32_t  baroTimer_;  // 대기 시작 시간 (millis)
    int32_t   baroUT_;     // 미보정 온도 raw 값
    int32_t   baroUP_;     // 미보정 기압 raw 값

    uint8_t gyroFS_, accelFS_, dlpf_;  // 센서 설정값

    // I2C 헬퍼 함수 (Wire 라이브러리 래퍼)
    void    i2cWrite(uint8_t dev, uint8_t reg, uint8_t val);
    uint8_t i2cRead (uint8_t dev, uint8_t reg);
    void    i2cReadN(uint8_t dev, uint8_t reg, uint8_t n, uint8_t *buf);

    bool initMPU6050();     // MPU6050 초기화
    void initHMC5883L();    // HMC5883L 초기화 (MPU I2C 바이패스 이용)
    bool initBMP180();      // BMP180 초기화 및 교정값 읽기
    void updateBaro();      // BMP180 상태 머신 (매 루프에서 호출)
    float bmp180Altitude(float pa) const;  // 기압→고도 변환
};

#endif // IMU_GY87_H
