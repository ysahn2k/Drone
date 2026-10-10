// ============================================================================
//  Baro_BMP280.h  —  BMP280 기압 센서 독립 드라이버 (SPI 방식)  (v8)
//
//  ★ 이 파일이 독립 분리된 이유
//  ─────────────────────────────────────────────────────────────────────────
//  기압 센서는 IMU와 독립적으로 교체될 수 있는 부품이라 별도 파일로 분리했습니다.
//  예를 들어:
//    BMP280  → 일반적인 기압·온도 센서 (현재)
//    BMP388  → 더 정밀한 기압 센서
//    MS5611  → 드론 전용 고정밀 기압 센서
//    SPL06   → 소형 고정밀 기압 센서
//
//  분리하면 향후 교체 시 이 파일(Baro_BMP280)만 새 파일로 바꾸면 됩니다.
//  메인 코드는 수정할 필요가 없습니다.
//
//  ★ SPI 방식 사용 이유
//  ─────────────────────────────────────────────────────────────────────────
//  BMP280은 I2C(최대 3.4MHz)와 SPI(최대 10MHz) 모두 지원합니다.
//  SPI를 사용하면:
//    - 최대 10 MHz → I2C(400kHz)보다 25배 빠름
//    - ICM-42688-P와 같은 SPI 버스 공유 가능 (CS 핀으로 구분)
//    - I2C 버스 혼잡 없음
//
//  ★ SPI 통신 원리
//  ─────────────────────────────────────────────────────────────────────────
//  SPI(Serial Peripheral Interface)는 마스터-슬레이브 구조입니다.
//
//       ESP32-S3 (마스터)         BMP280 (슬레이브)
//  SCK  ──────────────────────→  SCK   (클럭: 데이터 동기화)
//  MOSI ──────────────────────→  SDI   (마스터→슬레이브 데이터)
//  MISO ←──────────────────────  SDO   (슬레이브→마스터 데이터)
//  CS   ──────────────────────→  CSB   (칩 선택: LOW=활성)
//
//  CS = LOW로 만든 후 클럭에 맞춰 데이터를 주고받습니다.
//
//  ★ BMP280 SPI 레지스터 접근 방식
//  ─────────────────────────────────────────────────────────────────────────
//  읽기: 주소 바이트의 MSB = 1  (0x80 OR)
//  쓰기: 주소 바이트의 MSB = 0  (그대로)
//
//  ★ 하드웨어 연결 (v8 기준)
//  ─────────────────────────────────────────────────────────────────────────
//  BMP280 CSB  → ESP32 GPIO14 (PIN_CS_BMP280)
//  BMP280 SCK  → ESP32 GPIO12 (PIN_SPI_SCK,  ICM-42688-P와 버스 공유)
//  BMP280 SDI  → ESP32 GPIO11 (PIN_SPI_MOSI, ICM-42688-P와 버스 공유)
//  BMP280 SDO  → ESP32 GPIO13 (PIN_SPI_MISO, ICM-42688-P와 버스 공유)
//  BMP280 VCC  → 3.3V
//  BMP280 GND  → GND
//
//  ★ 향후 다른 기압 센서로 교체 방법
//  ─────────────────────────────────────────────────────────────────────────
//  1) lib/ 에 새 폴더 생성 (예: Baro_MS5611/)
//  2) 동일한 인터페이스 함수 구현:
//     begin(), update(), getPressure(), getTemperature(), getAltitude()
//  3) QuadDrone_ESP32S3.ino에서 include 파일만 변경
//  4) DroneConfig.h의 USE_BARO_BMP280 → USE_BARO_MS5611 으로 변경
// ============================================================================
#pragma once
#ifndef BARO_BMP280_H
#define BARO_BMP280_H

#include <Arduino.h>
#include <SPI.h>           // ESP32 Arduino SPI 라이브러리 (기본 내장)
#include "DroneConfig.h"

// ── BMP280 레지스터 주소 ──
// (데이터시트 4.3 Register map 참고)
#define BMP280_REG_ID        0xD0   // 칩 ID 레지스터 (BMP280=0x60, BME280=0x58)
#define BMP280_REG_RESET     0xE0   // 소프트 리셋 (0xB6 쓰면 리셋)
#define BMP280_REG_STATUS    0xF3   // 측정 상태 (bit3=measuring)
#define BMP280_REG_CTRL_MEAS 0xF4   // 오버샘플링 + 동작 모드 설정
#define BMP280_REG_CONFIG    0xF5   // 대기시간 + IIR필터 + SPI3w 설정
#define BMP280_REG_PRESS_MSB 0xF7   // 기압 데이터 시작 (3바이트)
#define BMP280_REG_TEMP_MSB  0xFA   // 온도 데이터 시작 (3바이트)
#define BMP280_REG_CALIB     0x88   // 교정 계수 시작 (24바이트)

// SPI 읽기: 주소 MSB를 1로 설정
#define BMP280_SPI_READ(addr)  ((addr) | 0x80)
// SPI 쓰기: 주소 MSB를 0으로 설정
#define BMP280_SPI_WRITE(addr) ((addr) & 0x7F)

// ============================================================================
//  BMP280 교정 계수 구조체
//  ─────────────────────────────────────────────────────────────────────────
//  BMP280은 칩 제조 시 측정한 교정값을 내부 플래시에 저장합니다.
//  이 값들을 이용해 raw ADC 값 → 실제 온도/기압으로 변환합니다.
//  (칩마다 다른 고유값, 한 번만 읽으면 됩니다)
// ============================================================================
struct BMP280_Cal {
    uint16_t dig_T1;                  // 온도 교정 계수 1 (unsigned)
    int16_t  dig_T2, dig_T3;          // 온도 교정 계수 2, 3
    uint16_t dig_P1;                  // 기압 교정 계수 1 (unsigned)
    int16_t  dig_P2, dig_P3, dig_P4;  // 기압 교정 계수 2~4
    int16_t  dig_P5, dig_P6, dig_P7;  // 기압 교정 계수 5~7
    int16_t  dig_P8, dig_P9;          // 기압 교정 계수 8~9
};

// ============================================================================
//  Baro_BMP280 클래스
// ============================================================================
class Baro_BMP280 {
public:
    Baro_BMP280();

    // ── 초기화 ──
    // csPin      : CS(칩 선택) 핀 번호 (PIN_CS_BMP280)
    // spiFreqHz  : SPI 통신 속도 (기본 10 MHz)
    // spiInst    : SPI 버스 인스턴스 (기본 SPI = SPI2/HSPI)
    // 반환값     : true=성공, false=BMP280 미검출
    bool begin(int csPin        = PIN_CS_BMP280,
               uint32_t spiFreqHz = SPI_FREQ_BMP280,
               SPIClass &spiInst  = SPI);

    // ── 매 루프 호출 — 기압·온도 읽기 및 고도 계산 ──
    void update();

    // ── 데이터 접근 ──
    float getTemperature() const { return temperature_; }  // [°C]
    float getPressure()    const { return pressure_;    }  // [Pa]
    float getAltitude()    const { return altitude_;    }  // [m]
    bool  isReady()        const { return ready_;       }

    // 해수면 기압 설정 (이륙 전 지상 기압으로 설정하면 상대 고도 정확)
    // 기본값: 101325 Pa (표준 대기압)
    void  setSeaLevelPressure(float pa) { seaLevelPa_ = pa; }
    float getSeaLevelPressure() const   { return seaLevelPa_; }

    // 현재 기압을 해수면 기압으로 설정 (이륙 전 지상에서 호출)
    void calibrateSeaLevel() { seaLevelPa_ = pressure_; }

private:
    int       csPin_;          // CS 핀 번호
    SPIClass *spi_;            // SPI 버스 포인터
    uint32_t  spiFreq_;        // SPI 통신 속도
    bool      ready_;          // 초기화 성공 여부

    float temperature_;        // 측정 온도 [°C]
    float pressure_;           // 측정 기압 [Pa]
    float altitude_;           // 계산 고도 [m]
    float seaLevelPa_;         // 해수면 기압 기준값 [Pa]

    // BMP280 교정 계수 (begin()에서 읽어 저장)
    BMP280_Cal cal_;

    // 온도 보정 내부 변수 (기압 보정에 필요)
    int32_t t_fine_;

    // ── SPI 저수준 통신 함수 ──
    // 1바이트 쓰기
    void     spiWrite(uint8_t reg, uint8_t val);
    // 1바이트 읽기
    uint8_t  spiRead (uint8_t reg);
    // n바이트 연속 읽기 (버스트 읽기)
    void     spiReadN(uint8_t reg, uint8_t n, uint8_t *buf);

    // ── 초기화 서브 함수 ──
    bool readCalibration();   // 교정 계수 읽기
    void configure();         // 오버샘플링·필터 설정

    // ── 데이터 변환 (Bosch BMP280 데이터시트 공식) ──
    // raw ADC 값 → 실제 온도 [0.01°C 단위]
    int32_t  compensateTemp (int32_t adc_T);
    // raw ADC 값 → 실제 기압 [Pa×256, Q24.8 고정소수점]
    uint32_t compensatePress(int32_t adc_P);

    // 기압 → 고도 변환 (국제 표준 기압 공식)
    float pressureToAltitude(float pressurePa) const;
};

#endif // BARO_BMP280_H
