// ============================================================================
//  GPS_RTH.h  —  u-blox GPS 파서 + Return To Home 내비게이션  (v6)
//
//  ★ 지원 모듈
//  ─────────────────────────────────────────────────────────────────────────
//  u-blox NEO-6M : GPS 전용 (1Hz~5Hz)
//  u-blox NEO-7M : GPS+GLONASS (최대 10Hz)
//  u-blox NEO-8M : GPS+GLONASS+BeiDou (최대 10Hz)
//
//  모든 버전이 동일한 NMEA 0183 표준 프로토콜을 사용하므로
//  버전 구분 없이 동일 코드로 동작합니다.
//
//  ★ 연결 방법
//  ─────────────────────────────────────────────────────────────────────────
//  GPS TX핀 → ESP32 GPIO16 (PIN_GPS_RX, UART2 RX)
//  GPS RX핀 → ESP32 GPIO15 (PIN_GPS_TX, UART2 TX)
//  GPS 전원 → 3.3V 또는 5V (모듈 사양 확인)
//  GPS GND  → GND
//
//  ★ 사용하는 NMEA 문장
//  ─────────────────────────────────────────────────────────────────────────
//  $GPRMC : 필수! 위도·경도·속도·방위각·날짜 (GPS 기본 문장)
//  $GPGGA : 위도·경도·고도·위성 수·HDOP
//  $GPGSA : 수평정밀도(HDOP), 수직정밀도(VDOP) (Fix 품질 판단)
//
//  ★ RTH (Return To Home) 동작 순서
//  ─────────────────────────────────────────────────────────────────────────
//  1) 이륙 전 홈 위치 저장 (GPS Fix 확보 후 자동 저장)
//  2) RTH 명령 수신 (CH9 HIGH 또는 페일세이프)
//  3) 안전 고도(RTH_ASCEND_ALT_M)까지 상승
//  4) 홈 방향으로 수평 이동 (RTH_SPEED_MPS)
//  5) 홈 도달 판정 (RTH_RADIUS_M 이내)
//  6) 수직 하강 및 착륙
// ============================================================================
#pragma once
#ifndef GPS_RTH_H
#define GPS_RTH_H

#include <Arduino.h>
#include <Preferences.h>
#include "DroneConfig.h"

// ============================================================================
//  GPS Fix 품질 상수 (GGA 문장의 Fix Quality 필드)
// ============================================================================
#define GPS_FIX_INVALID    0   // Fix 없음 (위성 미수신)
#define GPS_FIX_GPS        1   // 일반 GPS Fix (3D)
#define GPS_FIX_DGPS       2   // DGPS 보정 Fix (더 정확)
#define GPS_FIX_PPS        3   // PPS Fix
#define GPS_FIX_RTK_FIXED  4   // RTK Fixed (센티미터급)
#define GPS_FIX_RTK_FLOAT  5   // RTK Float

// NMEA 최대 문장 길이
#define NMEA_MAX_LEN       90

// ============================================================================
//  GPS 데이터 구조체
// ============================================================================
struct GPS_Data {
    // ── 위치 ──
    double  lat      = 0.0;    // 위도 [도] (양수=북위, 음수=남위)
    double  lon      = 0.0;    // 경도 [도] (양수=동경, 음수=서경)
    float   altMsl   = 0.0f;   // 평균해수면(MSL) 기준 고도 [m]

    // ── 속도·방향 ──
    float   speedMps = 0.0f;   // 대지 속도 [m/s]
    float   courseDeg= 0.0f;   // 대지 방위각 [도] (북=0, 동=90)

    // ── Fix 상태 ──
    uint8_t fixQuality= 0;     // GPS_FIX_xxx 상수 참조
    uint8_t numSats   = 0;     // 사용 중인 위성 수
    float   hdop      = 99.9f; // 수평 정밀도 (낮을수록 좋음, <2.0 권장)

    // ── 시간 (UTC) ──
    uint8_t hour = 0, minute = 0, second = 0;
    uint8_t day  = 0, month  = 0;
    uint16_t year= 0;

    // ── 상태 플래그 ──
    bool    valid    = false;  // 유효한 Fix 여부 (위성+HDOP 조건 충족)
    bool    hasHome  = false;  // 홈 위치 저장 여부
    uint32_t lastFixMs = 0;   // 마지막 Fix 수신 시각 [ms]
};

// ============================================================================
//  RTH 상태 머신 상태 열거형
// ============================================================================
enum RTH_State {
    RTH_IDLE        = 0,   // 대기 중 (RTH 비활성)
    RTH_ASCENDING   = 1,   // 안전 고도까지 상승 중
    RTH_NAVIGATING  = 2,   // 홈 방향으로 수평 이동 중
    RTH_DESCENDING  = 3,   // 착륙 하강 중
    RTH_LANDED      = 4,   // 착륙 완료
    RTH_ERROR       = 5    // 오류 (GPS Fix 없음 등)
};

// ============================================================================
//  내비게이션 명령 구조체 (RTH에서 메인 루프로 전달)
// ============================================================================
struct NavCommand {
    float rollCmd   = 0.0f;    // Roll  목표 각도 [도]
    float pitchCmd  = 0.0f;    // Pitch 목표 각도 [도]
    float yawCmd    = 0.0f;    // Yaw   목표 방위각 [도]
    float throttleUs= 1500.0f; // 스로틀 [µs]
    RTH_State state = RTH_IDLE;
};

// ============================================================================
//  GPS_RTH 클래스
// ============================================================================
class GPS_RTH {
public:
    GPS_RTH();

    // 초기화 (Serial2 등 UART 포트 전달)
    void begin(HardwareSerial &serial, uint32_t baud = GPS_BAUD);

    // 매 루프 호출 — NMEA 파싱 처리
    void update();

    // ── GPS 데이터 접근 ──
    const GPS_Data& getData() const { return data_; }
    bool  hasValidFix()  const { return data_.valid; }
    double getLat()      const { return data_.lat;   }
    double getLon()      const { return data_.lon;   }
    float  getAlt()      const { return data_.altMsl;}
    uint8_t getNumSats() const { return data_.numSats; }
    float  getHDOP()     const { return data_.hdop;  }

    // ── 홈 위치 관리 ──
    // 이륙 전 GPS Fix 확보 후 자동 호출, 또는 수동 호출
    bool saveHome(float baroAltM);  // ★ BMP180 기압 고도를 홈 고도로 저장

    // NVS에서 홈 위치 불러오기 (이전 비행의 마지막 홈 위치)
    bool loadHome();

    // 홈 위치를 특정 좌표로 직접 설정
    void setHome(double lat, double lon, float alt);

    double getHomeLat() const { return homeLat_; }
    double getHomeLon() const { return homeLon_; }
    float  getHomeAlt() const { return homeAlt_; }
    bool   hasHome()    const { return hasHome_;  }

    // ── RTH 제어 ──
    void startRTH();   // RTH 시작
    void stopRTH();    // RTH 중단 (수동 조종 복귀)
    bool isRTHActive() const { return rthState_ != RTH_IDLE; }
    RTH_State getRTHState() const { return rthState_; }

    // RTH 내비게이션 계산 — 매 루프에서 호출
    // 반환값: 메인 루프에서 사용할 조종 명령
    // currentRoll/Pitch/Yaw: 현재 자세 [도]
    // currentAltBaro: 기압 고도 [m] (GPS Fix 있을 때는 GPS MSL 고도가 우선)
    NavCommand computeRTH(float currentRoll, float currentPitch,
                          float currentYaw,  float currentAltBaro,
                          float dt);

    // ── 거리·방향 계산 (유틸리티) ──
    // 두 GPS 좌표 사이의 거리 [m] (Haversine 공식)
    static float calcDistanceM(double lat1, double lon1,
                               double lat2, double lon2);
    // 좌표1 → 좌표2 방향 [도] (북=0, 동=90)
    static float calcBearingDeg(double lat1, double lon1,
                                double lat2, double lon2);
    // 방위각 차이 정규화 (-180 ~ +180)
    static float normalizeBearing(float bearing);

    // ── 텔레메트리 ──
    // CRSF GPS 텔레메트리 프레임 생성 (ELRS_Receiver에서 송신)
    void fillCRSF_GPS(uint8_t *payload, uint8_t &len) const;

private:
    HardwareSerial *ser_;
    GPS_Data        data_;
    Preferences     prefs_;

    // 홈 위치
    double  homeLat_;
    double  homeLon_;
    float   homeAlt_;
    bool    hasHome_;

    // NMEA 파서 상태
    char    nmeaBuf_[NMEA_MAX_LEN];  // 현재 수신 중인 문장
    uint8_t nmeaIdx_;                 // 버퍼 인덱스
    bool    inSentence_;              // 문장 수신 중 여부

    // RTH 상태 머신
    RTH_State rthState_;
    float     rthTargetAlt_;   // RTH 목표 고도 [m]
    uint32_t  rthTimer_;

    // NMEA 파싱 함수들
    void parseNMEA(const char *sentence);
    void parseGPRMC(const char *s);   // 위도·경도·속도·방위·날짜
    void parseGPGGA(const char *s);   // 위도·경도·고도·위성수·HDOP
    void parseGPGSA(const char *s);   // HDOP·VDOP (Fix 품질)

    // NMEA 유틸리티
    bool   verifyChecksum(const char *sentence);  // XOR 체크섬 검증
    double parseLatLon(const char *field, const char *dir);  // 도분→십진도
    float  parseFloat(const char *field);
    int    parseInt(const char *field);
    const char* nextField(const char *p);  // 다음 쉼표 필드로 이동

    // Fix 유효성 판정
    void updateValidity();
};

#endif // GPS_RTH_H
