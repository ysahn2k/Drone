// ============================================================================
//  ELRS_Receiver.h  —  ExpressLRS / CRSF 수신기 드라이버  (v6)
//
//  ★ v6 변경사항
//    - CH6·CH7: 3단 스위치 지원 (LOW=1000 / MID=1500 / HIGH=2000 µs)
//    - GPS 텔레메트리 송신 기능 추가 (CRSF 프레임 0x02)
//    - CH_FAILSAFE_OPT 제거 (CH7 3단 스위치로 통합)
//
//  ★ CRSF 프로토콜 요약
//  ─────────────────────────────────────────────────────────────────────────
//  통신 속도 : 420,000 bps (고정)
//  패킷 구조 : [SYNC 0xC8 | 길이 | 타입 | 페이로드... | CRC8]
//  RC 채널   : 타입 0x16, 16채널 × 11비트 = 22바이트
//  링크 통계 : 타입 0x14, RSSI·LQ·SNR 포함
//
//  ★ 채널 값 범위
//  CRSF raw: 172(최소) ~ 1811(최대), 중앙 992
//  µs 변환:  1000(최소) ~ 2000(최대), 중앙 1500
//
//  ★ 3단 스위치 (CH6, CH7)
//  조종기에서 3단 스위치를 해당 채널에 할당하면:
//    아래 위치 → 약 1000 µs → SW3_LOW
//    중간 위치 → 약 1500 µs → SW3_MID
//    위  위치  → 약 2000 µs → SW3_HIGH
// ============================================================================
#pragma once
#ifndef ELRS_RECEIVER_H
#define ELRS_RECEIVER_H

#include <Arduino.h>
#include "DroneConfig.h"   // CH_xxx, CRSF_xxx, SW_xxx 상수 포함

// CRSF 프로토콜 상수
#define CRSF_SYNC_BYTE      0xC8   // 패킷 시작 동기 바이트
#define CRSF_MAX_PKT_LEN    64     // 최대 패킷 바이트 수
#define CRSF_TYPE_RC        0x16   // RC 채널 데이터 패킷 타입
#define CRSF_TYPE_LINK      0x14   // 링크 통계 패킷 타입

// ── ★ v8 추가: ELRS 장치 핸드셰이크 프레임 타입 ──
#define CRSF_TYPE_DEVICE_PING   0x28   // TX가 FC에 보내는 장치 탐색 요청
#define CRSF_TYPE_DEVICE_INFO   0x29   // FC가 TX에 보내는 장치 정보 응답
#define CRSF_TYPE_HEARTBEAT     0x0B   // 연결 유지 패킷 (수신만, 무시)

// ── CRSF 장치 주소 ──
#define CRSF_ADDR_BROADCAST     0x00   // 브로드캐스트 (DEVICE_PING 등)
#define CRSF_ADDR_FC            0xC8   // 비행 컨트롤러 (= SYNC_BYTE와 동일)
#define CRSF_ADDR_RX_MODULE     0xEA   // 수신기 모듈
#define CRSF_ADDR_TX_MODULE     0xEE   // 조종기 내장 TX 모듈

// ── 디버그 모드 (문제 진단 시에만 주석 해제, 비행 시 반드시 주석 처리)
//#define ELRS_DEBUG

// ============================================================================
//  링크 품질 통계 구조체
// ============================================================================
struct ELRS_LinkStats {
    uint8_t rssi1       = 0;   // 안테나 1 수신 신호 강도 (0~255, 낮을수록 강함)
    uint8_t rssi2       = 0;   // 안테나 2 수신 신호 강도
    uint8_t linkQuality = 0;   // 링크 품질 (0~100%)
    int8_t  snr         = 0;   // 신호 대 잡음비 [dB]
    uint8_t txPower     = 0;   // 송신기 출력 전력 인덱스
};

// ============================================================================
//  ELRS_Receiver 클래스
// ============================================================================
class ELRS_Receiver {
public:
    ELRS_Receiver();

    // ── 초기화 ──
    // serial : HardwareSerial 포트 (Serial1, Serial2 등)
    // rxPin  : ELRS 수신기 TX → ESP32 이 핀
    // txPin  : ELRS 수신기 RX ← ESP32 이 핀 (텔레메트리용)
    // baud   : 통신 속도 (CRSF 고정 420000)
    void begin(HardwareSerial &serial,
               int rxPin, int txPin,
               uint32_t baud = 420000UL);

    // ── 수신 처리 ── 매 루프마다 호출 (비블로킹)
    // 반환값: true = 새 RC 프레임 수신됨
    bool update();

    // ──────────────────────────────────────────────────────────────────────
    //  채널 값 읽기
    // ──────────────────────────────────────────────────────────────────────

    // 원시 CRSF 값 반환 (172~1811), ch: 1~16
    uint16_t getChannelRaw(uint8_t ch) const;

    // µs 변환값 반환 (1000~2000)
    float getChannelUs(uint8_t ch) const;

    // 정규화 값 반환 (-1.0 ~ +1.0)
    float getChannelNorm(uint8_t ch) const;

    // ── 주요 채널 편의 함수 (µs) ──
    float getRollUs()     const { return getChannelUs(CH_ROLL);     }
    float getPitchUs()    const { return getChannelUs(CH_PITCH);    }
    float getThrottleUs() const { return getChannelUs(CH_THROTTLE); }
    float getYawUs()      const { return getChannelUs(CH_YAW);      }

    // ──────────────────────────────────────────────────────────────────────
    //  2단 스위치 (HIGH/LOW 판별)
    //  반환: true = HIGH 상태 (채널값 > 1600 µs)
    // ──────────────────────────────────────────────────────────────────────
    bool isArmed()       const;   // CH5: 시동 ON/OFF
    bool isIMUCalibReq() const;   // CH8: IMU 캘리브레이션 트리거
    bool isRTHActive()   const;   // CH9: RTH 수동 명령

    // ──────────────────────────────────────────────────────────────────────
    //  3단 스위치 (SW3_LOW / SW3_MID / SW3_HIGH)
    //  DroneConfig.h의 SW_LOW_MAX, SW_MID_MAX, SW_HIGH_MIN으로 판별
    //
    //  채널 µs 값 → 스위치 상태:
    //    1000~1200 µs → SW3_LOW  (아래 위치)
    //    1300~1700 µs → SW3_MID  (중간 위치)
    //    1800~2000 µs → SW3_HIGH (위  위치)
    // ──────────────────────────────────────────────────────────────────────

    // 임의 채널을 3단 상태로 변환
    SwPos3 getSwitch3(uint8_t ch) const;

    // CH6: 비행 모드 3단 스위치
    //   SW3_LOW  → FLIGHT_MODE_ANGLE   (자동 수평, 초보자 권장)
    //   SW3_MID  → FLIGHT_MODE_ACRO    (각속도 제어, 레이싱)
    //   SW3_HIGH → FLIGHT_MODE_ALTHOLD (초음파 고도 유지)
    SwPos3 getFlightModeSwitch() const;

    // CH7: 보조 기능 3단 스위치
    //   SW3_LOW  → AUX_NORMAL     (일반 비행)
    //   SW3_MID  → AUX_SONAR_HOLD (초음파 고도 유지 강제)
    //   SW3_HIGH → AUX_RTH        (Return To Home)
    SwPos3 getAuxFuncSwitch() const;

    // ──────────────────────────────────────────────────────────────────────
    //  페일세이프 (신호 끊김 감지)
    // ──────────────────────────────────────────────────────────────────────
    // FAILSAFE_TIMEOUT_MS 동안 신호 없으면 true 반환
    bool     isFailsafe() const;
    void     setFailsafeTimeout(uint32_t ms) { fsTimeoutMs_ = ms; }
    uint32_t timeSinceLastPacketMs() const;

    // ──────────────────────────────────────────────────────────────────────
    //  링크 품질 모니터링
    // ──────────────────────────────────────────────────────────────────────
    const ELRS_LinkStats& getLinkStats() const { return link_; }
    uint8_t getLinkQuality() const { return link_.linkQuality; }
    uint8_t getRSSI()        const { return link_.rssi1;       }

    // ──────────────────────────────────────────────────────────────────────
    //  텔레메트리 송신 (ESP32 → 수신기 → 조종기 화면 표시)
    // ──────────────────────────────────────────────────────────────────────

    // 자세 텔레메트리 (CRSF 타입 0x1E)
    void sendAttitudeTelemetry(float rollDeg, float pitchDeg, float yawDeg);

    // 배터리 텔레메트리 (CRSF 타입 0x08)
    void sendBatteryTelemetry(float voltV, float currA, uint32_t mAh);

    // 비행 모드 텔레메트리 (CRSF 타입 0x21)
    // ELRS가 FC 텔레메트리 스트림을 인식하는 데 필요한 핵심 프레임
    // mode: "ANGLE" / "ACRO" / "ALTHOLD" / "FAILSAFE"
    void sendFlightModeTelemetry(const char *mode);

    // GPS 텔레메트리 (CRSF 타입 0x02)
    void sendGPSTelemetry(const uint8_t *payload, uint8_t payLen);

private:
    HardwareSerial *ser_;                         // UART 포트 포인터
    uint16_t        ch_[ELRS_NUM_CHANNELS + 1];   // 채널 배열 [1~16]

    // CRSF 패킷 파서 상태
    uint8_t  rxBuf_[CRSF_MAX_PKT_LEN];  // 수신 버퍼
    uint8_t  rxIdx_;                     // 현재 버퍼 위치
    bool     inPacket_;                  // 패킷 수신 중 여부

    uint32_t lastPktMs_;    // 마지막 유효 패킷 수신 시각 [millis]
    uint32_t fsTimeoutMs_;  // 페일세이프 타임아웃 [ms]
    bool     newFrame_;     // 이번 update()에서 새 RC 프레임 수신 여부

    ELRS_LinkStats link_;   // 링크 통계

    // 내부 파싱 함수
    void parsePacket(uint8_t payLen, uint8_t type, const uint8_t *pay);
    void parseRC   (const uint8_t *p);   // 22바이트 → 16채널 비트 언패킹
    void parseLink (const uint8_t *p);   // 링크 통계 파싱

    // ★ v8 추가: DEVICE_PING 응답 — 미처리 시 Model Match OFF 불통 원인
    void sendDeviceInfo();   // CRSF_TYPE_DEVICE_INFO(0x29) 응답 전송 (→ 0xEE)

    // 2단 스위치 내부 판별 (AUX_HIGH_THRESH 기준)
    bool getAuxHigh(uint8_t ch) const;

    // CRC-8/DVB-S2 계산 (다항식 0xD5)
    static uint8_t crc8dvbs2(const uint8_t *data, uint8_t len);

    // 텔레메트리 공통 송신 헬퍼
    void sendCRSF        (uint8_t type, const uint8_t *payload, uint8_t payLen);
    void sendCRSFWithAddr(uint8_t destAddr, uint8_t type,
                          const uint8_t *payload, uint8_t payLen);
};

#endif // ELRS_RECEIVER_H
