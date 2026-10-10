// ============================================================================
//  Sonar.h  —  HC-SR04 초음파 센서 + 고도 유지 PID  (v6)
//
//  ★ HC-SR04란?
//  ─────────────────────────────────────────────────────────────────────────
//  40kHz 초음파를 발사하고 반사파가 돌아오는 시간으로 거리를 측정합니다.
//
//  동작 원리:
//    1) TRIG 핀에 10µs HIGH 펄스 인가 → 8개의 40kHz 초음파 발사
//    2) ECHO 핀이 HIGH가 되는 시간 측정 (초음파 왕복 시간)
//    3) 거리[cm] = 왕복시간[µs] / 58  (음속 340m/s 기준)
//
//  사양:
//    측정 범위: 2 ~ 400 cm (실용 범위 2~300 cm)
//    정밀도:    ±3 mm
//    측정 주기: 50 ms 이상 권장 (초음파 간섭 방지)
//    빔 각도:   15도 이내 (지면 반사 고려)
//
//  ★ 연결 방법
//  ─────────────────────────────────────────────────────────────────────────
//  HC-SR04 VCC  → 5V (반드시 5V! 3.3V에서는 측정 오류 발생)
//  HC-SR04 TRIG → ESP32 GPIO10 (PIN_SONAR_TRIG)
//  HC-SR04 ECHO → 분압(1kΩ+2kΩ) 후 → ESP32 GPIO11 (PIN_SONAR_ECHO)
//                 ★ ECHO 핀은 5V 신호이므로 분압 필수!
//                 ★ 분압: ECHO → 1kΩ → GPIO11, GPIO11 → 2kΩ → GND
//  HC-SR04 GND  → GND
//
//  ★ 고도 유지 PID
//  ─────────────────────────────────────────────────────────────────────────
//  목표 고도 - 실제 고도 = 오차 → PID 계산 → 스로틀 보정량
//  스로틀 = 기본 스로틀(1500µs) + PID 보정량
//
//  ★ CH6/CH7 3단 스위치 활용
//  ─────────────────────────────────────────────────────────────────────────
//  CH6 MID: Acro 모드 + 고도 유지 (자세는 조종사가 직접 제어)
//  CH6 HIGH: Angle 모드 + 고도 유지
//  CH7 MID: 초음파 고도 유지 활성화
// ============================================================================
#pragma once
#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "DroneConfig.h"

// ============================================================================
//  고도 유지 PID 상태 구조체
// ============================================================================
struct AltHold_PID {
    float setpoint  = ALT_HOLD_TARGET_CM;  // 목표 고도 [cm]
    float iTerm     = 0.0f;                // 적분항 누적
    float prevError = 0.0f;                // 이전 오차 (D항 계산용)
};

// ============================================================================
//  Sonar 클래스
// ============================================================================
class Sonar {
public:
    Sonar();

    // 초기화 (setup()에서 호출)
    // trigPin: TRIG GPIO, echoPin: ECHO GPIO
    void begin(int trigPin = PIN_SONAR_TRIG, int echoPin = PIN_SONAR_ECHO);

    // 거리 측정 시작 (비블로킹 — 매 루프 호출)
    // SONAR_UPDATE_MS(50ms) 간격으로 자동 측정
    void update();

    // ── 거리 데이터 접근 ──
    float getDistanceCm() const { return filteredCm_; }    // 필터링된 거리 [cm]
    float getRawCm()      const { return rawCm_;       }    // 원시 거리 [cm]
    bool  isValid()       const { return valid_;       }    // 유효 범위 여부
    bool  isReady()       const { return ready_;       }    // 초기화 완료 여부

    // ── 고도 유지 PID ──

    // 목표 고도 설정 [cm]
    // setFromCurrent: true이면 현재 고도를 목표로 설정 (고도 유지 시작 시 호출)
    void setTargetAlt(float cm)      { altPid_.setpoint = cm; }
    void holdCurrentAlt()            { altPid_.setpoint = filteredCm_; altPid_.iTerm = 0; }

    float getTargetAlt() const       { return altPid_.setpoint; }

    // 고도 유지 PID 계산 → 스로틀 보정량 반환
    // 반환값: 양수=상승 보정, 음수=하강 보정 (µs 단위)
    // 예: +100이면 기본 1500µs + 100 = 1600µs 스로틀
    float computeAltHold(float dt);

    // 고도 유지 PID 초기화 (고도 유지 모드 진입 시 호출)
    void resetAltHold()  { altPid_.iTerm = 0; altPid_.prevError = 0; }

    // PID 게인 런타임 변경 (기본값은 DroneConfig.h에서 로드)
    void setGains(float kp, float ki, float kd)
        { kp_=kp; ki_=ki; kd_=kd; }

private:
    int     trigPin_, echoPin_;
    bool    ready_;
    bool    valid_;

    float   rawCm_;          // 필터 전 거리 [cm]
    float   filteredCm_;     // 저역통과 필터 후 거리 [cm]

    uint32_t lastMeasureMs_;  // 마지막 측정 시작 시각

    // 고도 유지 PID
    AltHold_PID altPid_;
    float kp_, ki_, kd_;

    // 단일 블로킹 측정 (pulseIn 사용)
    float measureOnce();

    // 저역통과 필터 적용
    // alpha: 0에 가까울수록 부드럽지만 반응 느림
    float lowPassFilter(float newVal, float prevVal, float alpha);
};

#endif // SONAR_H
