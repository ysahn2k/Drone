// ============================================================================
//  PID_Cascade.h  —  Cascade(이중) PID 컨트롤러  (v5)
//
//  ★ 일반 PID vs Cascade PID
//  ─────────────────────────────────────────────────────────────────────────
//  일반 PID:
//    스틱(각도 목표) → [단일 PID] → 모터 출력
//
//  Cascade PID:
//    스틱(각도 목표) → [외부 PID: 각도→각속도] → [내부 PID: 각속도→모터] → 모터
//
//  왜 Cascade를 사용하나?
//  - 각도 제어만으로는 빠른 외란(바람 등)에 느리게 반응
//  - 내부 루프(각속도)가 빠른 외란을 먼저 감쇠
//  - 외부 루프(각도)가 큰 그림의 자세를 유지
//  - 결과: 더 안정적이고 반응이 빠른 비행
//
//  ★ 각 루프의 입출력
//  ─────────────────────────────────────────────────────────────────────────
//  외부 루프 (Outer / Angle PID):
//    입력(setpoint): 조종 스틱에서 목표 각도 [도]
//    피드백(feedback): IMU에서 실제 각도 [도] (PRIMARY_IMU)
//    출력:           목표 각속도 [도/초] → 내부 루프의 입력
//
//  내부 루프 (Inner / Rate PID):
//    입력(setpoint): 외부 루프 출력 (Angle모드) 또는 스틱 직접 입력 (Acro모드)
//    피드백(feedback): 자이로에서 실제 각속도 [도/초] (SECONDARY_IMU)
//    출력:           모터 DShot 값 변화량
//
//  ★ D항 on Measurement (미분 측정치 기반)
//  ─────────────────────────────────────────────────────────────────────────
//  일반적인 D항: Kd × d(오차)/dt = Kd × (오차-이전오차)/dt
//  문제점: 설정값(setpoint)이 갑자기 변하면 D항이 폭발적으로 커짐 (D-kick)
//
//  D-on-Measurement: Kd × d(피드백)/dt 를 사용 (부호 반전)
//  장점: 설정값 변화에 영향 없음 → 스틱을 빠르게 움직여도 모터가 충격받지 않음
// ============================================================================
#pragma once
#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#include <Arduino.h>
#include "DroneConfig.h"

// ============================================================================
//  PID 게인 구조체
// ============================================================================
struct PIDGains {
    float kp = 0.0f;    // 비례 게인: 현재 오차에 비례한 반응
    float ki = 0.0f;    // 적분 게인: 오차 누적 제거 (드리프트 보정)
    float kd = 0.0f;    // 미분 게인: 변화율 억제 (진동 감쇠)
    float outLimit = 500.0f;  // 출력 최대값 (±)
    float iLimit   = 200.0f;  // I항 누적 최대값 (와인드업 방지)
};

// ============================================================================
//  PID 상태 구조체 (내부 계산 변수들)
// ============================================================================
struct PIDState {
    float setpoint = 0.0f;   // 목표값 (외부에서 설정)
    float feedback = 0.0f;   // 현재값 (센서에서 측정)
    float prevFB   = 0.0f;   // 이전 피드백값 (D항 계산용)
    float dFilt    = 0.0f;   // D항 LPF 상태 (노이즈 제거)
    float iTerm    = 0.0f;   // I항 누적값 (리셋 전까지 유지)
    float output   = 0.0f;   // 최종 PID 출력값
    float spLpf    = 0.0f;   // ★ itermRelax용: setpoint 저역통과 상태
    bool  spLpfInit = false; // ★ 최초 1회 초기화 플래그 (시작 시 급격한 변화 방지)
};

// ============================================================================
//  PID_Cascade 클래스
// ============================================================================
class PID_Cascade {
public:
    PID_Cascade();

    // ── 게인 설정 ──
    // axis: AXIS_YAW(0), AXIS_PITCH(1), AXIS_ROLL(2)
    void setOuterGains(uint8_t axis, float kp, float ki, float kd);
    void setInnerGains(uint8_t axis, float kp, float ki, float kd);
    void setOuterLimit(uint8_t axis, float lim);
    void setInnerLimit(uint8_t axis, float lim);

    // ── setpoint/feedback 주입 ──
    // 매 루프에서 최신 센서값과 목표값을 넣어줍니다
    void setOuterSetpoint(uint8_t a, float v) { outerSt_[a].setpoint = v; }
    void setOuterFeedback(uint8_t a, float v) { outerSt_[a].feedback = v; }
    void setInnerSetpoint(uint8_t a, float v) { innerSt_[a].setpoint = v; }
    void setInnerFeedback(uint8_t a, float v) { innerSt_[a].feedback = v; }

    // ── 계산 함수 ──

    // Angle 모드: 외부(각도)→내부(각속도) 전체 계산
    // dt: 루프 시간 간격 [초]
    float compute(uint8_t axis, float dt);

    // Acro 모드: 내부 루프만 계산 (각속도 직접 제어)
    float computeInnerOnly(uint8_t axis, float dt);

    // 외부 루프만 계산 (각속도 목표값 반환)
    float computeOuterOnly(uint8_t axis, float dt);

    // ── 리셋 ──
    void reset(uint8_t axis);  // 특정 축 I항과 상태 초기화
    void resetAll();            // 전체 축 리셋 (시동 시 반드시 호출)
    void resetYawITerm();       // ★ Yaw I항만 초기화 (암밍 시 와인드업 제거)
    void decayYawITerm(float factor); // ★ Yaw I항 감쇠 (스틱 중앙 시 호출)

    // ── I항 동결 ──
    // 스로틀 최저 위치에서 I항이 계속 쌓이면 시동 시 급발진 위험
    // throttle < THROTTLE_ARM_MIN 일 때 true로 설정
    void setITermFreeze(bool f) { freeze_ = f; }

    // ── 출력값 읽기 (디버그/텔레메트리용) ──
    float getOuterOutput(uint8_t a) const { return outerSt_[a].output; }
    float getInnerOutput(uint8_t a) const { return innerSt_[a].output; }

    // ★ 진단용: Pitch 바이어스 원인 추적
    float getOuterITerm(uint8_t a)  const { return (a<3) ? outerSt_[a].iTerm : 0.f; }
    float getInnerITerm(uint8_t a)  const { return (a<3) ? innerSt_[a].iTerm : 0.f; }
    float getOuterSetpointRead(uint8_t a) const { return (a<3) ? outerSt_[a].setpoint : 0.f; }
    float getOuterFeedbackRead(uint8_t a) const { return (a<3) ? outerSt_[a].feedback : 0.f; }
    float getInnerSetpointRead(uint8_t a) const { return (a<3) ? innerSt_[a].setpoint : 0.f; }
    float getInnerFeedbackRead(uint8_t a) const { return (a<3) ? innerSt_[a].feedback : 0.f; }

private:
    PIDGains outerG_[3];   // 외부 루프 게인 [YAW, PITCH, ROLL]
    PIDGains innerG_[3];   // 내부 루프 게인 [YAW, PITCH, ROLL]
    PIDState outerSt_[3];  // 외부 루프 상태
    PIDState innerSt_[3];  // 내부 루프 상태
    bool     freeze_;      // I항 동결 플래그

    // PID 실제 계산 (내부용)
    // g: 게인 설정, s: 현재 상태, dt: 시간 간격
    float runPID(PIDGains &g, PIDState &s, float dt);
};

#endif // PID_CASCADE_H
