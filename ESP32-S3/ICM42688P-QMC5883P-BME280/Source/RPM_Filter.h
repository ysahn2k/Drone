// ============================================================================
//  RPM_Filter.h  —  RPM 기반 동적 Biquad 노치 필터  (v8 신규)
//
//  ★ 개요
//  ─────────────────────────────────────────────────────────────────────────
//  모터 RPM에서 유발되는 진동 주파수를 실시간으로 추적하여
//  자이로 신호에서 해당 주파수 성분을 Biquad 노치 필터로 제거합니다.
//
//  ★ 필터 구조
//  ─────────────────────────────────────────────────────────────────────────
//  필터 뱅크: DSHOT_NUM_MOTORS(4) × RPM_FILTER_HARMONICS(2) × 3축
//  = 24개 Biquad 필터 (ESP32-S3 FPU로 ~1µs 처리)
//
//  각 Biquad 노치 필터:
//    H(z) = (1 - 2cos(ω₀)z⁻¹ + z⁻²) / (1 - 2r·cos(ω₀)z⁻¹ + r²z⁻²)
//    r = 1 - π×BW/fs  (BW = fc/Q, fs = 샘플링 주파수)
//
//  ★ 사용법
//  ─────────────────────────────────────────────────────────────────────────
//  1. 객체 생성 및 초기화:
//       RPM_Filter rpmFilter;
//       rpmFilter.begin(1000.0f);  // 샘플링 주파수 [Hz]
//
//  2. 매 루프: RPM 업데이트 (DShot로부터 얻은 RPM)
//       rpmFilter.updateRPM(motorIndex, rpm);
//
//  3. 매 루프: 자이로 신호 필터링
//       gyroX = rpmFilter.filter(RPM_FILTER_AXIS_X, gyroX);
//       gyroY = rpmFilter.filter(RPM_FILTER_AXIS_Y, gyroY);
//       gyroZ = rpmFilter.filter(RPM_FILTER_AXIS_Z, gyroZ);
// ============================================================================
#pragma once
#ifndef RPM_FILTER_H
#define RPM_FILTER_H

#include <Arduino.h>
#include "DroneConfig.h"

// ── 축 인덱스 (filter() 함수 axis 파라미터) ──
#define RPM_FILTER_AXIS_X  0   // Roll  각속도 (gyroX)
#define RPM_FILTER_AXIS_Y  1   // Pitch 각속도 (gyroY)
#define RPM_FILTER_AXIS_Z  2   // Yaw   각속도 (gyroZ)
#define RPM_FILTER_NUM_AXES 3

// ============================================================================
//  BiquadNotch — 단일 Biquad 노치 필터
//  ─────────────────────────────────────────────────────────────────────────
//  Direct Form II Transposed (메모리 효율적, 수치 안정적)
//
//  상태 변수: w1, w2 (내부 지연 요소)
//  계수:      b0, b1, b2 (분자), a1, a2 (분모, a0=1로 정규화)
//
//  ★ 계수 갱신: setFrequency()
//    - 중심주파수 fc [Hz]가 바뀔 때 호출
//    - sinf/cosf 계산 포함 → 매 루프 호출 삼가 (10ms 간격 권장)
//
//  ★ 필터 적용: apply()
//    - 매 루프 호출 (1kHz)
//    - FPU 사용으로 ~0.1µs 이내
// ============================================================================
struct BiquadNotch {
    // 계수 (a0=1로 정규화)
    float b0, b1, b2;  // 분자
    float a1, a2;      // 분모 (부호 주의: y[n] = b0·x - a1·w1 - a2·w2)

    // 상태 변수 (Direct Form II Transposed)
    float w1, w2;

    // 현재 중심주파수 [Hz] (갱신 여부 판단용)
    float centerHz;

    // 활성 여부 (RPM_FILTER_MIN/MAX_HZ 범위 밖이면 비활성)
    bool  active;

    // ── 초기화 (계수 = bypass: 필터 없는 상태) ──
    void init() {
        b0 = 1.0f; b1 = 0.0f; b2 = 0.0f;
        a1 = 0.0f; a2 = 0.0f;
        w1 = 0.0f; w2 = 0.0f;
        centerHz = 0.0f;
        active   = false;
    }

    // ── 중심주파수 설정 (계수 재계산) ──
    // fc: 노치 중심주파수 [Hz]
    // fs: 샘플링 주파수 [Hz] (= 루프 주파수, 1000Hz)
    // Q:  노치 Q팩터
    void setFrequency(float fc, float fs, float Q);

    // ── 필터 적용 ──
    // 입력 x → 출력 y (노치 주파수 성분 제거)
    // active=false이면 bypass (x 그대로 반환)
    inline float apply(float x) {
        if (!active) return x;
        float out = b0 * x + w1;
        w1 = b1 * x - a1 * out + w2;
        w2 = b2 * x - a2 * out;
        return out;
    }
};

// ============================================================================
//  RPM_Filter — RPM 기반 동적 노치 필터 뱅크
//  ─────────────────────────────────────────────────────────────────────────
//  구조:
//    filters_[모터][고조파][축]
//    = [4][2][3] = 24개 BiquadNotch
//
//  모터 RPM이 변경되면 해당 모터의 모든 고조파 × 3축 계수 갱신.
// ============================================================================
class RPM_Filter {
public:
    RPM_Filter();

    // ── 초기화 ──
    // fs: 샘플링 주파수 [Hz] (= 메인 루프 주파수, 보통 1000Hz)
    void begin(float fs);

    // ── RPM 업데이트 ──
    // motorIdx: 0~3 (M1~M4)
    // rpm:      기계적 RPM (DShot_ESP32::getRPM()에서 얻음)
    // RPM이 충분히 변경됐을 때만 계수 재계산 (CPU 절약)
    void updateRPM(uint8_t motorIdx, float rpm);

    // ── 단일 샘플 필터링 ──
    // axis: RPM_FILTER_AXIS_X/Y/Z
    // x:    입력 자이로 값 [deg/s]
    // 반환: 필터링된 자이로 값 [deg/s]
    // 모든 모터의 모든 고조파 필터가 순서대로 적용됨
    float filter(uint8_t axis, float x);

    // ── 디버깅: 필터 상태 시리얼 출력 ──
    // period_ms: 출력 주기 [ms] (예: 2000 = 2초마다)
    void debugPrint(uint32_t period_ms = 2000);

private:
    // 필터 뱅크: [모터][고조파][축]
    BiquadNotch filters_[DSHOT_NUM_MOTORS][RPM_FILTER_HARMONICS][RPM_FILTER_NUM_AXES];

    // 각 모터의 현재 RPM (계수 갱신 여부 판단)
    float   motorRPM_  [DSHOT_NUM_MOTORS];

    // 샘플링 주파수
    float   fs_;

    // 계수 갱신 임계값: RPM 변화가 이 비율 이상일 때만 재계산
    // 예) 0.05 = 5% 변화 시 갱신
    static constexpr float kRPMUpdateThresh = 0.03f;  // 3%

    // 디버그 타이머
    uint32_t debugTimer_;

    // 내부: 특정 모터의 모든 필터 계수 갱신
    void updateMotorFilters(uint8_t motorIdx, float rpm);
};

#endif // RPM_FILTER_H
