// ============================================================================
//  RPM_Filter.cpp  —  RPM 기반 동적 Biquad 노치 필터 구현  (v8 신규)
// ============================================================================
#include "RPM_Filter.h"
#include <math.h>

// ============================================================================
//  BiquadNotch::setFrequency — 노치 필터 계수 계산
//  ─────────────────────────────────────────────────────────────────────────
//  Biquad 노치 필터 설계 (Bilinear Transform 기반):
//
//  ω₀ = 2π × fc / fs  (정규화 각주파수)
//  BW = fc / Q         (대역폭 [Hz])
//  α  = sin(ω₀) / (2Q) = sin(ω₀) × BW / (2 × fc)
//
//  연속 전달함수:
//    H(z) = (1 - 2cos(ω₀)z⁻¹ + z⁻²) / ((1+α) - 2cos(ω₀)z⁻¹ + (1-α)z⁻²)
//
//  정규화 (÷ a0 = 1+α):
//    b0 = 1 / (1+α)
//    b1 = -2cos(ω₀) / (1+α)
//    b2 = 1 / (1+α)
//    a1 = -2cos(ω₀) / (1+α)   [같음: 노치의 특성]
//    a2 = (1-α) / (1+α)
//
//  참고: Direct Form II Transposed 적용 시:
//    w = x - a1·w1 - a2·w2
//    y = b0·w + b1·w1 + b2·w2
//  최적화하면:
//    out = b0·x + w1
//    w1  = b1·x - a1·out + w2
//    w2  = b2·x - a2·out
// ============================================================================
void BiquadNotch::setFrequency(float fc, float fs, float Q)
{
    // 범위 외 주파수 → 필터 비활성 (bypass)
    if (fc < RPM_FILTER_MIN_HZ || fc > RPM_FILTER_MAX_HZ) {
        active    = false;
        centerHz  = fc;
        return;
    }

    centerHz = fc;
    active   = true;

    const float omega = 2.0f * (float)M_PI * fc / fs;
    const float cosOm = cosf(omega);
    const float sinOm = sinf(omega);
    const float alpha  = sinOm / (2.0f * Q);
    const float a0inv  = 1.0f / (1.0f + alpha);  // 정규화 인수

    b0 =  a0inv;
    b1 = -2.0f * cosOm * a0inv;
    b2 =  a0inv;
    a1 =  b1;           // 노치 필터 특성: a1 = b1
    a2 = (1.0f - alpha) * a0inv;

    // 상태 변수 초기화는 하지 않음: 급격한 계수 변경 시 transient 최소화
    // (상태 유지가 초기화보다 보통 더 나은 결과)
}

// ============================================================================
//  RPM_Filter 생성자 / begin
// ============================================================================
RPM_Filter::RPM_Filter()
    : fs_(1000.0f), debugTimer_(0)
{
    memset(motorRPM_, 0, sizeof(motorRPM_));
}

void RPM_Filter::begin(float fs)
{
    fs_ = fs;

    // 모든 필터 초기화 (bypass 상태)
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        motorRPM_[m] = 0.0f;
        for (int h = 0; h < RPM_FILTER_HARMONICS; h++) {
            for (int ax = 0; ax < RPM_FILTER_NUM_AXES; ax++) {
                filters_[m][h][ax].init();
            }
        }
    }

    Serial.printf("[RPMFilter] 초기화: %.0f Hz 루프, %d모터 × %d고조파 × %d축 = %d 필터\n",
                  fs_, DSHOT_NUM_MOTORS, RPM_FILTER_HARMONICS, RPM_FILTER_NUM_AXES,
                  DSHOT_NUM_MOTORS * RPM_FILTER_HARMONICS * RPM_FILTER_NUM_AXES);
    Serial.printf("[RPMFilter] 노치 Q=%.1f, 범위=%.0f~%.0f Hz\n",
                  (float)RPM_FILTER_Q, (float)RPM_FILTER_MIN_HZ, (float)RPM_FILTER_MAX_HZ);
}

// ============================================================================
//  updateRPM — 모터 RPM 업데이트 + 필터 계수 재계산
//  ─────────────────────────────────────────────────────────────────────────
//  매 루프 호출되지만, RPM 변화가 kRPMUpdateThresh(3%) 미만이면 계수 유지.
//  이유: sinf/cosf 계산 비용 절약 + 불필요한 계수 변동 방지.
// ============================================================================
void RPM_Filter::updateRPM(uint8_t motorIdx, float rpm)
{
    if (motorIdx >= DSHOT_NUM_MOTORS) return;

    // RPM이 너무 작으면 (모터 정지) 필터 비활성화
    if (rpm < 10.0f) {
        if (motorRPM_[motorIdx] >= 10.0f) {
            // 방금 정지됨 → 필터 비활성화
            motorRPM_[motorIdx] = 0.0f;
            for (int h = 0; h < RPM_FILTER_HARMONICS; h++)
                for (int ax = 0; ax < RPM_FILTER_NUM_AXES; ax++)
                    filters_[motorIdx][h][ax].active = false;
        }
        return;
    }

    // RPM 변화율 확인
    float prevRPM = motorRPM_[motorIdx];
    float changeFrac = fabsf(rpm - prevRPM) / (prevRPM > 0.0f ? prevRPM : 1.0f);

    if (changeFrac < kRPMUpdateThresh) {
        return;  // 변화 미미 → 계수 유지
    }

    motorRPM_[motorIdx] = rpm;
    updateMotorFilters(motorIdx, rpm);
}

// ============================================================================
//  updateMotorFilters — 특정 모터의 모든 고조파 필터 계수 갱신
//  ─────────────────────────────────────────────────────────────────────────
//  고조파 h (0-indexed):
//    h=0: 기본주파수 = RPM / 60 [Hz]
//    h=1: 2고조파   = 2 × RPM / 60 [Hz]
//    h=2: 3고조파   = 3 × RPM / 60 [Hz]
// ============================================================================
void RPM_Filter::updateMotorFilters(uint8_t motorIdx, float rpm)
{
    for (int h = 0; h < RPM_FILTER_HARMONICS; h++) {
        // 고조파 주파수 = (h+1) × 기본주파수
        float fc = (float)(h + 1) * rpm / 60.0f;

        for (int ax = 0; ax < RPM_FILTER_NUM_AXES; ax++) {
            filters_[motorIdx][h][ax].setFrequency(fc, fs_, RPM_FILTER_Q);
        }
    }
}

// ============================================================================
//  filter — 단일 샘플에 모든 필터 적용 (직렬 연결)
//  ─────────────────────────────────────────────────────────────────────────
//  4모터 × 2고조파 = 8개 Biquad 필터를 순서대로 통과.
//  비활성 필터(active=false)는 bypass되므로 추가 비용 없음.
//
//  처리 시간 (ESP32-S3 FPU 기준):
//    활성 필터 1개: ~0.08µs
//    8개 최대: ~0.64µs (PID 루프 부담 매우 적음)
// ============================================================================
float RPM_Filter::filter(uint8_t axis, float x)
{
    if (axis >= RPM_FILTER_NUM_AXES) return x;

    float y = x;
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        for (int h = 0; h < RPM_FILTER_HARMONICS; h++) {
            y = filters_[m][h][axis].apply(y);
        }
    }
    return y;
}

// ============================================================================
//  debugPrint — 시리얼 모니터 디버그 출력
// ============================================================================
void RPM_Filter::debugPrint(uint32_t period_ms)
{
    if (millis() - debugTimer_ < period_ms) return;
    debugTimer_ = millis();

    Serial.println("[RPMFilter] ── 상태 ──────────────────────────────");
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        Serial.printf("  M%d  RPM=%6.0f  ", m+1, motorRPM_[m]);
        for (int h = 0; h < RPM_FILTER_HARMONICS; h++) {
            float fc = filters_[m][h][0].centerHz;
            bool  on = filters_[m][h][0].active;
            Serial.printf("f%d=%.0fHz(%s) ", h+1, fc, on ? "ON" : "OFF");
        }
        Serial.println();
    }
    Serial.println("[RPMFilter] ──────────────────────────────────────");
}
