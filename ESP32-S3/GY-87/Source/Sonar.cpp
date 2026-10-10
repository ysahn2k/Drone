// ============================================================================
//  Sonar.cpp  —  HC-SR04 초음파 센서 + 고도 유지 PID 구현  (v6)
// ============================================================================
#include "Sonar.h"

// ============================================================================
//  생성자
// ============================================================================
Sonar::Sonar()
    : trigPin_(PIN_SONAR_TRIG), echoPin_(PIN_SONAR_ECHO),
      ready_(false), valid_(false),
      rawCm_(0.0f), filteredCm_(ALT_HOLD_TARGET_CM),
      lastMeasureMs_(0),
      kp_(ALT_KP), ki_(ALT_KI), kd_(ALT_KD)
{}

// ============================================================================
//  begin — GPIO 핀 초기화
// ============================================================================
void Sonar::begin(int trigPin, int echoPin)
{
    trigPin_ = trigPin;
    echoPin_ = echoPin;

    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(trigPin_, LOW);
    delay(50);  // 센서 안정화 대기

    // 초기 측정으로 필터 초기화
    float initDist = measureOnce();
    if (initDist > 0) {
        filteredCm_       = initDist;
        altPid_.setpoint  = initDist;  // 현재 위치를 초기 목표로
    }

    ready_ = true;
    Serial.printf("[Sonar] 초기화 완료 (TRIG=%d ECHO=%d 초기거리=%.1fcm)\n",
                  trigPin_, echoPin_, filteredCm_);
}

// ============================================================================
//  update — 비블로킹 주기적 측정 (매 루프 호출)
//  ─────────────────────────────────────────────────────────────────────────
//  SONAR_UPDATE_MS(50ms)마다 한 번 측정합니다.
//  연속 측정 시 초음파 간섭이 발생하므로 충분한 간격이 필요합니다.
//  ★ 실내 또는 단단한 바닥: 간격을 줄여도 됨 (30ms)
//  ★ 실외 잡음 환경: 간격 늘리기 권장 (70ms)
// ============================================================================
void Sonar::update()
{
    if (!ready_) return;

    uint32_t now = millis();
    if (now - lastMeasureMs_ < SONAR_UPDATE_MS) return;
    lastMeasureMs_ = now;

    float dist = measureOnce();

    if (dist >= SONAR_MIN_CM && dist <= SONAR_MAX_CM) {
        rawCm_      = dist;
        // 저역통과 필터 적용: 새 값의 가중치를 SONAR_FILTER_ALPHA로 제한
        // alpha=0.2이면 "새값 20% + 이전값 80%" = 부드러운 출력
        filteredCm_ = lowPassFilter(dist, filteredCm_, SONAR_FILTER_ALPHA);
        valid_      = true;
    } else {
        // 측정 실패 또는 범위 초과
        valid_ = false;
        // 필터값은 유지 (급격한 변화 방지)
    }
}

// ============================================================================
//  measureOnce — HC-SR04 단일 측정 (블로킹, 최대 25ms)
//  ─────────────────────────────────────────────────────────────────────────
//  측정 순서:
//  1) TRIG 핀에 10µs HIGH 펄스 → 초음파 발사
//  2) pulseIn()으로 ECHO 핀 HIGH 지속시간 측정
//  3) 거리[cm] = 시간[µs] / 58.0
//     (음속 340m/s: 1cm 왕복에 약 58µs 소요)
// ============================================================================
float Sonar::measureOnce()
{
    // TRIG 핀을 일시적으로 LOW로 확실히 초기화
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);

    // 10µs HIGH 펄스 전송 → 초음파 발사
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(SONAR_TRIGGER_US);  // 10µs
    digitalWrite(trigPin_, LOW);

    // ECHO 핀이 HIGH가 되는 시간 측정
    // timeout=SONAR_TIMEOUT_US(25000µs): 4m 이상 거리에서 타임아웃
    long duration = pulseIn(echoPin_, HIGH, SONAR_TIMEOUT_US);

    if (duration <= 0) return -1.0f;  // 타임아웃 = 측정 실패

    // 거리 계산: 왕복 시간 / 2 / (음속 1cm당 시간)
    // 음속 340m/s = 0.034 cm/µs → 1cm 편도 = 1/0.034 ≈ 29.4µs
    // 왕복 1cm = 58.8µs → 거리[cm] = 시간[µs] / 58.0
    return (float)duration / 58.0f;
}

// ============================================================================
//  lowPassFilter — 1차 저역통과 필터 (지수 이동 평균)
//  ─────────────────────────────────────────────────────────────────────────
//  공식: y_n = alpha × x_n + (1 - alpha) × y_{n-1}
//    alpha: 0에 가까울수록 부드럽고 반응이 느림
//    alpha: 1에 가까울수록 노이즈가 많고 반응이 빠름
//  SONAR_FILTER_ALPHA = 0.2 → "새값 20% + 이전값 80%"
// ============================================================================
float Sonar::lowPassFilter(float newVal, float prevVal, float alpha)
{
    return alpha * newVal + (1.0f - alpha) * prevVal;
}

// ============================================================================
//  computeAltHold — 고도 유지 PID 계산
//  ─────────────────────────────────────────────────────────────────────────
//  입력: 현재 시간 간격 dt [초]
//  출력: 스로틀 보정량 [µs] (양수=상승, 음수=하강)
//
//  PID 계산:
//    오차 = 목표고도 - 현재고도
//    P항  = Kp × 오차
//    I항 += Ki × 오차 × dt  (드리프트 제거)
//    D항  = -Kd × (현재오차 - 이전오차) / dt  (흔들림 감쇠)
//    출력  = P + I + D
//
//  ★ 튜닝 팁:
//    Kp 올리기: 반응 빠름 → 너무 크면 위아래 진동
//    Ki 올리기: 고도 드리프트 제거 → 너무 크면 느린 진동
//    Kd 올리기: 진동 감쇠 → 너무 크면 노이즈 증폭
// ============================================================================
float Sonar::computeAltHold(float dt)
{
    if (!valid_ || dt <= 0.0f) return 0.0f;

    float error = altPid_.setpoint - filteredCm_;  // 오차: 목표-현재 [cm]

    // P항
    float pTerm = kp_ * error;

    // I항 (적분, 와인드업 방지)
    altPid_.iTerm += ki_ * error * dt;
    if (altPid_.iTerm >  ALT_ITERM_LIMIT) altPid_.iTerm =  ALT_ITERM_LIMIT;
    if (altPid_.iTerm < -ALT_ITERM_LIMIT) altPid_.iTerm = -ALT_ITERM_LIMIT;

    // D항 (오차 변화율, D-on-Error 방식)
    float dErr  = (error - altPid_.prevError) / dt;
    float dTerm = kd_ * dErr;
    altPid_.prevError = error;

    // 출력 계산 및 제한
    float output = pTerm + altPid_.iTerm + dTerm;
    if (output >  ALT_OUTPUT_LIMIT) output =  ALT_OUTPUT_LIMIT;
    if (output < -ALT_OUTPUT_LIMIT) output = -ALT_OUTPUT_LIMIT;

    return output;  // µs 단위 스로틀 보정량
}
