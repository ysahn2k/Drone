// ============================================================================
//  AHRS_Mahony.cpp  —  Mahony 상보 필터 구현부  (v5)
//
//  이 파일에는 헤더(.h)에 선언된 함수들의 실제 동작 코드가 있습니다.
//  알고리즘 수식은 Mahony의 원본 논문을 기반으로 합니다.
// ============================================================================
#include "AHRS_Mahony.h"

// ============================================================================
//  생성자 — 멤버 변수 초기값 설정
//  콜론(:) 뒤는 멤버 초기화 리스트입니다 (대입보다 효율적)
// ============================================================================
MahonyAHRS::MahonyAHRS()
    : twoKp_(2.0f * MAHONY_KP),  // 게인을 2배로 저장 (계산 최적화)
      twoKi_(2.0f * MAHONY_KI),
      q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),  // 단위 쿼터니언 (회전 없음)
      iFBx_(0.0f), iFBy_(0.0f), iFBz_(0.0f),         // 적분 피드백 초기화
      invSampleFreq_(0.001f)  // 기본값: 1/1000 = 1ms (1 kHz)
{}

// ============================================================================
//  초기화
// ============================================================================
void MahonyAHRS::begin(float sampleRateHz, float kp, float ki)
{
    // 샘플링 주기의 역수를 저장 (매번 나눗셈하지 않고 곱셈으로 최적화)
    invSampleFreq_ = (sampleRateHz > 0.0f) ? (1.0f / sampleRateHz) : 0.001f;
    twoKp_ = 2.0f * kp;
    twoKi_ = 2.0f * ki;
    reset();
}

void MahonyAHRS::reset()
{
    // 단위 쿼터니언으로 초기화 = "회전 없음" 상태
    q0_ = 1.0f;  q1_ = 0.0f;  q2_ = 0.0f;  q3_ = 0.0f;
    iFBx_ = iFBy_ = iFBz_ = 0.0f;
}

// ============================================================================
//  updateIMU — 자이로 + 가속도계만으로 자세 업데이트
//  ─────────────────────────────────────────────────────────────────────────
//  동작 원리:
//  1) 가속도계로 중력 방향을 추정 → 현재 자세의 오차 계산
//  2) 오차를 자이로 데이터에 피드백 → 드리프트 보정
//  3) 보정된 자이로로 쿼터니언 적분 → 새 자세 계산
//  4) 쿼터니언 정규화 (항상 크기=1 유지)
// ============================================================================
void MahonyAHRS::updateIMU(float gx, float gy, float gz,
                            float ax, float ay, float az)
{
    float norm;
    float halfvx, halfvy, halfvz;   // 현재 쿼터니언에서 추정한 중력 방향
    float halfex, halfey, halfez;   // 추정 중력 vs 실측 중력의 오차
    float qa, qb, qc;               // 쿼터니언 임시 저장

    // ── 가속도계 유효성 검사 ──
    // 모든 값이 0이면 센서 오류 → 가속도계 보정을 건너뜀
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // ── 가속도계 정규화 ──
        // 크기를 1로 만들어 방향 정보만 남깁니다 (단위벡터화)
        norm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= norm;  ay *= norm;  az *= norm;

        // ── 현재 쿼터니언에서 예상 중력 방향 계산 ──
        // 쿼터니언을 회전 행렬 3행(중력 방향)으로 변환
        halfvx =  q1_*q3_ - q0_*q2_;
        halfvy =  q0_*q1_ + q2_*q3_;
        halfvz =  q0_*q0_ - 0.5f + q3_*q3_;

        // ── 오차 계산 (벡터 외적) ──
        // 실측 가속도 방향 × 예상 중력 방향 → 오차 벡터
        halfex = ay*halfvz - az*halfvy;
        halfey = az*halfvx - ax*halfvz;
        halfez = ax*halfvy - ay*halfvx;

        // ── 적분 피드백 (KI) — 자이로 드리프트 누적 보정 ──
        if (twoKi_ > 0.0f) {
            iFBx_ += twoKi_ * halfex * invSampleFreq_;  // 오차 누적
            iFBy_ += twoKi_ * halfey * invSampleFreq_;
            iFBz_ += twoKi_ * halfez * invSampleFreq_;
            gx += iFBx_;  gy += iFBy_;  gz += iFBz_;   // 자이로에 적용
        } else {
            iFBx_ = iFBy_ = iFBz_ = 0.0f;  // KI=0이면 누적값 리셋
        }

        // ── 비례 피드백 (KP) — 즉각적인 오차 보정 ──
        gx += twoKp_ * halfex;
        gy += twoKp_ * halfey;
        gz += twoKp_ * halfez;
    }

    // ── 쿼터니언 적분 ──
    // 각속도로 쿼터니언을 미소 회전시킵니다 (1차 오일러 적분)
    gx *= 0.5f * invSampleFreq_;  // 절반 각속도 × 시간 = 미소 각도
    gy *= 0.5f * invSampleFreq_;
    gz *= 0.5f * invSampleFreq_;
    qa = q0_;  qb = q1_;  qc = q2_;  // 현재값 임시 저장
    q0_ += (-qb*gx - qc*gy - q3_*gz);
    q1_ += ( qa*gx + qc*gz - q3_*gy);
    q2_ += ( qa*gy - qb*gz + q3_*gx);
    q3_ += ( qa*gz + qb*gy - qc*gx);

    // ── 정규화 — 쿼터니언 크기를 항상 1로 유지 ──
    // 적분 오류 누적으로 크기가 변할 수 있으므로 매번 보정
    norm = invSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= norm;  q1_ *= norm;  q2_ *= norm;  q3_ *= norm;
}

// ============================================================================
//  update — 지자계(자력계) 포함 업데이트
//  updateIMU와 동일하지만 지자계로 Yaw 드리프트까지 보정합니다.
//  GY-87의 HMC5883L 데이터가 있을 때 이 함수를 사용합니다.
// ============================================================================
void MahonyAHRS::update(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz)
{
    float norm;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    bool magOk = !((mx==0.0f) && (my==0.0f) && (mz==0.0f));
    bool accOk = !((ax==0.0f) && (ay==0.0f) && (az==0.0f));

    if (accOk) {
        norm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= norm;  ay *= norm;  az *= norm;

        if (magOk) {
            // 지자계 정규화
            norm = invSqrt(mx*mx + my*my + mz*mz);
            mx *= norm;  my *= norm;  mz *= norm;

            // 지자계 데이터를 현재 자세로 월드 좌표계에 투영
            hx = 2.0f*(mx*(0.5f-q2_*q2_-q3_*q3_)+my*(q1_*q2_-q0_*q3_)+mz*(q1_*q3_+q0_*q2_));
            hy = 2.0f*(mx*(q1_*q2_+q0_*q3_)+my*(0.5f-q1_*q1_-q3_*q3_)+mz*(q2_*q3_-q0_*q1_));
            bx = sqrtf(hx*hx + hy*hy);  // 수평 성분 크기
            bz = 2.0f*(mx*(q1_*q3_-q0_*q2_)+my*(q2_*q3_+q0_*q1_)+mz*(0.5f-q1_*q1_-q2_*q2_));

            // 예상 중력 + 예상 자기장 방향
            halfvx =  q1_*q3_ - q0_*q2_;
            halfvy =  q0_*q1_ + q2_*q3_;
            halfvz =  q0_*q0_ - 0.5f + q3_*q3_;
            halfwx =  bx*(0.5f-q2_*q2_-q3_*q3_)+bz*(q1_*q3_-q0_*q2_);
            halfwy =  bx*(q1_*q2_-q0_*q3_)      +bz*(q0_*q1_+q2_*q3_);
            halfwz =  bx*(q0_*q2_+q1_*q3_)      +bz*(0.5f-q1_*q1_-q2_*q2_);

            // 가속도 오차 + 자기장 오차 합산
            halfex = (ay*halfvz-az*halfvy) + (my*halfwz-mz*halfwy);
            halfey = (az*halfvx-ax*halfvz) + (mz*halfwx-mx*halfwz);
            halfez = (ax*halfvy-ay*halfvx) + (mx*halfwy-my*halfwx);
        } else {
            // 지자계 없으면 가속도계 오차만 사용
            halfvx =  q1_*q3_ - q0_*q2_;
            halfvy =  q0_*q1_ + q2_*q3_;
            halfvz =  q0_*q0_ - 0.5f + q3_*q3_;
            halfex = ay*halfvz - az*halfvy;
            halfey = az*halfvx - ax*halfvz;
            halfez = ax*halfvy - ay*halfvx;
        }

        if (twoKi_ > 0.0f) {
            iFBx_ += twoKi_ * halfex * invSampleFreq_;
            iFBy_ += twoKi_ * halfey * invSampleFreq_;
            iFBz_ += twoKi_ * halfez * invSampleFreq_;
            gx += iFBx_;  gy += iFBy_;  gz += iFBz_;
        } else {
            iFBx_ = iFBy_ = iFBz_ = 0.0f;
        }
        gx += twoKp_ * halfex;
        gy += twoKp_ * halfey;
        gz += twoKp_ * halfez;
    }

    gx *= 0.5f * invSampleFreq_;
    gy *= 0.5f * invSampleFreq_;
    gz *= 0.5f * invSampleFreq_;
    qa = q0_;  qb = q1_;  qc = q2_;
    q0_ += (-qb*gx - qc*gy - q3_*gz);
    q1_ += ( qa*gx + qc*gz - q3_*gy);
    q2_ += ( qa*gy - qb*gz + q3_*gx);
    q3_ += ( qa*gz + qb*gy - qc*gx);

    norm = invSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= norm;  q1_ *= norm;  q2_ *= norm;  q3_ *= norm;
}

// ============================================================================
//  오일러 각도 변환 함수들
//  쿼터니언(q0~q3) → Roll/Pitch/Yaw [도, °]
//  ZYX 회전 순서 (항공기 표준 규약)
// ============================================================================
float MahonyAHRS::getRoll() const {
    // Roll(φ) = atan2(2(q0·q1 + q2·q3), 1 - 2(q1² + q2²))
    return atan2f(2.0f*(q0_*q1_ + q2_*q3_),
                  1.0f - 2.0f*(q1_*q1_ + q2_*q2_)) * RAD_TO_DEG;
}

float MahonyAHRS::getPitch() const {
    // Pitch(θ) = asin(2(q0·q2 - q3·q1))
    // 클램핑: asin 입력이 ±1 범위를 벗어나면 NaN 발생 방지
    float s = 2.0f*(q0_*q2_ - q3_*q1_);
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s) * RAD_TO_DEG;
}

float MahonyAHRS::getYaw() const {
    // Yaw(ψ) = atan2(2(q0·q3 + q1·q2), 1 - 2(q2² + q3²))
    return atan2f(2.0f*(q0_*q3_ + q1_*q2_),
                  1.0f - 2.0f*(q2_*q2_ + q3_*q3_)) * RAD_TO_DEG;
}

void MahonyAHRS::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    roll  = getRoll();
    pitch = getPitch();
    yaw   = getYaw();
}

void MahonyAHRS::getQuaternion(float &w, float &x, float &y, float &z) const {
    w = q0_;  x = q1_;  y = q2_;  z = q3_;
}

// ============================================================================
//  빠른 역제곱근 invSqrt(x) = 1/√x
//  ─────────────────────────────────────────────────────────────────────────
//  1999년 Quake III Arena 게임 소스코드에서 유명해진 알고리즘입니다.
//  비트 조작으로 초기 근사치를 구하고 Newton-Raphson 반복으로 정밀화합니다.
//  일반 sqrtf() + 나눗셈보다 빠릅니다 (ESP32 FPU에서는 비슷하지만 안전함).
// ============================================================================
float MahonyAHRS::invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;   // 0 또는 음수 방어

    // union을 이용해 float 비트를 uint32_t로 재해석
    union { float f; uint32_t i; } conv = { x };

    // 비트 마술: 역제곱근 초기 근사치 계산
    conv.i = 0x5f3759dfu - (conv.i >> 1);

    // Newton-Raphson 한 번 반복 → 정밀도 향상
    conv.f *= 1.5f - (x * 0.5f * conv.f * conv.f);

    return conv.f;
}
