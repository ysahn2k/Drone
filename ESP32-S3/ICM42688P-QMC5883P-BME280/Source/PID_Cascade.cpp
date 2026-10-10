// ============================================================================
//  PID_Cascade.cpp  —  Cascade PID 컨트롤러 구현  (v5)
// ============================================================================
#include "PID_Cascade.h"

// ============================================================================
//  생성자 — DroneConfig.h의 기본 게인값으로 초기화
// ============================================================================
PID_Cascade::PID_Cascade() : freeze_(false)
{
    // 외부 루프 게인 설정 (DroneConfig.h 참조)
    setOuterGains(AXIS_YAW,   OUTER_KP_YAW,   OUTER_KI_YAW,   OUTER_KD_YAW);
    setOuterGains(AXIS_PITCH, OUTER_KP_PITCH, OUTER_KI_PITCH, OUTER_KD_PITCH);
    setOuterGains(AXIS_ROLL,  OUTER_KP_ROLL,  OUTER_KI_ROLL,  OUTER_KD_ROLL);

    // 내부 루프 게인 설정
    setInnerGains(AXIS_YAW,   INNER_KP_YAW,   INNER_KI_YAW,   INNER_KD_YAW);
    setInnerGains(AXIS_PITCH, INNER_KP_PITCH, INNER_KI_PITCH, INNER_KD_PITCH);
    setInnerGains(AXIS_ROLL,  INNER_KP_ROLL,  INNER_KI_ROLL,  INNER_KD_ROLL);

    // 출력 제한 설정
    for (int a = 0; a < 3; a++) {
        outerG_[a].outLimit = PID_OUTER_LIMIT;  // ±300 deg/s
        outerG_[a].iLimit   = PID_ITERM_LIMIT;  // ±200
        innerG_[a].outLimit = PID_INNER_LIMIT;  // ±500 (DShot 단위)
        innerG_[a].iLimit   = PID_ITERM_LIMIT;
    }
    resetAll();
}

// 게인 설정 헬퍼 함수들
void PID_Cascade::setOuterGains(uint8_t a, float kp, float ki, float kd) {
    if (a > 2) return;
    outerG_[a].kp = kp;  outerG_[a].ki = ki;  outerG_[a].kd = kd;
}
void PID_Cascade::setInnerGains(uint8_t a, float kp, float ki, float kd) {
    if (a > 2) return;
    innerG_[a].kp = kp;  innerG_[a].ki = ki;  innerG_[a].kd = kd;
}
void PID_Cascade::setOuterLimit(uint8_t a, float l) {
    if (a <= 2) outerG_[a].outLimit = l;
}
void PID_Cascade::setInnerLimit(uint8_t a, float l) {
    if (a <= 2) innerG_[a].outLimit = l;
}

// ============================================================================
//  runPID — 단일 PID 루프 계산 (내부 공통 함수)
//  ─────────────────────────────────────────────────────────────────────────
//  PID 계산 순서:
//  1) 오차 = 목표값 - 현재값
//  2) P항 = Kp × 오차
//  3) I항 += Ki × 오차 × dt  (적분, 클램핑 포함)
//  4) D항 = -Kd × (피드백 변화량) / dt  (측정치 미분, D-kick 방지)
//  5) 출력 = P + I + D  (클램핑)
// ============================================================================
float PID_Cascade::runPID(PIDGains &g, PIDState &s, float dt)
{
    if (dt <= 0.0f) return s.output;  // dt=0 방어 (첫 루프 등)

    // ── P항 (비례) ──
    float error = s.setpoint - s.feedback;
    float pTerm = g.kp * error;

    // ── I항 (적분) ──
    // ★ Betaflight 분석 적용: itermRelax (사전 예방형 안티윈드업)
    //   스틱을 빠르게 움직이는 동안(setpoint 고주파 변화)에는 I항 누적을
    //   미리 감쇄시켜, 큰 스틱 입력 이후 갑작스런 I-kick을 방지한다.
    //   다만 I항이 스스로 풀리는 방향(자연 감소)일 때는 relax를 적용하지
    //   않는다 — 자연스러운 와인드업 해소를 방해하지 않기 위함.
    if (!freeze_) {
        float alpha = (2.0f * 3.14159f * ITERM_RELAX_LPF_HZ * dt)
                    / (1.0f + 2.0f * 3.14159f * ITERM_RELAX_LPF_HZ * dt);
        if (!s.spLpfInit) { s.spLpf = s.setpoint; s.spLpfInit = true; }  // 최초 점프 방지
        s.spLpf += alpha * (s.setpoint - s.spLpf);
        float setpointHpf = fabsf(s.setpoint - s.spLpf);   // 스틱 급변 정도(고주파 성분)

        float itermErrorRate = error;
        bool  isDecreasingI = ((s.iTerm > 0.0f) && (error < 0.0f)) ||
                               ((s.iTerm < 0.0f) && (error > 0.0f));
        if (!isDecreasingI) {
            float relaxFactor = 1.0f - setpointHpf / ITERM_RELAX_THRESHOLD_DPS;
            if (relaxFactor < 0.0f) relaxFactor = 0.0f;
            itermErrorRate *= relaxFactor;
        }

        // dt를 곱해 시간에 따른 오차 면적을 적산
        s.iTerm += g.ki * itermErrorRate * dt;
        // 와인드업 방지: 최대값 클램핑
        if (s.iTerm >  g.iLimit) s.iTerm =  g.iLimit;
        if (s.iTerm < -g.iLimit) s.iTerm = -g.iLimit;
    }
    // freeze_=true이면 현재 iTerm 유지 (스로틀 최저 위치에서 누적 방지)

    // ── D항 (미분, D-on-Measurement + LPF) ──
    // 피드백의 변화율을 사용 (목표값 변화에 영향받지 않음)
    // ★ LPF 필수: 자이로 노이즈/진동 성분이 D항에 그대로 들어가면 스파이크 발생
    //   → LPF 없으면 D항이 매 2ms마다 스파이크 → 모터 최고속 발산 현상
    //
    // LPF 차단주파수 100Hz (α = 2π×100×dt / (1 + 2π×100×dt)):
    //   dt=0.001 → α ≈ 0.386
    //   → 100Hz 이하 진동 억제 정보만 D항에 반영
    //   → 고주파 노이즈(센서 스테일/업데이트 패턴) 차단
    {
        float dFB_raw  = (s.feedback - s.prevFB) / dt;
        // D항 LPF 150Hz: 고주파 노이즈 차단, 실제 진동은 통과
        // α = 2π×150×dt / (1 + 2π×150×dt), dt=0.001 → α ≈ 0.485
        float alpha    = (2.0f * 3.14159f * 150.0f * dt)
                       / (1.0f + 2.0f * 3.14159f * 150.0f * dt);
        s.dFilt        = alpha * dFB_raw + (1.0f - alpha) * s.dFilt;
    }
    float dTerm = -g.kd * s.dFilt;
    s.prevFB = s.feedback;

    // ── 출력 계산 및 클램핑 ──
    float out = pTerm + s.iTerm + dTerm;
    if (out >  g.outLimit) out =  g.outLimit;
    if (out < -g.outLimit) out = -g.outLimit;
    s.output = out;
    return out;
}

// ============================================================================
//  compute — Angle 모드: 전체 Cascade PID 계산
//  외부 루프 출력(각속도 목표)을 내부 루프 입력으로 연결합니다.
// ============================================================================
float PID_Cascade::compute(uint8_t axis, float dt)
{
    if (axis > 2) return 0.0f;
    // 1) 외부 루프: 각도 오차 → 목표 각속도
    float rateSetpoint = runPID(outerG_[axis], outerSt_[axis], dt);
    // 2) 내부 루프: 외부 출력을 입력으로 사용
    innerSt_[axis].setpoint = rateSetpoint;
    return runPID(innerG_[axis], innerSt_[axis], dt);
}

// ============================================================================
//  computeInnerOnly — Acro 모드: 내부 루프만 계산
//  setpoint는 외부에서 스틱 각속도 값을 직접 넣어야 합니다.
// ============================================================================
float PID_Cascade::computeInnerOnly(uint8_t axis, float dt)
{
    if (axis > 2) return 0.0f;
    return runPID(innerG_[axis], innerSt_[axis], dt);
}

// 외부 루프만 계산 (향후 분리 실행 시 사용)
float PID_Cascade::computeOuterOnly(uint8_t axis, float dt)
{
    if (axis > 2) return 0.0f;
    return runPID(outerG_[axis], outerSt_[axis], dt);
}

// ============================================================================
//  reset / resetAll — PID 상태 초기화
//  ─────────────────────────────────────────────────────────────────────────
//  시동(Arm) 시 반드시 호출해야 합니다.
//  ★ prevFB = 0 으로 리셋하면 첫 루프에서 D항이 (현재값-0)/dt = 거대한 스파이크!
//     → prevFB를 현재 feedback으로 초기화하여 첫 루프 D항 스파이크 방지
// ============================================================================
void PID_Cascade::reset(uint8_t axis)
{
    if (axis > 2) return;
    float outerFB = outerSt_[axis].feedback;  // 현재 각도값 보존
    float innerFB = innerSt_[axis].feedback;  // 현재 자이로값 보존
    outerSt_[axis] = PIDState();
    innerSt_[axis] = PIDState();
    outerSt_[axis].prevFB = outerFB;  // ★ 현재값으로 초기화 → 첫 루프 D=0
    innerSt_[axis].prevFB = innerFB;
}
void PID_Cascade::resetAll()
{
    for (int a = 0; a < 3; a++) reset(a);
}

void PID_Cascade::resetYawITerm()
{
    // ★ Yaw I항만 0으로 초기화 (암밍 전 와인드업 제거용)
    innerSt_[AXIS_YAW].iTerm = 0.0f;
    innerSt_[AXIS_YAW].output   = 0.0f;
}

void PID_Cascade::decayYawITerm(float factor)
{
    // ★ Yaw I항 감쇠: 스틱 중앙 시 I항을 factor배씩 줄임
    // factor=0.92: 1ms 루프 기준 약 80ms에 50% 감소
    innerSt_[AXIS_YAW].iTerm *= factor;
}
