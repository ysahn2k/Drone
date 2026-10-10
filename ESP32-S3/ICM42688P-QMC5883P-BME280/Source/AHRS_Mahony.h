// ============================================================================
//  AHRS_Mahony.h  —  Mahony 상보 필터 (자세 추정 알고리즘)  (v5)
//
//  ★ 이 파일의 역할
//  ─────────────────────────────────────────────────────────────────────────
//  GY-87의 MPU6050에서 얻은 자이로+가속도계 데이터를 이용해
//  Roll(롤) / Pitch(피치) / Yaw(요) 각도를 계산합니다.
//
//  ★ AHRS란?
//  Attitude and Heading Reference System (자세·방위 기준 시스템)
//  드론이 현재 얼마나 기울어져 있는지를 알아내는 알고리즘입니다.
//
//  ★ Mahony 필터란?
//  자이로(빠르지만 드리프트 발생) + 가속도계(느리지만 절대 기준)를
//  서로 보완하여 정확한 각도를 구하는 상보 필터입니다.
//  Sebastian Mahony가 발표한 오픈소스 알고리즘입니다.
//  참고: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
//  ★ 쿼터니언(Quaternion)이란?
//  3D 회전을 나타내는 4개 숫자 (w, x, y, z)입니다.
//  오일러 각도(Roll/Pitch/Yaw)보다 짐벌락 문제가 없어 수치 계산에 유리합니다.
//  내부적으로 쿼터니언으로 계산하고 마지막에 각도로 변환합니다.
//
//  ★ 사용법
//    MahonyAHRS ahrs;
//    ahrs.begin(1000.0f);                          // 1000 Hz로 초기화
//    ahrs.updateIMU(gx, gy, gz, ax, ay, az);       // 자이로+가속도만으로 업데이트
//    ahrs.update(gx,gy,gz, ax,ay,az, mx,my,mz);   // 지자계 포함 업데이트
//    float roll  = ahrs.getRoll();                  // 결과 읽기
//    float pitch = ahrs.getPitch();
//    float yaw   = ahrs.getYaw();
// ============================================================================
#pragma once
#ifndef AHRS_MAHONY_H
#define AHRS_MAHONY_H

#include <Arduino.h>
#include <math.h>          // sqrtf, atan2f, asinf 등 수학 함수
#include "DroneConfig.h"   // MAHONY_KP, MAHONY_KI, RAD_TO_DEG 등

// ============================================================================
//  MahonyAHRS 클래스
//  ─────────────────────────────────────────────────────────────────────────
//  클래스(Class)는 관련 데이터와 함수를 하나로 묶는 C++ 구조입니다.
//  여기서는 필터 상태(쿼터니언 등)와 업데이트 함수를 묶었습니다.
// ============================================================================
class MahonyAHRS {
public:
    // ── 생성자 (Constructor) ──
    // 클래스 변수를 기본값으로 초기화합니다.
    MahonyAHRS();

    // ── 초기화 함수 ──
    // setup() 에서 한 번 호출합니다.
    // sampleRateHz : 업데이트 주기(Hz), 루프 주파수와 일치시켜야 합니다
    //               예) LOOP_TIME_US=1000 → sampleRateHz=1000
    // kp, ki : Mahony 게인 (DroneConfig.h의 MAHONY_KP/KI가 기본값)
    void begin(float sampleRateHz,
               float kp = MAHONY_KP,
               float ki = MAHONY_KI);

    // ── 쿼터니언 초기화 ──
    // 재시작 또는 자세가 크게 틀어졌을 때 호출합니다.
    void reset();

    // ── IMU 전용 업데이트 (지자계 없이) ──
    // 매 루프마다 호출합니다.
    // gx,gy,gz : 자이로 데이터 [rad/s]  — DEG_TO_RAD 곱해서 변환 필요
    // ax,ay,az : 가속도계 데이터 (단위 무관, 내부에서 정규화됨)
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az);

    // ── 지자계 포함 전체 업데이트 ──
    // GY-87의 HMC5883L 지자계 데이터를 사용하면 Yaw 드리프트가 줄어듭니다.
    // mx,my,mz : 지자계 데이터 (단위 무관, 내부에서 정규화됨)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);

    // ── 결과 읽기 — 각도 [도, °] ──
    float getRoll()  const;   // 롤  각도 (-180 ~ +180°)
    float getPitch() const;   // 피치 각도  (-90 ~  +90°)
    float getYaw()   const;   // 요  각도 (-180 ~ +180°)

    // 세 축을 한 번에 읽기 (세 번 각각 호출하는 것보다 효율적)
    void  getEulerDeg(float &roll, float &pitch, float &yaw) const;

    // ── 내부 쿼터니언 직접 읽기 ──
    // 디버깅이나 외부 알고리즘에서 쿼터니언을 직접 사용할 때 씁니다.
    void getQuaternion(float &w, float &x, float &y, float &z) const;

    // ── 게인 런타임 변경 ──
    void setKp(float kp) { twoKp_ = 2.0f * kp; }
    void setKi(float ki) { twoKi_ = 2.0f * ki; }

private:
    // twoKp_, twoKi_ : 계산 효율을 위해 2를 미리 곱해 저장합니다
    float twoKp_;      // 2 × 비례 게인
    float twoKi_;      // 2 × 적분 게인

    // 쿼터니언 상태 (w, x, y, z) — 초기값은 단위 쿼터니언 (1,0,0,0)
    float q0_, q1_, q2_, q3_;

    // 적분 피드백 항 (자이로 드리프트 보정 누적값)
    float iFBx_, iFBy_, iFBz_;

    // 샘플링 주기 역수 (1/Hz) — 적분 계산에 사용
    float invSampleFreq_;

    // 빠른 역제곱근 (Quake III 알고리즘)
    // 1/√x 를 일반 나눗셈보다 수십 배 빠르게 계산합니다
    static float invSqrt(float x);
};

#endif // AHRS_MAHONY_H
