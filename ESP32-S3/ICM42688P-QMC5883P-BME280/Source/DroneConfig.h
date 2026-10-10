// ============================================================================
//  DroneConfig.h  —  QuadDrone ESP32-S3 중앙 설정 파일  (v8)
//  작성자: Mark Youngsoo Ahn  |  버전: 2.4.0
//
//  ★ 버전 관리 규칙
//  ─────────────────────────────────────────────────────────────────────────
//  형식: v{메이저}_{YYMMDD}_{NNN}
//    메이저 : 펌웨어 대분류 (현재 v8)
//    YYMMDD : 수정 날짜 (예: 260607 = 2026년 06월 07일)
//    NNN    : 해당 날짜 내 일련번호 (001부터 시작)
//
//  예시: v8_260607_001 = 2026-06-07 첫 번째 빌드
//        v8_260607_002 = 2026-06-07 두 번째 빌드
//
//  버전 이력:
//    v8_260603_001  RTH 검증 완료, BMP180 I2C 수정, Yaw 부호 수정
//    v8_260607_001  June 3 상태 복원 (pitch 피드백 원복, GPS 진단코드 제거)
// ============================================================================
#define FW_VERSION      "v8_260704_003"   // ICM42688P 최종 확정 PID (Inner KP=1.5, KD=0.015)
#define FW_DATE         "2026-07-04"
#define FW_SERIAL       3
//  ★ 보드: ESP32-S3 (단일) — 이전 버전에서 잘못 추가된 BOARD_ESP32 분기 제거됨
//    (일반 ESP32 DevKitC는 별도 독립 테스트용으로만 사용, 본 펌웨어 대상 아님)

//    1. DShot Bidir(양방향) 지원 추가
//       - ESC → FC eRPM 피드백 (GCR 4b5b 인코딩)
//       - 요구 ESC: BLHeli_32 또는 AM32 (Bidir 펌웨어)
//    2. RPM 동적 노치 필터 추가
//       - 모터 RPM 실시간 추적 → 자이로 진동 주파수 제거
//       - 4모터 × 2고조파 × 3축 = 24개 Biquad 노치 필터
//    3. DroneConfig [22] 섹션 신규 추가
// ============================================================================
#pragma once
#ifndef DRONECONFIG_H
#define DRONECONFIG_H

// ============================================================================
//  [1] IMU 선택 — 반드시 하나만 활성화
//  ─────────────────────────────────────────────────────────────────────────
//  GY87 (MPU6050)  : Phase 3 완료. I2C. MARK4 테스트 기준.
//  ICM42688P ★추천 : Phase 4 목표. SPI 24MHz. 32kHz 자이로. 노이즈 극소.
// ============================================================================
//#define IMU_SELECT_GY87           // ← 현재 사용 (GY-87 / MPU6050 I2C)
#define IMU_SELECT_ICM42688P    // ← Phase 4 활성화 (ICM-42688-P SPI)

#if defined(IMU_SELECT_GY87) && defined(IMU_SELECT_ICM42688P)
  #error "[DroneConfig] IMU_SELECT_GY87 와 IMU_SELECT_ICM42688P 중 하나만 선택하세요!"
#endif
#if !defined(IMU_SELECT_GY87) && !defined(IMU_SELECT_ICM42688P)
  #error "[DroneConfig] IMU_SELECT_GY87 또는 IMU_SELECT_ICM42688P 를 선택하세요!"
#endif

// ============================================================================
//  [2] 기압 센서 옵션 (BMP280)
// ============================================================================
// ICM42688P는 자체 기압센서 없음 → BME280/BMP280 별도 연결
// GY87에는 BMP180 내장이지만 정밀도 향상을 위해 BMP280 권장
#define USE_BARO_BMP280

// ============================================================================
//  [3] 추가 옵션 하드웨어
// ============================================================================
#define USE_GPS
//#define USE_SONAR
#define USE_ELRS
#define USE_DSHOT

// ── 자력계 옵션 (RTH 및 GPS 항법에만 사용, AHRS 제외) ──────────────────────
// QMC5883P I2C 자력계 모듈
// 주의: 비행 자세 제어(AHRS)에는 사용하지 않음 → 모터 자기 간섭 방지
//       RTH의 기수 방향 결정 및 GPS 항법에만 활용
#define USE_MAGNETOMETER   // ← RTH 정밀도 향상 시 활성화

// 자기 편각 (지역별 상이, 한국 서울 기준: 약 -8°)
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml 에서 확인
#define MAGNETIC_DECLINATION_DEG  (-8.0f)

// ============================================================================
//  [4] GPIO 핀 배치
// ============================================================================
// ── I2C 버스 (GY-87 / QMC5883P 자력계 공용) ──
#define PIN_I2C_SDA       8
#define PIN_I2C_SCL       9
#define I2C_FREQ_HZ       400000UL

// ── ICM42688P SPI 버스 (Phase 4 신규) ──
// ★ 핀 배정 이력:
//   1차 시도: CS=34 SCK=36 MISO=37 MOSI=35
//     → GPIO34: 모듈에 따라 PSRAM(Octal SPI) 예약 핀일 수 있어 사용 불가 사례 확인
//   2차 시도: CS=5 SCK=18 MISO=19 MOSI=23 (독립 ICM42688P 테스트보드에서 검증된 값)
//     → 본 펌웨어 통합 시 충돌 발견:
//        GPIO5  = PIN_MOTOR_M2 (모터2)      ← 충돌
//        GPIO18 = PIN_ELRS_RX  (수신기)      ← 충돌
//        GPIO19 = ESP32-S3 네이티브 USB D-   ← 충돌 위험
//   3차 확정: 기존 핀맵과 충돌 없는 새 핀으로 배정 (아래)
#define PIN_SPI_SCK         12    // SPI Clock
#define PIN_SPI_MISO        13    // SPI MISO
#define PIN_SPI_MOSI        11    // SPI MOSI
#define PIN_CS_ICM42688P    10    // ICM-42688-P Chip Select
#define SPI_FREQ_ICM        8000000UL   // 8 MHz (최대 24MHz, 초기에는 보수적으로)

// ── BMP280 기압센서 SPI (USE_BARO_BMP280 활성 시) ──
#define PIN_CS_BMP280       14
#define SPI_FREQ_BMP280     10000000UL

// ── QMC5883P 자력계 I2C (USE_MAGNETOMETER 활성 시) ──
// I2C 버스 공유 (SDA/SCL 동일 사용)
#define QMC5883P_I2C_ADDR   0x2C    // QMC5883P 기본 주소


// ── ELRS 수신기 UART ──
#define ELRS_SERIAL       Serial1
#define PIN_ELRS_RX       18
#define PIN_ELRS_TX       17
#define ELRS_BAUD         420000UL

// ── DShot300 모터 핀 (RMT 하드웨어 전용) ──
//  Betaflight 배치:
//    M1 = Rear  Right (RR, CW)   → GPIO4
//    M2 = Front Right (FR, CCW)  → GPIO5
//    M3 = Rear  Left  (RL, CCW)  → GPIO6
//    M4 = Front Left  (FL, CW)   → GPIO7
#define PIN_MOTOR_M1      4
#define PIN_MOTOR_M2      5
#define PIN_MOTOR_M3      6
#define PIN_MOTOR_M4      7

// ── GPS UART ──
// ★ BZGNSS BZ-181: u-blox M10050, 기본값 115200bps / 10Hz (FLASH 저장)
//   이전 9600 설정이 통신 실패 원인이었음!
#define GPS_SERIAL        Serial2
#define PIN_GPS_RX        16
#define PIN_GPS_TX        2
#define GPS_BAUD          115200UL   // ★ 9600→115200 (BZ-181 기본값)

// ── GPS 진단 모드 (연결 확인 후 반드시 비활성화) ──
//#define GPS_DEBUG   // ← 정상 비행 시 반드시 주석 처리!

// ── 초음파 HC-SR04 ──
#define PIN_SONAR_TRIG    36
#define PIN_SONAR_ECHO    37

// ── 배터리 전압 ADC ──
// 4S LiPo: 최대 16.8V → ESP32-S3 ADC 최대 3.3V로 분압 필요
// ─────────────────────────────────────────────────────────────────────────
// ★ 권장 분배저항 조합 (4S 기준):
//   R1=100kΩ, R2=22kΩ: 16.8V → 3.03V  ← 표준 보유 부품
//   R1=68kΩ,  R2=15kΩ: 16.8V → 3.04V  ← 대안
//   R1=47kΩ,  R2=10kΩ: 16.8V → 2.95V  ← 대안
//
// BATTERY_DIVIDER = (R1+R2)/R2
//   R1=100k, R2=22k → (100+22)/22 = 5.545
//   R1=68k,  R2=15k → (68+15)/15  = 5.533
//   R1=47k,  R2=10k → (47+10)/10  = 5.700
//
// 배선: LiPo+ → R1 → GPIO1(ADC) → R2 → GND
// ─────────────────────────────────────────────────────────────────────────
// ★ 진단 기록 (2026-06-30) — 최종 결론: 보드 고장 아님, 소프트웨어 버그였음
//   1차: GPIO1 단독 측정 시 3.618V 고정 출력 (Divider 케이블 자체는 2.4V 정상)
//        → GPIO1 핀 손상으로 1차 오판, GPIO3으로 이전 시도.
//   2차: GPIO3으로 이전 후에도 3.458V 동일 증상 → "ADC1 자체 고장"으로 2차 오판.
//   3차: 오실로스코프 파형을 자세히 분석한 결과, 순수 아날로그 노이즈가 아닌
//        주기적 디지털 펄스 다발(burst) 패턴임을 확인. 원인은 setup()의
//        pinMode(PIN_BATTERY_ADC, INPUT) 호출 — ESP32 계열에서 ADC 핀에
//        pinMode(INPUT)을 걸면 디지털 입력 버퍼가 함께 켜진 채로 남아,
//        인접한 고속 디지털 신호(ELRS UART 등)에 반응해 내부적으로
//        토글되며 ADC 샘플링에 노이즈를 주입함. 해당 pinMode() 호출을
//        제거하여 해결(QuadDrone_ESP32S3.ino setup() 참고).
//   → GPIO1 정상 사용 가능. 보드 교체 불필요했음.
#define PIN_BATTERY_ADC   1
#define BATTERY_DIVIDER   6.000f   // ★ R1=100kΩ, R2=20kΩ → (100+20)/20=6.0  실측 보정 후 조정
#define BATT_WARN_VOLTAGE  14.0f   // 4S 경고 전압 (셀당 3.5V × 4)

// ── 기타 ──
#define PIN_LED           48
#define PIN_BUZZER        40   // ★ 39→40 변경 (GPIO39 하드웨어 불량)

// ============================================================================
//  [5] 핀 연결 요약표 (v8)
//  ─────────────────────────────────────────────────────────────────────────
//  ★ v8 추가: DShot Bidir는 추가 핀 없음 (같은 모터 핀 사용, 방향 전환)
//  ★ Bidir 신뢰성을 위해 각 모터 핀에 10kΩ 풀업 저항 권장 (GPIO→3.3V)
// ============================================================================

// ============================================================================
//  [6] ELRS 채널 매핑
// ============================================================================
#define CH_ROLL           1
#define CH_PITCH          2
#define CH_THROTTLE       3
#define CH_YAW            4
#define CH_ARM            5
#define CH_FLIGHTMODE     6
#define CH_AUX_FUNC       7
#define CH_IMU_CALIB      8
#define CH_RTH            9
#define CH_BUZZER         10   // ★ CH10: 추락 탐색 부저 (수동 ON/OFF)
#define CH_AUX_FUNC       7
#define CH_IMU_CALIB      8
#define CH_RTH            9
#define ELRS_NUM_CHANNELS 16

#define CRSF_CENTER       992
#define CRSF_MIN          172
#define CRSF_MAX          1811

#define SW_LOW_MAX        1200.0f
#define SW_MID_MIN        1300.0f
#define SW_MID_MAX        1700.0f
#define SW_HIGH_MIN       1800.0f

enum SwPos3 { SW3_LOW = 0, SW3_MID = 1, SW3_HIGH = 2 };

// ── 비행 모드 (CH6, 3단 스위치) ──
#define FLIGHTMODE_ANGLE    SW3_LOW    // 자동 수평 복귀
#define FLIGHTMODE_HORIZON  SW3_MID    // ★ 신규: 중앙=Angle, 풀편향=Acro(플립 가능)
#define FLIGHTMODE_ACRO     SW3_HIGH   // 완전 레이트 제어

// ── CH7 보조 기능 (3단 스위치) ──
// LOW  = 없음 (일반 비행)
// MID  = ALTHOLD (기압계/소나 고도 유지)
// HIGH = SONAR 단독 고도 유지 (소나 센서 연결 시)
#define AUX_NORMAL          SW3_LOW
#define AUX_ALTHOLD         SW3_MID
#define AUX_SONAR_HOLD      SW3_HIGH   // ★ 소나 전용 고도 유지

// ── Horizon 모드 파라미터 ──
// HORIZON_SENSITIVITY: 스틱 편향 몇 %에서 Acro로 완전 전환하는지
//   1.0 = 풀 편향에서 Acro  (기본값, Betaflight 기본과 유사)
//   0.5 = 50% 편향에서 Acro (더 빠르게 Acro로 전환)
#define HORIZON_SENSITIVITY   1.0f

// ============================================================================
//  [7] 비행 한계값
// ============================================================================
#define MAX_ANGLE_ROLL_DEG    45.0f
#define MAX_ANGLE_PITCH_DEG   45.0f
#define MAX_RATE_ROLL_DPS    200.0f  // Acro 초기 안전값

#define MAX_RATE_PITCH_DPS   200.0f  // Acro 초기 안전값, 익숙해지면 400까지 증가
#define MAX_RATE_YAW_DPS     200.0f
#define THROTTLE_ARM_MIN     1020.0f

// ── ★ Betaflight 분석 적용: 캘리브레이션 중 흔들림 감지 ──
// performGyroCalibration()의 표준편차 검증 패턴 참고.
// 캘리브레이션 도중 기체가 흔들리면(사람이 부딛히거나 진동 등) 평균값이
// 왜곡되어 잘못된 오프셋이 저장될 수 있다. 표준편차가 이 임계값을 넘으면
// 캘리브레이션을 처음부터 재시도한다.
#define GYRO_CAL_STDDEV_THRESHOLD_DPS   3.0f   // 이 이상 흔들리면 재시도
#define GYRO_CAL_MAX_RETRY              5      // 최대 재시도 횟수 (무한루프 방지)

// ── ★ Betaflight 분석 적용: I항 itermRelax (사전 예방형 안티윈드업) ──
// 기존 itermLimit(클램핑)·Freeze(시동 전 차단)는 이미 적용되어 있으나,
// "스틱을 빠르게 움직이는 동안 I항이 과도하게 누적되는 것"을 사전에
// 막아주는 itermRelax는 없었음. Betaflight pid.c itermRelax 패턴 참고.
// 스틱 급변 정도(setpoint의 고주파 성분)에 비례해 I항 누적을 감쇄시킨다.
#define ITERM_RELAX_THRESHOLD_DPS   40.0f  // 이 변화율 이상이면 I 누적 0%로 감쇄
#define ITERM_RELAX_LPF_HZ          15.0f  // setpoint 저역통과 차단주파수

// ── 스틱 Deadband ─────────────────────────────────────────────────────────
// 중립(1500µs) ±이 범위 이내는 1500으로 처리 (Betaflight 기본값 5µs)
// Pitch 1502µs → Deadband 5 → 1500으로 처리 → tP = 0° (전진 편향 없음)
#define STICK_DEADBAND_US    5.0f

// ============================================================================
//  [7-B] 자이로 저역통과필터(LPF) — Betaflight gyro_lpf 동일 원리
//  ─────────────────────────────────────────────────────────────────────────
//  Betaflight가 기체가 달라져도 기본 PID가 통하는 핵심 이유:
//  자이로 원시값을 PID에 넣기 전에 LPF로 "표준화"하기 때문이다.
//
//  센서가 바뀌면 노이즈 특성이 달라진다:
//    GY-87 (MPU-6050): 노이즈 밀도 0.005 °/s/√Hz
//    ICM-42688-P:      노이즈 밀도 0.0028 °/s/√Hz (약 절반)
//
//  LPF가 없으면 → 센서마다 PID(특히 KD)를 처음부터 다시 찾아야 함
//  LPF가 있으면 → 필터 이후 신호가 "균일"해져 PID 재사용 가능
//
//  구현: PT1(1차 저역통과필터) 방식 — 가장 단순하고 연산 부하 거의 없음
//    y_new = y_old + alpha × (x - y_old)
//    alpha = dt / (dt + 1 / (2π × cutoff_Hz))
//
//  단계:
//    1단계 — GYRO_LPF1_HZ 차단주파수로 PT1 적용 (주요 노이즈 제거)
//    2단계 — GYRO_LPF2_HZ 차단주파수로 PT1 한 번 더 (잔여 노이즈 제거)
//    2단계를 1단계와 같은 주파수로 두 번 적용하면 PT2(2차)와 동등한 효과
//
//  튜닝 가이드:
//    - 주파수가 낮을수록 노이즈는 더 잘 잡히나 응답이 느려짐(지연 증가)
//    - ICM-42688-P에는 200Hz가 출발점으로 적당
//    - Wobbling이 심하면 낮춰보고(150Hz), 응답이 둔하면 올려본다(250Hz)
//    - 2단계를 비활성화하려면 GYRO_LPF2_HZ를 0으로 설정
// ── Gyro LPF 활성화 여부 (주석처리하면 해당 단계 비활성화) ────────────────
#define GYRO_LPF1_ENABLED            // 1단계 활성 — 주석처리: #//define
#define GYRO_LPF2_ENABLED            // 2단계 활성 — 주석처리: #//define

// ── Gyro LPF 차단주파수 (Hz) — 값만 조정, 활성/비활성은 위 플래그로 ───
// 값이 낮을수록 노이즈 차단 강화 (응답 약간 느려짐)
// 값이 높을수록 응답 빠름 (노이즈 차단 약화)
// 권장 범위: 100~300Hz / 출발점: 200Hz
#define GYRO_LPF1_HZ    200.0f       // 1단계 LPF 차단주파수
#define GYRO_LPF2_HZ    200.0f       // 2단계 LPF 차단주파수

// ============================================================================
//  [8] Cascade PID 게인
//  ─────────────────────────────────────────────────────────────────────────
//  ★ 기체 변경 이력:
//    ZMR250  (구): 2205 2400KV 4S 5" 3엽
//    MARK4 225mm (현): Phase 3 테스트 기체. PID 재조정 필요 (3축 지그 사용)
//  튜닝 순서: Outer KP → Inner KP → Inner KD → KI (마지막)
//
//  진단: 이전 Inner KI=0.5 → I항 급속 누적 → 진동 원인
//  수정: KI를 0.05로 대폭 감소, Outer KP 10.0으로 수평 복원력 강화
// ============================================================================

// ── Outer (각도 → 각속도 목표) ──
// KP: 각도 오차 → 각속도 목표 변환. 10° × KP = 목표 각속도(deg/s)
// KI: ★ 전진 쏠림 보정 — 지속 각도 오차 누적 → 정상 상태 각도 오프셋 제거
//     Betaflight의 Level_I 역할. iLimit=150(±150deg/s)으로 와인드업 방지.
//     테스트 권장 순서: 0.01 → 0.03 → 0.05 (지그 확인 후 비행)
#define OUTER_KP_YAW    2.5f
#define OUTER_KI_YAW    0.0f
#define OUTER_KD_YAW    0.0f
#define OUTER_KP_PITCH   6.0f   // ★ ICM-42688P 기체 실측 확정값
#define OUTER_KI_PITCH   0.02f
#define OUTER_KD_PITCH   0.05f
#define OUTER_KP_ROLL    6.0f
#define OUTER_KI_ROLL    0.02f
#define OUTER_KD_ROLL    0.05f

// ── Inner (각속도 → 모터 출력) ──
// ★ D항 공식: D = -KD × (gyro변화 / dt)  → dt=0.001이므로 KD는 매우 작아야 함
// ★ ICM-42688P 기체 실측값: GY-87보다 KP와 KD가 낮음
//   (ICM 노이즈가 절반 수준이라 같은 KD라도 D항 반응이 달라짐 → 낮춰야 안정)
#define INNER_KP_YAW    6.0f    // ★ ICM 기체에서 6.0으로 확인됨
#define INNER_KI_YAW    0.02f
#define INNER_KD_YAW    0.0f
#define INNER_KP_PITCH   1.5f   // ★ ICM 기체 최종 확정값 (Gyro LPF 추가 후 실비행 튜닝)
#define INNER_KI_PITCH   0.02f
#define INNER_KD_PITCH   0.015f // ★ ICM 기체 최종 확정값 (LPF로 노이즈 감소 → KD 더 높게 가능)
#define INNER_KP_ROLL    1.5f
#define INNER_KI_ROLL    0.02f
#define INNER_KD_ROLL    0.015f

// ============================================================================
//  [8-B] Betaflight 호환 변환 스케일 (참고용)
//  ─────────────────────────────────────────────────────────────────────────
//  Betaflight(BF)값 → 우리 시스템 변환 공식:
//    KP_us = BF_P × 0.1          (단순 ÷10)
//    KI_us = BF_I × BF_I_SCALE   (BF I-term 내부 처리 보정 포함)
//    KD_us = BF_D × BF_D_SCALE   (BF D LPF 보정 포함)
//
//  공식 유도:
//    P: BF내부 = P_ui/100 × PTERM_SCALE(0.032) × 500(출력범위)≈0.16×P_ui
//       우리  = Kp → Kp = P_ui × 0.16/1.6 ≈ P_ui × 0.1
//    I: 이론상 = P_scale 동일 (연속적분 loop-rate 독립)
//       실제  = BF I-relax·anti-gravity 보정 → 체감 1/200
//       → Ki_us ≈ I_ui × 0.00025 (관측값 기준)
//    D: BF Biquad LPF 적용 시 실효게인 약 50% 감소
//       BF D항 내부스케일 ≈ D_ui/100 × 500 = D_ui×5
//       우리 D항: Kd × (dError/dt)
//       → Kd_us ≈ D_ui × 0.000178 (관측값 기준)
//
//  ★★ 안전 사용 지침 ★★
//    - 변환값은 ZMR250에서 검증된 값. MARK4 등 다른 기체는 비례적으로 다를 수 있음
//    - 반드시 3축 지그에서 먼저 확인 → 이상 없으면 실제 비행
//    - BF값 그대로 주석 해제 후 즉시 비행 금지 (기체/모터가 다르면 위험)
//
//  검증된 변환 예시 (ZMR250 5" 4S 2205 2400KV):
//    BF: P=45 I=80 D=28  →  KP=4.5  KI=0.020  KD=0.005  ✓
//  MARK4에서 사용 시: 지그 테스트 후 scale 인수 미세 조정 필요
//
//  새 기체에서 BF 튜닝값 사용법:
//    #define BF_COMPAT_P  50   // BF Configurator P 값
//    #define BF_COMPAT_I  80
//    #define BF_COMPAT_D  30
//    // 아래 값을 위 INNER_KP_PITCH 등에 대입:
//    //  KP = BF_COMPAT_P * 0.100f
//    //  KI = BF_COMPAT_I * 0.00025f
//    //  KD = BF_COMPAT_D * 0.000178f
// ============================================================================

#define PID_OUTER_LIMIT  250.0f
#define PID_INNER_LIMIT  500.0f
#define PID_ITERM_LIMIT  150.0f

#define ALT_KP           5.0f
#define ALT_KI           0.1f
#define ALT_KD           3.0f
#define ALT_ITERM_LIMIT  100.0f
#define ALT_OUTPUT_LIMIT 200.0f
#define ALT_HOLD_TARGET_CM  80.0f

// ============================================================================
//  [9] 루프 타이밍
// ============================================================================
#define LOOP_TIME_US      1000UL    // 1ms = 1kHz

// ============================================================================
//  [10] Mahony AHRS 게인 (GY-87 선택 시)
//  ─────────────────────────────────────────────────────────────────────────
//  Kp: 수렴 속도. 시상수 ≈ 1/(Kp×2)
//    Kp=0.5 → 시상수 1초  → 2~3초 지연 느낌 (이전 테스트에서 느렸던 원인)
//    Kp=3.0 → 시상수 0.17초 → 거의 실시간 (Betaflight급)
//    Kp=5.0 → 시상수 0.1초 → 완전 실시간 (진동 시 약간 노이즈 가능)
//  Ki: 자이로 드리프트 보정. 보통 0으로 시작.
// ============================================================================
#define MAHONY_KP   3.0f   // ★ 0.5→3.0: 8년 전처럼 실시간 응답 복구
#define MAHONY_KI   0.0f

// ============================================================================
//  [11] GY-87 / MPU6050 설정
// ============================================================================
#define MPU6050_GYRO_FS   3
#define MPU6050_ACCEL_FS  2
#define MPU6050_DLPF_CFG  0

#define MPU6050_AX_OFFSET   150
#define MPU6050_AY_OFFSET    39
#define MPU6050_AZ_OFFSET     3
#define MPU6050_GX_OFFSET    27
#define MPU6050_GY_OFFSET   -47
#define MPU6050_GZ_OFFSET   -17

// ============================================================================
//  [13] BMP280 SPI 설정
// ============================================================================
#define BMP280_OSS_TEMP     2
#define BMP280_OSS_PRESS    5
#define BMP280_FILTER       2

// ============================================================================
//  [14] DShot 설정
// ============================================================================
#define DSHOT_NUM_MOTORS      4    // 모터 수 (RPM_Filter 등 공유 상수)
#define DSHOT_THROTTLE_MIN    48
#define DSHOT_THROTTLE_MAX    2047
#define DSHOT_THROTTLE_IDLE   100

// ============================================================================
//  [15] 페일세이프
// ============================================================================
// ── Failsafe 설정 ──
// FAILSAFE_ACTION 옵션:
//   FAILSAFE_CUT  : 즉시 모터 정지 (GPS/기압계 불필요)
//   FAILSAFE_LAND : 기압계로 자세 유지 착륙 (GPS 불필요 — 기압계만으로 동작)
//   FAILSAFE_RTH  : GPS Fix+홈 있으면 복귀, GPS 없으면 LAND 폴백
#define FAILSAFE_CUT    0
#define FAILSAFE_LAND   1
#define FAILSAFE_RTH    2

#define FAILSAFE_ACTION       FAILSAFE_CUT    // ★ 기본: 즉시 모터 정지
#define FAILSAFE_TIMEOUT_MS   1500
#define FAILSAFE_LAND_US      1300.0f         // LAND 모드 착륙 스로틀 (조정 가능)

// ============================================================================
//  [16] GPS 설정
// ============================================================================
#define GPS_MIN_SATS          6
#define GPS_HDOP_MAX          2.5f
#define RTH_ASCEND_ALT_M      10.0f
#define RTH_SPEED_MPS         3.0f
#define RTH_LAND_DESCENT_MPS  0.3f
#define RTH_RADIUS_M          1.5f

// ============================================================================
//  [17] 초음파 설정
// ============================================================================
#define SONAR_MAX_CM          300
#define SONAR_MIN_CM          5
#define SONAR_TRIGGER_US      10
#define SONAR_TIMEOUT_US      25000
#define SONAR_FILTER_ALPHA    0.2f
#define SONAR_UPDATE_MS       50

// ============================================================================
//  [18] NVS 저장 키
// ============================================================================
#define NVS_NAMESPACE        "dronecfg"
#define NVS_KEY_IMU_CALIB    "imu_cal"
#define NVS_KEY_HOME_LAT     "home_lat"
#define NVS_KEY_HOME_LON     "home_lon"
#define NVS_KEY_HOME_ALT     "home_alt"

// ============================================================================
//  [19] 축 인덱스 상수
// ============================================================================
#define AXIS_YAW      0
#define AXIS_PITCH    1
#define AXIS_ROLL     2
#define AXIS_THROTTLE 3

// ============================================================================
//  [20] 비행 모드 상수
// ============================================================================
#define FLIGHT_MODE_ANGLE    0
#define FLIGHT_MODE_HORIZON  1   // ★ 신규: Angle↔Acro 블렌딩
#define FLIGHT_MODE_ACRO     2
#define FLIGHT_MODE_ALTHOLD  3

// ============================================================================
//  [21] 수학 상수
// ============================================================================
#define DEG_TO_RAD   0.01745329251994f
#define RAD_TO_DEG   57.29577951308f
#define GRAVITY_MS2  9.80665f
#define EARTH_RADIUS_M  6371000.0f

// ============================================================================
//  [22] v8 신규: DShot Bidir + RPM 동적 노치 필터
//  ─────────────────────────────────────────────────────────────────────────
//
//  ★ 요구 ESC (Bidir 미지원 ESC에서는 DSHOT_BIDIR 주석 처리):
//    - BLHeli_32: BLHeliSuite32 → Bidirectional DSHOT = ON
//    - AM32:      AM32 Configurator → Bidirectional DSHOT = ON
//
//  ★ 하드웨어 권장:
//    - 모터 신호 핀(GPIO4~7)과 3.3V 사이에 10kΩ 풀업 저항
//    - 풀업 이유: Bidir에서 FC가 핀 해제 후 ESC가 드라이브하기 전
//                라인이 플로팅되지 않도록 HIGH로 유지
//
//  ★ Bidir DShot 동작 원리:
//    - 일반 DShot: FC만 전송 (단방향)
//    - Bidir DShot: FC가 인버티드 DShot 전송 → ESC가 GCR 인코딩으로 eRPM 응답
//    - GCR (Gray Code Return, 실제로는 4b5b 인코딩):
//        4비트 니블 → 5비트 코드 → 20비트 직렬 스트림으로 전송
//
//  ★ RPM → 진동 주파수 변환:
//    - 모터 회전수 RPM → 기본 진동 주파수: f₁ = RPM / 60 [Hz]
//    - 2고조파: f₂ = 2 × RPM / 60 [Hz]
//    - 예) 모터 15,000 RPM → f₁=250Hz, f₂=500Hz
//    - 이 주파수에서 자이로 신호를 Biquad 노치 필터로 제거
//
//  ★ 필터 뱅크 구성:
//    - 4모터 × HARMONICS(2) × 3축(X,Y,Z) = 24개 Biquad 필터
//    - ESP32-S3 FPU로 ~1µs 이내 처리 (PID 부담 없음)
//
//  ★ DSHOT_BIDIR 비활성화 시:
//    - USE_RPM_FILTER도 자동 비활성화
//    - v7과 동일한 단방향 DShot 동작
// ============================================================================

// Bidir DShot 활성화 (Bidir 미지원 ESC → 주석 처리)
//#define DSHOT_BIDIR

// RPM 동적 노치 필터 (DSHOT_BIDIR 없이는 작동 안 함)
#ifdef DSHOT_BIDIR
  #define USE_RPM_FILTER
#endif

// ── 모터 자극(극) 수 ──
// 모터 라벨 또는 Betaflight Configurator → Motors → Motor poles 참고
// 소형 레이싱 모터: 12 또는 14극이 일반적
// eRPM → 기계적 RPM 변환: RPM = eRPM ÷ (MOTOR_POLES ÷ 2)
#define MOTOR_POLES           14

// ── RPM 필터 파라미터 ──
// 추적할 고조파 수 (1=기본주파수만, 2=기본파+2배, 최대 3 권장)
#define RPM_FILTER_HARMONICS   2

// 노치 Q팩터 — 높을수록 좁은 노치, 낮을수록 넓은 노치
// 권장: 1.0~3.0 (너무 낮으면 위상 지연, 너무 높으면 인접 주파수 영향)
#define RPM_FILTER_Q           1.5f

// 노치 필터 작동 범위 [Hz]
// RPM 주파수가 이 범위 밖이면 해당 필터 비활성화 (계수 미갱신)
#define RPM_FILTER_MIN_HZ    100.0f   // 최소: 너무 낮으면 자이로 DC 응답 왜곡
#define RPM_FILTER_MAX_HZ    480.0f   // 최대: 루프 500Hz 나이퀴스트 이하

// RPM 신호 저역통과 필터 차단주파수 [Hz]
// RPM 급변 시 노치 필터 계수가 급격히 바뀌는 것을 완화
// 높을수록 RPM 추종 빠름, 낮을수록 안정적
#define RPM_FILTER_LPF_HZ     50.0f

// DShot Bidir RMT 수신 버퍼 크기
// GCR 응답: 최대 20비트 × 2 edge = 40 symbol + 여유 = 48
#define DS_RX_BUF_SYMBOLS     48

// ── Bidir 타이밍 (DShot300 기준) ──
// ESC 응답 지연: ~26µs (ESC 처리 시간)
// GCR 응답 길이: 20비트 × 3.33µs/비트 ≈ 67µs
// 합계: ~93µs → 1kHz(1000µs) 루프 내 충분히 수신 가능
#define DS_BIDIR_RESPONSE_TIMEOUT_US  200  // ESC 응답 최대 대기 [µs]

// ── RMT RX 펄스 범위 ──
// GCR 최소 펄스: 약 1비트 × 37% = 1.23µs → 1000ns로 설정
// GCR 최대 펄스: 최대 연속 동일비트 ≈ 4비트 × 3.33µs = 13.3µs → 14000ns
// 이보다 긴 펄스 = EOF (수신 완료 트리거)
#define DS_RX_MIN_PULSE_NS    1000    // 최소 유효 펄스 [ns]
#define DS_RX_MAX_PULSE_NS   14000    // 최대 유효 펄스 [ns] (초과 시 EOF)

// ============================================================================
//  [23] 시리얼 실시간 PID 튜너  (v8 신규)
//  ─────────────────────────────────────────────────────────────────────────
//  활성화 시: 비행 중 시리얼 명령으로 PID 게인을 실시간 변경 가능
//  비활성화 시: 코드 크기/처리 시간 감소 → 최종 비행 버전에서 주석 처리
//
//  ★ 시리얼 명령 (115200 bps, 줄끝 없음 또는 Newline)
//  ─────────────────────────────────────────────────────────────────────────
//  게인 변경:
//    opi 4.0   → Outer KP (Pitch/Roll)
//    oii 0.0   → Outer KI
//    odi 0.0   → Outer KD
//    ipi 4.0   → Inner KP (Pitch/Roll)
//    iii 0.05  → Inner KI
//    idi 0.15  → Inner KD
//    ipy 3.5   → Yaw Inner KP
//    iiy 0.3   → Yaw Inner KI
//
//  유틸리티:
//    show      → 현재 게인 전체 출력
//    reset     → PID 상태 초기화 (I항 리셋)
//    plot on   → 시리얼 플로터 모드 ON
//    plot off  → 시리얼 플로터 모드 OFF
//
//  플로터 출력 형식 (Arduino IDE Serial Plotter):
//    TgtPitch:XX Pitch:XX Rate:XX OuterOut:XX InnerOut:XX M1:XXX M2:XXX
// ============================================================================
//#define USE_PID_TUNER   // 주석 처리 시 튜너 비활성화

// ============================================================================
//  [24] WiFi PID 모니터링 대시보드
//  ─────────────────────────────────────────────────────────────────────────
//  //#define USE_WIFI_MONITOR    // ← 주석 해제 시 WiFi AP + 브라우저 대시보드 활성
//
//  활성화 전 요구사항:
//    Arduino 라이브러리 매니저에서 "ArduinoJson" by Benoit Blanchon 설치
//
//  접속 방법:
//    1. FC 전원 ON → WiFi "QuadDrone-v8" 접속 (비밀번호 없음)
//    2. 브라우저에서 http://192.168.4.1 접속
//    3. 실시간 Roll/Pitch/Yaw, 모터 출력, PID 파라미터 확인 및 조정
// ============================================================================
//#define USE_WIFI_MONITOR   // WiFi AP + 브라우저 PID 대시보드 활성
#define WIFI_MONITOR_SSID  "QuadDrone-v8"  // 접속할 WiFi 이름 (변경 가능)

// ── 시리얼 모니터 ──
// Pitch 편향 진단(MONITOR_PITCH_CHAIN), Yaw 봉 테스트(MONITOR_YAW) → 완료, 제거됨
// 표준 출력: USE_PID_TUNER의 P:/G: 출력 사용 (arming 후 100ms 간격)

#ifdef USE_PID_TUNER
  #define TUNER_SERIAL_BAUD   115200
  #define TUNER_PRINT_HZ      20     // 플로터 출력 주파수 [Hz]
#endif

#endif // DRONECONFIG_H
