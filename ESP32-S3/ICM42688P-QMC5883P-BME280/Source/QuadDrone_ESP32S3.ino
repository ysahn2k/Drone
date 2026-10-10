// ============================================================================
//  QuadDrone_ESP32S3.ino  —  쿼드콥터 비행 컨트롤러 메인 스케치  (v8)
//
//  작성자 : Mark Youngsoo Ahn (ysahn2000@hotmail.com)
//  버전   : 2.4.0
//  보드   : ESP32-S3-WROOM-1 DevKit (240 MHz, 내장 FPU)
//
// ============================================================================
//  ★ v8 주요 변경사항 (v7 대비)
//  ─────────────────────────────────────────────────────────────────────────
//  [1] DShot Bidir(양방향) 추가 — DShot_ESP32.h v8
//      - ESC → FC eRPM 피드백 수신 (GCR 4b5b 디코딩)
//      - 활성화: DroneConfig.h [22]의 #define DSHOT_BIDIR
//      - 요구 ESC: BLHeli_32 또는 AM32 (BLHeliSuite → BIDIRECTIONAL DSHOT = ON)
//
//  [2] RPM 동적 노치 필터 추가 — RPM_Filter.h/cpp v8 신규
//      - 모터 RPM 실시간 추적 → Biquad 노치 필터로 자이로 진동 제거
//      - 4모터 × 2고조파 × 3축 = 24개 Biquad 필터
//      - readIMU() 내 gyroX/Y/Z에 필터 적용
//      - 활성화: DroneConfig.h [22]의 #define USE_RPM_FILTER
//
//  [3] lib/RPM_Filter/ 신규 라이브러리 추가
//
// ============================================================================
//  ★ v8 시스템 구조도
// ============================================================================
//
//  [조종기] ──ELRS──→ [수신기] ──CRSF──→ [ESP32-S3]
//                                              │
//             ┌────────────────────────────────┤
//             │                                │
//   ┌─────────┴────────┐             ┌─────────┴──────────┐
//   │   옵션 A: GY-87   │             │ 옵션 B: ICM-42688-P│
//   │   (I2C 통신)      │             │ (SPI 통신)         │
//   └─────────┬────────┘             └─────────┬──────────┘
//             └──────────[Mahony AHRS]──────────┘
//                               │
//              ┌────────────────▼────────────────┐
//              │         자이로 Raw 값            │
//              │  gyroX, gyroY, gyroZ [deg/s]    │
//              └────────────────┬────────────────┘
//                               │  ★ v8 추가
//              ┌────────────────▼────────────────┐
//              │     RPM 동적 노치 필터           │
//              │  4모터 × 2고조파 × 3축 = 24 BQ  │
//              │  [Bidir DShot RPM 기반 계수]     │
//              └────────────────┬────────────────┘
//                               │
//              ┌────────────────▼────────────────┐
//              │       Cascade PID 제어           │
//              │  [외부: 각도→각속도]              │
//              │  [내부: 각속도→모터출력]          │
//              └────────────────┬────────────────┘
//                               │
//              ┌────────────────▼────────────────┐
//              │        X자형 모터 믹싱           │
//              └────────────────┬────────────────┘
//                               │ DShot300 (인버티드, Bidir)
//              ┌────────────────▼────────────────┐
//              │       ESC × 4 (BLHeli_32)       │
//              │  ↕ Bidir DShot (eRPM 피드백)    │  ★ v8
//              └─────────────────────────────────┘
//
// ============================================================================
//  ★ 파일 구조 (lib/ 폴더)
// ============================================================================
//
//  DroneConfig/          설정 파일 (핀, 게인, 상수 — 여기서만 수정)
//  DShot/                DShot300 + Bidir DShot (v8: GCR RPM 피드백)
//  RPM_Filter/           RPM 동적 노치 필터       ★ v8 신규
//  AHRS_Mahony/          Mahony 상보 필터 (GY-87, ICM-42688-P 공용)
//  IMU_GY87/             GY-87 드라이버 (I2C)
//  IMU_ICM42688P/        ICM-42688-P 드라이버 (SPI) — Phase 4
//  Mag_QMC5883P/         QMC5883P 자력계 (I2C, RTH 전용)
//  Baro_BMP280/          BMP280 기압 센서 (SPI)
//  ELRS_Receiver/        ExpressLRS CRSF 수신기
//  PID_Cascade/          Cascade PID 컨트롤러
//  GPS_RTH/              u-blox GPS + RTH (선택)
//  Sonar/                HC-SR04 초음파 (선택)
//
// ============================================================================
//  ★ 외부 라이브러리 설치 (Arduino IDE → 라이브러리 관리)
// ============================================================================
//
//  ArduinoJson  ← USE_WIFI_MONITOR 선택 시 필요
//
// ============================================================================
//  ★ 보드 설정 (Arduino IDE → 툴 메뉴)
// ============================================================================
//
//  보드          : ESP32S3 Dev Module
//  Upload Speed  : 921600
//  CPU Frequency : 240MHz (WiFi)
//  Flash Mode    : QIO 80MHz
//  Partition     : Default 4MB with spiffs
//
// ============================================================================

// ── [포함 파일] ─────────────────────────────────────────────────────────────
#include "DroneConfig.h"      // 모든 설정값
#include <esp_system.h>       // esp_reset_reason() — 재시작 원인 진단용
#include <string.h>           // strncpy
#include "AHRS_Mahony.h"      // Mahony 상보 필터
#include "ELRS_Receiver.h"    // ExpressLRS CRSF 수신기
#include "DShot_ESP32.h"      // DShot300 + Bidir DShot (v8)
#include "PID_Cascade.h"      // Cascade PID 컨트롤러

#ifdef USE_WIFI_MONITOR
  #include "WiFiMonitor.h"    // ★ WiFi PID 모니터링 대시보드
#endif

// ── ★ v8 신규: RPM 동적 노치 필터 ──
#ifdef USE_RPM_FILTER
  #include "RPM_Filter.h"
#endif

// ── IMU 선택 ──
#ifdef IMU_SELECT_GY87
  #include "IMU_GY87.h"
  IMU_GY87   imu;
  MahonyAHRS ahrs;
#elif defined(IMU_SELECT_ICM42688P)
  // ★ Phase 4: ICM-42688-P SPI 고성능 IMU
  // SPI 24MHz / 32kHz 자이로 / 노이즈 극소 / 비행체 전용 칩
  #include "IMU_ICM42688P.h"
  IMU_ICM42688P imu;
  MahonyAHRS    ahrs;  // 6축 모드 (자력계 없음 → updateIMU 사용)
  SPIClass      spiIMU(FSPI);   // ESP32-S3 FSPI(SPI2)
#endif

// ── 기압 센서 (BMP280 선택 시) ──
#ifdef USE_BARO_BMP280
  #include "Baro_BMP280.h"
  Baro_BMP280 baro;
#endif

// ── 자력계 (RTH/항법 전용, AHRS 제외) ──
#ifdef USE_MAGNETOMETER
  #include "Mag_QMC5883P.h"
  Mag_QMC5883P mag;
  static float magHeadingDeg = 0.0f;  // 기수 방향 [°], RTH에서 사용
#endif

// ── 선택 옵션 ──
#ifdef USE_GPS
  #include "GPS_RTH.h"
  GPS_RTH gps;
#endif

#ifdef USE_SONAR
  #include "Sonar.h"
  Sonar sonar;
#endif

// ── 항상 사용하는 모듈 ──
ELRS_Receiver elrs;
DShot_ESP32   motors;
PID_Cascade   pid;

// ── ★ v8: RPM 필터 객체 ──
#ifdef USE_RPM_FILTER
  RPM_Filter rpmFilter;
#endif

// ── WiFi 모니터 ──
// (WiFiMonitor.h는 위 IMU 블록 안의 USE_WIFI_MONITOR에서 include됨)

// ============================================================================
//  float 보간 헬퍼
// ============================================================================
static inline float fmap(float x,
                          float inMin,  float inMax,
                          float outMin, float outMax)
{
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ── 스틱 Deadband 적용 ────────────────────────────────────────────────────
// 중립(1500µs) 기준 ±STICK_DEADBAND_US 이내이면 정확히 1500으로 처리
// 목적: 조종기 스틱 중립 미세 오프셋(±2~5µs)으로 인한 드리프트 방지
//       Betaflight도 동일한 방식으로 중립 오프셋을 처리함
static inline float applyDeadband(float us)
{
    const float center = 1500.0f;
    if (fabsf(us - center) <= STICK_DEADBAND_US)
        return center;
    return us;
}

// ============================================================================
//  전역 변수
// ============================================================================
static bool     armed         = false;
static uint32_t armTimeMs     = 0;

// ── 부저 상태 ──
// 우선순위: CH10 탐색 > 배터리경고 > Failsafe > 시동/해제 확인
enum BuzzMode : uint8_t {
    BZR_OFF = 0,
    BZR_ARMED,    // 시동: 삐삐(2회 짧은 음)
    BZR_DISARMED, // 해제: 삐-(1회 긴 음)
    BZR_WARN,     // 배터리/Failsafe: 빠른 반복
    BZR_SEARCH    // CH10 추락 탐색: 느린 반복 (최고 우선순위)
};
static BuzzMode  buzzerMode  = BZR_OFF;
static uint32_t  buzzerMs    = 0;
static bool      buzzerPhase = false;
static uint8_t   buzzerTick  = 0;  // 패턴 진행 카운터
static const uint32_t IMU_WARMUP_MS = 2000;  // Mahony AHRS 수렴 안정화 대기시간 (~2초)
// ★ 안전수정: 초기값을 true로 설정. 재부팅(전원 글리치 등) 발생 시
//   조종기 ARM 스위치가 마침 켜진 상태였다면, 기존(false 초기값)에서는
//   "꺼짐→켜짐" 전환으로 잘못 인식되어 재부팅 직후 즉시 재시동되는
//   위험한 사례가 실제로 확인되었다. true로 시작하면 부팅 직후 첫
//   프레임은 항상 "꺼짐 전환"(무해)으로만 처리되고, 실제 ARM에는
//   조종사가 스위치를 명시적으로 OFF→ON 한 번 더 움직여야만 한다.
static bool     prevArmed     = true;
static uint8_t  flightMode    = FLIGHT_MODE_ANGLE;
static bool     failsafeActive= false;
static float    failsafeLandThr = 0.0f;  // ★ LAND/RTH 모드 스로틀 오버라이드
static bool     failsafeRTHStarted = false; // ★ Failsafe가 시작한 RTH (CH9 체크로 중단 방지)
static bool     calibPending  = false;
esp_reset_reason_t g_lastResetReason = ESP_RST_UNKNOWN;  // ★ WiFi 대시보드 표시용
char            g_resetReasonStr[40] = "?";              // ★ 대시보드에 표시할 짧은 문자열

static float    roll  = 0.0f;
static float    pitch = 0.0f;
static float    yaw   = 0.0f;

static float    gyroX = 0.0f;
static float    gyroY = 0.0f;
static float    gyroZ = 0.0f;

static float    currentAltCm  = 0.0f;
static float    currentAltM   = 0.0f;

static uint16_t motorOut[4]   = {0, 0, 0, 0};
static bool     imuFreezeDetected = false;  // 자이로 동결 감지 플래그 (ICM42688P에서는 드뭄)

#ifdef USE_GPS
  static NavCommand navCmd;
#endif

static uint32_t loopTimer = 0;
static float    dt        = 0.001f;

static uint32_t ledTimer  = 0;
static bool     ledState  = false;
static uint32_t telemTimer= 0;
static float    battVoltage = 0.0f;

// ── ★ v8: RPM 디버그 타이머 ──
#ifdef DSHOT_BIDIR
  static uint32_t rpmDebugTimer = 0;
#endif

// ── ★ v8: PID 실시간 튜너 전역 변수 ──
#ifdef USE_PID_TUNER
// 런타임에서 변경 가능한 게인 저장 변수
// (#define 상수는 컴파일 타임 고정 → 이 변수로 오버라이드)
struct TunerGains {
    // Outer (각도→각속도) — Pitch/Roll 공통
    float opi = OUTER_KP_PITCH;
    float oii = OUTER_KI_PITCH;
    float odi = OUTER_KD_PITCH;
    // Inner (각속도→모터) — Pitch/Roll 공통
    float ipi = INNER_KP_PITCH;
    float iii = INNER_KI_PITCH;
    float idi = INNER_KD_PITCH;
    // Yaw Inner
    float ipy = INNER_KP_YAW;
    float iiy = INNER_KI_YAW;
} tg;

static bool     plotMode_    = false;  // true: 플로터 형식 출력
static uint32_t plotTimer_   = 0;
static uint32_t plotInterval_= 1000000UL / TUNER_PRINT_HZ;  // [µs]
#endif

// ============================================================================
//  함수 전방 선언
// ============================================================================
void readIMU();
void readSensors();
void computeFlightControl();
void doArm();
void doDisarm();
void handleFailsafe();
void sendTelemetry();
void updateBuzzer();   // ★ 비차단 부저 업데이트 (메인 루프에서 매 루프 호출)
float readBatteryVoltage();
void ledBlink(uint32_t ms);
#ifdef DSHOT_BIDIR
  void printRPMDebug();
#endif
#ifdef USE_PID_TUNER
  void applyTunerGains();
  void processTunerInput();
  void printTunerGains();
  void printPlotterData();
#endif

// ============================================================================
//  setup()
// ============================================================================
void setup()
{
    // ★ 시리얼 초기화
    // ESP32-S3 DevKitC-1 기준: UART0 TX=GPIO43, RX=GPIO44 (USB-UART 브리지)
    Serial.begin(115200);
    Serial.setTimeout(20);   // ★ 기본 1000ms→20ms: processTunerInput 블로킹 방지
    delay(500);

    Serial.println("\n========================================");
    Serial.println(" QuadDrone ESP32-S3 v8 시작");
    Serial.println(" ★ Bidir DShot + RPM 노치 필터");
    Serial.println("========================================");

    // ── ★ 재시작 원인 진단 ──────────────────────────────────────────────
    // 정상적인 첫 전원 인가(POWERON_RESET)가 아니라면 무언가 비정상적으로
    // 재시작되었다는 뜻이다. 원인별로 대응이 완전히 다르므로 반드시 확인한다.
    //   - POWERON_RESET   : 정상적인 전원 인가 (문제 없음)
    //   - SW_RESET        : 코드에서 esp_restart() 호출 (의도적)
    //   - PANIC_RESET     : 크래시(널 포인터, 배열범위초과 등) → 코드 버그
    //   - INT_WDT/TASK_WDT: 워치독 타임아웃 → 어딘가 너무 오래 블로킹됨
    //   - BROWNOUT_RESET  : 전압 강하 감지 → 전원/배선 문제
    //   - RTCWDT_RTC_RESET: RTC 워치독 → 부팅 자체가 너무 오래 걸림
    {
        esp_reset_reason_t rr = esp_reset_reason();
        const char* rrName = "UNKNOWN";
        switch (rr) {
            case ESP_RST_POWERON:   rrName = "POWERON (정상 전원 인가)"; break;
            case ESP_RST_SW:        rrName = "SW_RESET (코드에서 의도적 재시작)"; break;
            case ESP_RST_PANIC:     rrName = "★PANIC (코드 크래시! 버그 의심)"; break;
            case ESP_RST_INT_WDT:   rrName = "★INT_WDT (인터럽트 워치독 타임아웃)"; break;
            case ESP_RST_TASK_WDT:  rrName = "★TASK_WDT (태스크 워치독 타임아웃, 루프 블로킹 의심)"; break;
            case ESP_RST_WDT:       rrName = "★WDT (기타 워치독 타임아웃)"; break;
            case ESP_RST_BROWNOUT:  rrName = "★BROWNOUT (전압 강하 감지! 전원/배선 점검)"; break;
            case ESP_RST_DEEPSLEEP: rrName = "DEEPSLEEP (절전모드 복귀)"; break;
            case ESP_RST_EXT:       rrName = "EXT (외부 리셋 핀)"; break;
            default: break;
        }
        Serial.printf(" 재시작 원인: %s\n", rrName);
        if (rr != ESP_RST_POWERON && rr != ESP_RST_SW) {
            Serial.println(" ★★★ 비정상 재시작 감지! 위 원인을 반드시 확인하세요 ★★★");
        }
        g_lastResetReason = rr;   // WiFi 대시보드에 표시하기 위해 저장
        strncpy(g_resetReasonStr, rrName, sizeof(g_resetReasonStr) - 1);
        g_resetReasonStr[sizeof(g_resetReasonStr) - 1] = '\0';
    }
    Serial.println("========================================");

#ifdef IMU_SELECT_GY87
    Serial.println("[설정] IMU: GY-87 / MPU6050 (I2C)");
#elif defined(IMU_SELECT_ICM42688P)
    Serial.println("[설정] IMU: ICM-42688-P (SPI) - Phase 4");
#endif

#ifdef DSHOT_BIDIR
    Serial.println("[설정] DShot: 양방향(Bidir) ON");
    Serial.printf("       모터 극수: %d  pole_pairs: %d\n",
                  MOTOR_POLES, MOTOR_POLES / 2);
    Serial.println("       ★ ESC 설정 필수: BLHeliSuite → BIDIRECTIONAL DSHOT = ON");
    Serial.println("       ★ 모터 핀(GPIO4~7)에 10kΩ 풀업 저항 권장");
#else
    Serial.println("[설정] DShot: 단방향");
#endif

    // ── ★ Betaflight 분석 적용: 루프 주기 vs DShot 프로토콜 제약 자동 검증 ──
    // Betaflight validateAndFixGyroConfig()의 motorUpdateRestriction과 동일한 원리.
    // DShot300 프레임 전송 자체에 물리적으로 필요한 최소 시간(헤더+리셋갭) 대비
    // 루프 주기가 너무 짧으면 프레임이 겹쳐 모터가 오작동할 수 있다.
    // 현재는 LOOP_TIME_US=1000(1kHz)으로 충분한 여유가 있으나, 향후 루프 주기를
    // 변경할 경우를 대비해 부팅 시 자동으로 점검한다.
    {
        const uint32_t DSHOT300_MIN_FRAME_US = 100;  // Betaflight 실측 기준값
        if (LOOP_TIME_US < DSHOT300_MIN_FRAME_US) {
            Serial.printf("[치명적 오류] LOOP_TIME_US(%lu) < DShot300 최소 프레임 시간(%lu)!\n",
                          (unsigned long)LOOP_TIME_US, (unsigned long)DSHOT300_MIN_FRAME_US);
            Serial.println("              모터 프레임 겹침으로 오작동 위험 — DroneConfig.h 점검 필요");
            while (1) ledBlink(50);   // 위험 설정이므로 부팅 자체를 차단
        } else if (LOOP_TIME_US < DSHOT300_MIN_FRAME_US * 3) {
            // 여유가 3배 미만이면 위험하진 않지만 권장하지 않는 수준 → 경고만
            Serial.printf("[경고] LOOP_TIME_US(%lu)가 DShot 최소시간 대비 여유가 적습니다 (권장: %lu 이상)\n",
                          (unsigned long)LOOP_TIME_US, (unsigned long)(DSHOT300_MIN_FRAME_US * 3));
        }
    }


#ifdef USE_RPM_FILTER
    Serial.println("[설정] RPM 노치 필터: ON");
    Serial.printf("       Q=%.1f  범위=%.0f~%.0fHz  고조파=%d\n",
                  (float)RPM_FILTER_Q,
                  (float)RPM_FILTER_MIN_HZ, (float)RPM_FILTER_MAX_HZ,
                  RPM_FILTER_HARMONICS);
#endif

    // ── LED ──
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // ★ 부저 초기화: 반드시 LOW로 시작 (미초기화 시 3.3V 상시 출력 방지)
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    // 부팅 완료 비프 (100ms)
    digitalWrite(PIN_BUZZER, HIGH); delay(100);
    digitalWrite(PIN_BUZZER, LOW);

    // ── 배터리 ADC ──
    // ★ 추가 정리(2026-06-30): pinMode() 제거에 이어, analogSetPinAttenuation()도
    //   제거한다. ADC_11db는 어차피 ESP32 Arduino 코어의 기본값이라 이 호출은
    //   기능적으로 아무 효과가 없는 군더더기였다. Arduino Uno에서 analogRead()
    //   대상 핀을 pinMode 등으로 별도 설정하지 않는 것과 같은 원칙으로,
    //   ADC 핀은 analogRead() 호출 시점에 라이브러리가 알아서 필요한 설정을
    //   하도록 맡기고 애플리케이션 코드에서는 아예 손대지 않는 것이 가장 안전하다.
    analogReadResolution(12);   // 분해능 설정만 유지 (보드 전역 설정, 특정 GPIO 모드와 무관)

    // ── IMU 버스 초기화 ──
#ifdef IMU_SELECT_GY87
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_FREQ_HZ);
    Serial.printf("[I2C] SDA=GPIO%d SCL=GPIO%d %.0fkHz\n",
                  PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ / 1000.0f);
#elif defined(IMU_SELECT_ICM42688P)
    // ICM42688P: SPI (FSPI)
    spiIMU.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    Serial.printf("[SPI/FSPI] SCK=GPIO%d MISO=GPIO%d MOSI=GPIO%d CS=GPIO%d\n",
                  PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CS_ICM42688P);
    // I2C (QMC5883P 자력계 + BMP280용)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_FREQ_HZ);
#endif

#ifdef USE_MAGNETOMETER
    Serial.printf("[I2C] 자력계(QMC5883P) SDA=GPIO%d SCL=GPIO%d\n",
                  PIN_I2C_SDA, PIN_I2C_SCL);
#endif

    // ── IMU 초기화 ──
    float loopHz = 1000000.0f / (float)LOOP_TIME_US;
#ifdef IMU_SELECT_GY87
    if (!imu.begin()) {
        Serial.println("[오류] GY-87 초기화 실패!");
        while (1) ledBlink(100);
    }
    ahrs.begin(loopHz);
    Serial.printf("[AHRS] Mahony 6축 %.0f Hz (GY-87)\n", loopHz);

#elif defined(IMU_SELECT_ICM42688P)
    if (!imu.begin(PIN_CS_ICM42688P, spiIMU, SPI_FREQ_ICM)) {
        Serial.println("[오류] ICM-42688-P 초기화 실패! CS/SPI 핀 확인");
        while (1) ledBlink(100);
    }
    ahrs.begin(loopHz);
    Serial.printf("[AHRS] Mahony 6축 %.0f Hz (ICM-42688-P)\n", loopHz);
    Serial.println("  [주의] 자력계(QMC5883P)는 RTH에만 사용, AHRS 제외");
#endif

    // ── Gyro LPF 설정 출력 ──
    Serial.printf("[LPF] Gyro LPF1=%.0fHz, LPF2=%.0fHz\n",
                  (float)GYRO_LPF1_HZ, (float)GYRO_LPF2_HZ);
#if defined(GYRO_LPF1_ENABLED) && defined(GYRO_LPF2_ENABLED)
    Serial.println("  → 1+2단계 활성 (PT2와 동등, 노이즈 감쇠 강함)");
#elif defined(GYRO_LPF1_ENABLED)
    Serial.println("  → 1단계만 활성 (PT1)");
#else
    Serial.println("  → LPF 비활성 (DroneConfig.h에서 활성화 가능)");
#endif

#ifdef USE_MAGNETOMETER
    if (!mag.begin(QMC5883P_I2C_ADDR, Wire)) {
        Serial.println("[경고] QMC5883P 초기화 실패 (RTH 헤딩 불가)");
    }
#endif

#ifdef USE_BARO_BMP280
#if defined(IMU_SELECT_ICM42688P)
    // ★ ICM42688P와 동일한 SPI 버스(spiIMU, FSPI) 공유
    //   spiIMU.begin()은 위에서 이미 호출됨 → 여기서 재호출 불필요
    //   SCK/MISO/MOSI 공유, CS만 별도(PIN_CS_BMP280=14)
    if (!baro.begin(PIN_CS_BMP280, SPI_FREQ_BMP280, spiIMU)) {
#else
    // GY-87 등 다른 IMU와 함께 사용 시: 기본 SPI 버스 별도 초기화 필요
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    if (!baro.begin(PIN_CS_BMP280, SPI_FREQ_BMP280, SPI)) {
#endif
        Serial.println("[경고] BMP280 초기화 실패");
    } else {
        delay(100);
        baro.update();
        baro.calibrateSeaLevel();
        Serial.printf("[BMP280] 해수면 기압: %.1f Pa\n",
                      baro.getSeaLevelPressure());
    }
#endif

    // ── ELRS 수신기 ──
#ifdef USE_ELRS
    elrs.begin(ELRS_SERIAL, PIN_ELRS_RX, PIN_ELRS_TX, ELRS_BAUD);
    elrs.setFailsafeTimeout(FAILSAFE_TIMEOUT_MS);
    Serial.println("[ELRS] 수신기 준비");
#endif

    // ── GPS ──
#ifdef USE_GPS
    // ★ gps.begin() 내부에서 Serial2.begin() 호출하므로 중복 호출 불필요
    gps.begin(GPS_SERIAL, GPS_BAUD);
    Serial.printf("[GPS] BZGNSS BZ-181 초기화: %lu bps, RX=GPIO%d TX=GPIO%d\n",
                  GPS_BAUD, PIN_GPS_RX, PIN_GPS_TX);

#ifdef GPS_DEBUG
    // ─────────────────────────────────────────────────────────────────────
    // GPS 진단 모드: 5초간 원시 바이트를 ASCII + HEX 동시 출력
    //
    // 정상 NMEA 출력 예시:
    //   $GNGGA,123456.00,3729.12,N,12700.67,E,1,08,1.2,50.0,M,...*XX
    //
    // UBX 바이너리 출력 시 (u-blox 전용 프로토콜):
    //   HEX: B5 62 01 07 5C 00 ...  ← $ 아닌 B5로 시작
    //   → NMEA 출력으로 전환 필요
    //
    // 아무것도 없을 때:
    //   → RX/TX 핀 교차 또는 5V 전원 미공급 확인
    // ─────────────────────────────────────────────────────────────────────
    Serial.println();
    Serial.println("======= GPS 진단 시작 (5초) =======");
    Serial.println("기대 출력: $GNGGA, $GNRMC 등 $ 시작 문장");
    Serial.println("-----------------------------------");

    uint32_t t0 = millis();
    int nmeaLines = 0, ubxBytes = 0, totalBytes = 0;
    char lineBuf[128]; int lineIdx = 0;

    while (millis() - t0 < 5000) {
        if (GPS_SERIAL.available()) {
            uint8_t b = GPS_SERIAL.read();
            totalBytes++;
            if (b == 0xB5) ubxBytes++;
            if (b == '$') { lineIdx = 0; }
            if (lineIdx < 127) lineBuf[lineIdx++] = (char)b;
            if (b == '\n' && lineIdx > 5) {
                lineBuf[lineIdx] = '\0';
                Serial.print(lineBuf);
                nmeaLines++;
                lineIdx = 0;
            }
        }
    }

    Serial.println("-----------------------------------");
    Serial.printf("수신 바이트: %d개  NMEA 라인: %d개\n", totalBytes, nmeaLines);
    if (nmeaLines > 0)
        Serial.println("✅ GPS 통신 정상 (배터리 5V 연결 필수)");
    else
        Serial.println("❌ GPS 데이터 없음 → 5V 전원 및 TX/RX 핀 확인");
    Serial.println("======= GPS 진단 끝 =======");
    Serial.println();
#endif  // GPS_DEBUG
#endif  // USE_GPS

    // ── 초음파 ──
#ifdef USE_SONAR
    sonar.begin(PIN_SONAR_TRIG, PIN_SONAR_ECHO);
    Serial.println("[Sonar] HC-SR04 초기화");
#endif

    // ── DShot300 (+ Bidir) ──
#ifdef USE_DSHOT
    const int motorPins[4] = {
        PIN_MOTOR_M1,  // M1: Rear  Right (CW)
        PIN_MOTOR_M2,  // M2: Front Right (CCW)
        PIN_MOTOR_M3,  // M3: Rear  Left  (CCW)
        PIN_MOTOR_M4   // M4: Front Left  (CW)
    };
    if (!motors.begin(motorPins)) {
        Serial.println("[오류] DShot RMT 초기화 실패!");
        while (1) ledBlink(100);
    }
    Serial.println("========================================");
    Serial.println(" ★ 안전 주의 ★  프롭 제거 후 테스트!");
    Serial.println("========================================");
    motors.arm(1000);
#endif

    // ── ★ v8: RPM 동적 노치 필터 초기화 ──
#ifdef USE_RPM_FILTER
    float fsHz = 1000000.0f / (float)LOOP_TIME_US;
    rpmFilter.begin(fsHz);
    Serial.println("[RPMFilter] 초기화 완료");
#endif

    // ── ★ v8: PID 실시간 튜너 초기화 ──
#ifdef USE_PID_TUNER
    applyTunerGains();  // DroneConfig.h 초기값 → PID 객체에 적용
    Serial.println("[PID 튜너] 활성화 완료");
    Serial.println("  명령어: opi/oii/odi=Outer KP/KI/KD,  ipi/iii/idi=Inner KP/KI/KD");
    Serial.println("  예시: 'ipi 4.0' → Inner KP=4.0 즉시 적용");
    Serial.println("  plot on/off → 시리얼 플로터 모드 전환");
    printTunerGains();
#endif

    // ── 초기화 완료 ──
    digitalWrite(PIN_LED, LOW);
    Serial.println();
    Serial.println("[준비 완료] CH5 스위치로 시동을 거세요.");
    Serial.printf ("빌드: %s  (%s  #%03d)\n", FW_VERSION, FW_DATE, FW_SERIAL);

#ifdef USE_WIFI_MONITOR
    initWiFiMonitor();   // ★ WiFi AP + 브라우저 대시보드 시작
    strncpy(g_wifiSnap.resetReason, g_resetReasonStr, sizeof(g_wifiSnap.resetReason) - 1);
    g_wifiSnap.resetReason[sizeof(g_wifiSnap.resetReason) - 1] = '\0';
#endif

    loopTimer = micros();
}

// ============================================================================
//  loop() — 메인 제어 루프 (1 kHz)
// ============================================================================
void loop()
{
    // ── [1] 루프 타이밍 ──
    while (micros() - loopTimer < LOOP_TIME_US) {}
    uint32_t nowUs = micros();
    dt = (float)(nowUs - loopTimer) * 1.0e-6f;
    loopTimer = nowUs;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.005f)  dt = 0.005f;

    // ── [2] ELRS 수신기 처리 ──
#ifdef USE_ELRS
    elrs.update();

    if (elrs.isFailsafe() && armed) {
        if (!failsafeActive) {
            failsafeActive = true;
            Serial.println("[★ 페일세이프] RC 신호 끊김!");
        }
        handleFailsafe();
        // ★ FAILSAFE_CUT: doDisarm()으로 armed=false → 아래 computeFlightControl 건너뜀
        // ★ FAILSAFE_LAND/RTH: throttleUs 수정 후 계속 실행하여 자세 유지
#if FAILSAFE_ACTION == FAILSAFE_CUT
        return;   // CUT은 모터 정지 후 즉시 종료
#endif
    }
    failsafeActive = false;

    SwPos3 modeSw = elrs.getFlightModeSwitch();
    if      (modeSw == SW3_LOW)  flightMode = FLIGHT_MODE_ANGLE;
    else if (modeSw == SW3_MID)  flightMode = FLIGHT_MODE_HORIZON;  // ★ Horizon
    else                         flightMode = FLIGHT_MODE_ACRO;

    // CH7: ALTHOLD (소나/기압 고도 유지) — CH6에서 분리
    SwPos3 auxSw = elrs.getAuxFuncSwitch();

#ifdef USE_GPS
    // ★ RTH 트리거: CH9 전용 (CH7-HIGH 제거 — 스위치 통과 시 오동작 방지)
    // ★ 안전조건: GPS Fix + 홈 저장된 경우에만 RTH 시작 (아니면 무시)
    bool rthReq = elrs.isRTHActive();   // CH9 HIGH
    if (rthReq && !gps.isRTHActive() && armed) {
        if (gps.hasValidFix() && gps.hasHome()) {
            failsafeRTHStarted = false;  // CH9 수동 RTH
            gps.startRTH();
        } else {
            Serial.println("[RTH] GPS Fix 없음 → RTH 무시 (Fix 확보 후 재시도)");
        }
    }
    // ★ Failsafe가 시작한 RTH는 CH9 OFF로 중단하지 않음
    //    CH9 수동 RTH만 CH9 OFF 시 중단
    if (!rthReq && gps.isRTHActive() && !failsafeRTHStarted) gps.stopRTH();
#endif

    if (elrs.isIMUCalibReq() && !armed) {
        if (!calibPending) {
            calibPending = true;
            Serial.println("[캘리브레이션] 드론을 수평으로 유지...");
#ifdef IMU_SELECT_GY87
            imu.calibrate(2000);
            // ★ 캘리브레이션 완료 — 저장된 오프셋값 즉시 출력 (변화 확인용)
            {
                const GY87_Cal& c = imu.getCalData();
                Serial.println("[캘리브레이션 완료] ─────────────────────");
                Serial.printf("  가속도(acc): axOfs=%6d  ayOfs=%6d  azOfs=%6d\n",
                              c.axOfs, c.ayOfs, c.azOfs);
                Serial.printf("  자이로(gyr): gxOfs=%6d  gyOfs=%6d  gzOfs=%6d\n",
                              c.gxOfs, c.gyOfs, c.gzOfs);
                Serial.println("─────────────────────────────────────────");
            }
#elif defined(IMU_SELECT_ICM42688P)
            imu.calibrate(2000);
            // ICM42688P: calibrate() 내부에서 오프셋 출력 및 NVS 저장 처리
#endif
        }
    } else {
        calibPending = false;
    }

    bool armSw = elrs.isArmed();
    if ( armSw && !prevArmed) doArm();
    if (!armSw &&  prevArmed) doDisarm();
    prevArmed = armSw;
#endif

    // ── [3] IMU 읽기 + 자세 계산 + ★ RPM 노치 필터 ──
    readIMU();

    // ── [4] 보조 센서 ──
    readSensors();

    // ── [5] GPS ──
#ifdef USE_GPS
    gps.update();
    // Arming 시점에 GPS Fix가 있으면 즉시 홈 저장,
    // Fix 없었다가 나중에 얻으면 홈 폴백 저장 (두 경우 모두 처리)
    if (armed && !gps.hasHome() && gps.hasValidFix()) {
        gps.saveHome(currentAltM);
        Serial.printf("[GPS] 홈 저장 (Armed): lat=%.6f lon=%.6f alt=%.1fm sats=%d\n",
                      gps.getHomeLat(), gps.getHomeLon(), currentAltM, gps.getNumSats());
    }
#endif

    // ── [6] 비행 제어 ──
    if (armed) {
#ifdef USE_GPS
        // ★ RTH 항법 계산 (활성 시)
        // computeFlightControl()은 항상 실행 — navCmd.throttleUs를 내부에서 적용
        if (gps.isRTHActive()) {
            navCmd = gps.computeRTH(roll, pitch, yaw, currentAltM, dt);
            if (navCmd.state == RTH_LANDED) {
                Serial.println("[RTH] 착륙 완료! 자동 시동 해제");
                doDisarm();
            }
        }
#endif
        computeFlightControl();  // ★ RTH 활성 여부와 무관하게 항상 호출
    } else {
#ifdef USE_DSHOT
        motors.stopAll();
#endif
    }

    // ── [7] LED 상태 ──
    uint32_t blinkMs = 500;
    if (armed) {
#ifdef USE_GPS
        blinkMs = gps.hasValidFix() ? 80 : 150;
#else
        blinkMs = 100;
#endif
    }
    if (millis() - ledTimer > blinkMs) {
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
        ledTimer = millis();
    }

    // ── [7.5] 부저 업데이트 (비차단) ──
    updateBuzzer();

    // ── [8] 텔레메트리 ──
    // ★ USE_PID_TUNER 활성 시: Serial 출력 부하로 타이밍 지연 방지를 위해 10Hz
    //    USE_PID_TUNER 비활성 시: 20Hz (50ms) 정상
    //    주의: 텔레메트리 센서 초기 등록 시 USE_PID_TUNER 비활성화 권장
#ifdef USE_PID_TUNER
    if (millis() - telemTimer >= 100) {   // 10Hz (PID 튜너 충돌 방지)
#else
    if (millis() - telemTimer >= 50) {    // 20Hz
#endif
        telemTimer = millis();
        sendTelemetry();
    }

    // ── [9] PID 실시간 모니터 (Armed 상태, 100ms마다) ──
#ifdef USE_PID_TUNER
    if (armed) {
        static uint32_t pidMonMs = 0;
        if (millis() - pidMonMs >= 100) {
            pidMonMs = millis();
            if (imuFreezeDetected) {
                Serial.printf("[★FREEZE] P:%.1f G:%.1f M:%d/%d/%d/%d\n",
                              pitch, gyroY,
                              (int)motorOut[0], (int)motorOut[1],
                              (int)motorOut[2], (int)motorOut[3]);
            } else if (flightMode == FLIGHT_MODE_ACRO) {
                float rsP = fmap(applyDeadband(elrs.getPitchUs()), 1000.f, 2000.f,
                                 -MAX_RATE_PITCH_DPS, MAX_RATE_PITCH_DPS);
                Serial.printf("[ACRO] RS:%.0f G:%.1f M:%d/%d/%d/%d\n",
                              rsP, gyroY,
                              (int)motorOut[0], (int)motorOut[1],
                              (int)motorOut[2], (int)motorOut[3]);
            } else if (flightMode == FLIGHT_MODE_HORIZON) {
                float pitchStick = fmap(applyDeadband(elrs.getPitchUs()), 1000.f,2000.f,-1.f,1.f);
                float lvl = constrain(1.f - fabsf(pitchStick)/HORIZON_SENSITIVITY, 0.f, 1.f);
                Serial.printf("[HRZ] P:%.1f G:%.1f Lvl:%.0f%% M:%d/%d/%d/%d\n",
                              pitch, gyroY, lvl * 100.f,
                              (int)motorOut[0], (int)motorOut[1],
                              (int)motorOut[2], (int)motorOut[3]);
            } else {
                // Angle / AltHold 모드
                Serial.printf("P:%.1f G:%.1f M:%d/%d/%d/%d\n",
                              pitch, gyroY,
                              (int)motorOut[0], (int)motorOut[1],
                              (int)motorOut[2], (int)motorOut[3]);
            }
        }
    }
#endif
    // ── [10] ★ v8: PID 튜너 처리 ──
#ifdef USE_PID_TUNER
    processTunerInput();   // 시리얼 명령 확인

#ifdef USE_WIFI_MONITOR
    // ─ WiFi 대시보드 스냅샷 업데이트 (Core 0이 읽음) ─
    // ★ seqlock: 쓰기 시작/종료를 명시해 Core0의 torn-read 방지
    {
        wifiSnapWriteBegin();
        g_wifiSnap.roll    = roll;   g_wifiSnap.pitch   = pitch;
        g_wifiSnap.yaw     = yaw;
        g_wifiSnap.gx      = gyroX;  g_wifiSnap.gy = gyroY;
        g_wifiSnap.gz      = gyroZ;
        g_wifiSnap.m1      = motorOut[0]; g_wifiSnap.m2 = motorOut[1];
        g_wifiSnap.m3      = motorOut[2]; g_wifiSnap.m4 = motorOut[3];
        g_wifiSnap.armed   = armed;
        g_wifiSnap.battV   = battVoltage;
        g_wifiSnap.uptimeMs = millis();  // ★ 재부팅 감지용 — 이 값이 갑자기 작아지면 리셋 발생
        // ★ 최저 전압 기록: 200ms 폴링으로는 놓칠 수 있는 순간적 전압 강하를
        //   1kHz 루프에서 직접 추적해 절대 놓치지 않도록 함. 0.5V 이상이고
        //   현재 최저치보다 낮을 때만 갱신 (ADC 0V 글리치 자체는 무시).
        if (battVoltage > 0.5f && (g_wifiSnap.minBattV <= 0.5f || battVoltage < g_wifiSnap.minBattV)) {
            g_wifiSnap.minBattV = battVoltage;
        }
        g_wifiSnap.outerKP = tg.opi;
        g_wifiSnap.outerKI = tg.oii;
        g_wifiSnap.outerKD = tg.odi;
        g_wifiSnap.innerKP = tg.ipi; g_wifiSnap.innerKI = tg.iii;
        g_wifiSnap.innerKD = tg.idi;
        g_wifiSnap.yawKP   = tg.ipy; g_wifiSnap.yawKI   = tg.iiy;
        wifiSnapWriteEnd();
    }
    // ─ WiFi에서 PID 변경 명령이 왔으면 적용 ─
    if (g_wifiPidCmd.pending) {
        tg.opi = g_wifiPidCmd.outerKP;
        tg.oii = g_wifiPidCmd.outerKI;  // ★ 추가
        tg.odi = g_wifiPidCmd.outerKD;  // ★ 추가
        tg.ipi = g_wifiPidCmd.innerKP;
        tg.iii = g_wifiPidCmd.innerKI;
        tg.idi = g_wifiPidCmd.innerKD;
        tg.ipy = g_wifiPidCmd.yawKP;
        tg.iiy = g_wifiPidCmd.yawKI;
        applyTunerGains();
        g_wifiPidCmd.pending = false;
        Serial.printf("[WiFi PID] outerKP=%.2f outerKI=%.4f outerKD=%.4f innerKP=%.3f innerKI=%.4f innerKD=%.4f\n",
                      tg.opi, tg.oii, tg.odi, tg.ipi, tg.iii, tg.idi);
    }
#endif // USE_WIFI_MONITOR
    if (plotMode_) {
        uint32_t nowUs2 = micros();
        if (nowUs2 - plotTimer_ >= plotInterval_) {
            plotTimer_ = nowUs2;
            printPlotterData();
        }
    }
#endif

    // ── [10] ★ v8: RPM 디버그 출력 (2초마다) ──
#ifdef DSHOT_BIDIR
    if (millis() - rpmDebugTimer >= 2000) {
        rpmDebugTimer = millis();
        printRPMDebug();
    }
#endif

    // ── [11] ★ DShot TX 타임아웃 진단 (5초마다, 항상 동작) ──
    // Betaflight 분석 적용: txWait()의 무한대기를 유한 타임아웃으로 바꾼 뒤,
    // 그 타임아웃이 실제로 발생하는지 상시 감시. 0이 유지되어야 정상이며,
    // 증가한다면 RMT 하드웨어 이상 또는 루프 과부하를 의심해야 한다.
    {
        static uint32_t dshotDiagMs = 0;
        static uint32_t lastTxTimeoutCount = 0;
        if (millis() - dshotDiagMs >= 5000) {
            dshotDiagMs = millis();
            uint32_t cnt = motors.getTxTimeoutCount();
            if (cnt != lastTxTimeoutCount) {
                Serial.printf("[경고] DShot TX 타임아웃 발생! 누적 %lu회 (RMT 하드웨어 점검 필요)\n",
                              (unsigned long)cnt);
                lastTxTimeoutCount = cnt;
            }
        }
    }
}

// ============================================================================
//  readIMU — IMU 읽기 + 자세 계산 + ★ v8 RPM 노치 필터 적용
//  ─────────────────────────────────────────────────────────────────────────
//  ★ v8 변경사항:
//    - gyroX/Y/Z 계산 후 rpmFilter.filter()로 노치 필터 적용
//    - DSHOT_BIDIR + USE_RPM_FILTER 모두 활성화된 경우에만 동작
// ============================================================================
void readIMU()
{
#ifdef IMU_SELECT_GY87
    imu.update();
    const GY87_Data &g = imu.getData();

    if (g.magReady) {
        ahrs.update(g.gx, g.gy, g.gz,
                    g.ax, g.ay, g.az,
                    g.mx, g.my, g.mz);
    } else {
        ahrs.updateIMU(g.gx, g.gy, g.gz,
                       g.ax, g.ay, g.az);
    }
    ahrs.getEulerDeg(roll, pitch, yaw);

    gyroX = g.gx * RAD_TO_DEG;
    gyroY = g.gy * RAD_TO_DEG;
    gyroZ = g.gz * RAD_TO_DEG;

    if (g.baroReady) {
        currentAltM = g.altitude;          // BMP180 정상 → 기압 고도 사용
    } else {
#ifdef USE_GPS
        // BMP180 이상 → GPS MSL 고도 폴백 (RTH 고도 기준)
        currentAltM = gps.getAlt();        // 0이면 GPS Fix 없음
#endif
    }

#elif defined(IMU_SELECT_ICM42688P)
    // ── ICM-42688-P: 6축 AHRS (accel + gyro) ─────────────────────────────
    imu.update();
    const ICM_Data &gI = imu.getData();

    // 6축 Mahony (자력계 없음 → updateIMU)
    ahrs.updateIMU(gI.gx, gI.gy, gI.gz,
                   gI.ax, gI.ay, gI.az);
    ahrs.getEulerDeg(roll, pitch, yaw);

    gyroX = gI.gx * RAD_TO_DEG;
    gyroY = gI.gy * RAD_TO_DEG;
    gyroZ = gI.gz * RAD_TO_DEG;

#ifdef USE_BARO_BMP280
    baro.update();
    currentAltM = baro.getAltitude();
#endif

#endif // IMU_SELECT

    // ── QMC5883P 자력계 업데이트 (10ms 주기 = 100Hz) ─────────────────────
    // AHRS 루프 밖에서 별도 업데이트 → 모터 자기 간섭 무관
#ifdef USE_MAGNETOMETER
    {
        static uint32_t magMs = 0;
        if (millis() - magMs >= 10) {
            magMs = millis();
            if (mag.update()) {
                // 틸트 보정 헤딩 (AHRS roll/pitch 활용)
                float rollR  = roll  * (float)(M_PI / 180.0);
                float pitchR = pitch * (float)(M_PI / 180.0);
                magHeadingDeg = mag.getHeadingTilted(rollR, pitchR,
                                                      MAGNETIC_DECLINATION_DEG);
            }
        }
    }
#endif

    // ── ★ v8: RPM 노치 필터 적용 ──
    // 모터 진동 주파수(와 고조파)를 자이로 신호에서 제거합니다.
    // PID에 입력되기 전 자이로 신호의 S/N비를 크게 향상시킵니다.
    //
    // 적용 시점: IMU 값을 읽은 직후, PID 계산 직전
    // 적용 축: gyroX(Roll), gyroY(Pitch), gyroZ(Yaw) 모두
    //
    // 조건: DSHOT_BIDIR(RPM 피드백)과 USE_RPM_FILTER 모두 활성 시에만 동작
    // → DSHOT_BIDIR 없으면 RPM 데이터 없으므로 필터 작동 안 함
#ifdef USE_RPM_FILTER
    gyroX = rpmFilter.filter(RPM_FILTER_AXIS_X, gyroX);
    gyroY = rpmFilter.filter(RPM_FILTER_AXIS_Y, gyroY);
    gyroZ = rpmFilter.filter(RPM_FILTER_AXIS_Z, gyroZ);
#endif

    // ── ★ Gyro LPF 1·2단계 ───────────────────────────────────────────────
    //
    // [왜 여기에 적용하는가]
    // RPM 필터 이후, AHRS·PID 모두에 들어가기 직전이라
    // 한 곳에서 처리하면 AHRS와 Inner PID 양쪽에 동시 적용된다.
    //
    // [PT1 LPF 수식]
    //   alpha = dt / (dt + 1 / (2π × cutoff_Hz))
    //   y = y_prev + alpha × (input - y_prev)
    //
    // [alpha의 물리적 의미]
    //   alpha가 1에 가까울수록 필터 효과 약함(원신호에 가까움)
    //   alpha가 0에 가까울수록 필터 효과 강함(이전값 유지, 반응 느림)
    //   1kHz 루프, 200Hz 차단: alpha ≈ 0.557 (적당한 균형)

#ifdef GYRO_LPF1_ENABLED
    {
        // 1단계: GYRO_LPF1_HZ 차단주파수 PT1
        static float lpf1X = 0.0f, lpf1Y = 0.0f, lpf1Z = 0.0f;
        static bool  lpf1Init = false;
        if (!lpf1Init) {                    // 최초 1회: 현재값으로 초기화
            lpf1X = gyroX; lpf1Y = gyroY; lpf1Z = gyroZ;
            lpf1Init = true;
        }
        const float alpha1 = dt / (dt + 1.0f / (2.0f * 3.14159265f * GYRO_LPF1_HZ));
        lpf1X += alpha1 * (gyroX - lpf1X);
        lpf1Y += alpha1 * (gyroY - lpf1Y);
        lpf1Z += alpha1 * (gyroZ - lpf1Z);
        gyroX = lpf1X; gyroY = lpf1Y; gyroZ = lpf1Z;
    }
#endif

#ifdef GYRO_LPF2_ENABLED
    {
        // 2단계: GYRO_LPF2_HZ 차단주파수 PT1 한 번 더
        // 같은 주파수로 두 번 = PT2(2차)와 동등한 효과 → 노이즈 감쇠 더 강함
        static float lpf2X = 0.0f, lpf2Y = 0.0f, lpf2Z = 0.0f;
        static bool  lpf2Init = false;
        if (!lpf2Init) {
            lpf2X = gyroX; lpf2Y = gyroY; lpf2Z = gyroZ;
            lpf2Init = true;
        }
        const float alpha2 = dt / (dt + 1.0f / (2.0f * 3.14159265f * GYRO_LPF2_HZ));
        lpf2X += alpha2 * (gyroX - lpf2X);
        lpf2Y += alpha2 * (gyroY - lpf2Y);
        lpf2Z += alpha2 * (gyroZ - lpf2Z);
        gyroX = lpf2X; gyroY = lpf2Y; gyroZ = lpf2Z;
    }
#endif
}

// ============================================================================
//  readSensors — 보조 센서 업데이트 (v7과 동일)
// ============================================================================
void readSensors()
{
#ifdef USE_SONAR
    sonar.update();
    if (sonar.isValid()) {
        currentAltCm = sonar.getDistanceCm();
    }
#endif

    static float prevVolt = 0.0f;
    float rawVolt = readBatteryVoltage();
    if (prevVolt == 0.0f) prevVolt = rawVolt;
    battVoltage = 0.1f * rawVolt + 0.9f * prevVolt;
    prevVolt    = battVoltage;
}

// ============================================================================
//  computeFlightControl — PID + 모터 믹싱 + DShot 출력
//  ─────────────────────────────────────────────────────────────────────────
//  ★ v8 변경사항:
//    - motors.writeThrottle() 직전에 RPM 필터 계수 갱신
//    - DShot Bidir가 활성화된 경우, 각 모터의 최신 RPM을 rpmFilter에 전달
// ============================================================================
void computeFlightControl()
{
    // ★ 스로틀 우선순위:
    //   1순위: GPS RTH 명령 throttleUs (RTH 활성 시) ← navCmd.throttleUs 적용
    //   2순위: Failsafe LAND 고정 스로틀
    //   3순위: 조종기 스틱 값
    float throttleUs = elrs.getThrottleUs();
#ifdef USE_GPS
    if (gps.isRTHActive() &&
        navCmd.state != RTH_IDLE && navCmd.state != RTH_LANDED) {
        throttleUs = navCmd.throttleUs;  // RTH 상승/수평/하강 명령 적용
    } else
#endif
    if (failsafeLandThr > 0.0f) {
        throttleUs = failsafeLandThr;
    }
    bool  thrLow     = (throttleUs < (THROTTLE_ARM_MIN + 30.0f));

    bool imuWarmedUp = (millis() - armTimeMs) >= IMU_WARMUP_MS;
    if (!imuWarmedUp) {
        motors.stopAll();
        return;
    }

    pid.setITermFreeze(thrLow || imuFreezeDetected);  // ★ 동결 시 I항 누적 차단

    // ★ Yaw I항 보호: 스틱 중앙 복귀 시 I항 빠른 감쇠
    // Betaflight의 "Yaw relaxation" 개념 — 스틱 놓으면 I항을 서서히 0으로 줄임
    {
        float yawUs  = elrs.getChannelUs(CH_YAW);
        float yawStick = fabsf(yawUs - 1500.0f);          // 0(중앙)~500(최대)
        if (yawStick < 30.0f && !thrLow) {                 // 스틱이 거의 중앙
            pid.decayYawITerm(0.92f);                      // I항을 92%로 감쇠 (매 루프)
        }
    }

    // ── ★ TPA (Throttle PID Attenuation) — Betaflight 방식 ──
    // 고스로틀에서 모터 응답성이 높아지므로 Inner KP를 자동 감소
    // tpaBreak(DShot 600) 이하: KP 전체 사용 (저스로틀은 감쇠 없음)
    // tpaBreak(DShot 600) 이상: KP 선형 감소 → DShot 900에서 최소 60% 유지
    //
    //  스로틀↑ → 모터 응답성↑ → 효과적 게인↑ → 진동 유발
    //  TPA:       스로틀↑ →          KP ↓       → 진동 억제
    {
        const float tpaBreak = 600.0f;   // TPA 시작 DShot값
        const float tpaMax   = 900.0f;   // TPA 최대 감쇠 DShot값
        const float tpaMin   = 0.6f;     // 최소 KP 비율 (60%)
        float thr_cur = (float)DShot_ESP32::usToDShot(throttleUs);
        float tpaScale;
        if (thr_cur <= tpaBreak) {
            tpaScale = 1.0f;             // 저스로틀: 감쇠 없음
        } else {
            // 고스로틀: tpaBreak~tpaMax 구간에서 1.0→tpaMin 선형 감소
            float ratio = (thr_cur - tpaBreak) / (tpaMax - tpaBreak);
            tpaScale = 1.0f - ratio * (1.0f - tpaMin);
            if (tpaScale < tpaMin) tpaScale = tpaMin;
        }

#ifdef USE_PID_TUNER
        pid.setInnerGains(AXIS_PITCH, tg.ipi * tpaScale, tg.iii, tg.idi);
        pid.setInnerGains(AXIS_ROLL,  tg.ipi * tpaScale, tg.iii, tg.idi);
#else
        pid.setInnerGains(AXIS_PITCH, INNER_KP_PITCH * tpaScale, INNER_KI_PITCH, INNER_KD_PITCH);
        pid.setInnerGains(AXIS_ROLL,  INNER_KP_ROLL  * tpaScale, INNER_KI_ROLL,  INNER_KD_ROLL);
#endif
    }

    // ── 초음파 고도 유지 ──
#ifdef USE_SONAR
    SwPos3 auxSw2 = elrs.getAuxFuncSwitch();
    // CH7 MID: ALTHOLD (기압+소나 통합), HIGH: SONAR 단독
    bool altHoldOn  = (auxSw2 == AUX_ALTHOLD)
                   || (flightMode == FLIGHT_MODE_ALTHOLD);
    bool sonarOnlyOn = (auxSw2 == AUX_SONAR_HOLD);

    if ((altHoldOn || sonarOnlyOn) && sonar.isValid()) {
        static bool prevAltHold = false;
        if (!prevAltHold) {
            sonar.holdCurrentAlt();
            Serial.printf("[AltHold] 목표: %.1f cm\n", sonar.getTargetAlt());
        }
        prevAltHold = true;

        float stickOffset = (throttleUs - 1500.0f) * 0.05f;
        sonar.setTargetAlt(sonar.getTargetAlt() + stickOffset * dt);

        float altCorr = sonar.computeAltHold(dt);
        throttleUs = 1500.0f + altCorr;
        if (throttleUs < 1100.0f) throttleUs = 1100.0f;
        if (throttleUs > 1900.0f) throttleUs = 1900.0f;
    } else {
        static bool prevAltHold = false;
        if (prevAltHold) sonar.resetAltHold();
        prevAltHold = false;
    }
#endif

    float pidOut[3] = {0.0f, 0.0f, 0.0f};

    // ── Angle / AltHold 모드 ──
    if (flightMode == FLIGHT_MODE_ANGLE ||
        flightMode == FLIGHT_MODE_ALTHOLD) {

        float tR = fmap(applyDeadband(elrs.getRollUs()),  1000.f,2000.f,
                        -MAX_ANGLE_ROLL_DEG,  MAX_ANGLE_ROLL_DEG);
        float tP = fmap(applyDeadband(elrs.getPitchUs()), 1000.f,2000.f,
                        -MAX_ANGLE_PITCH_DEG, MAX_ANGLE_PITCH_DEG);
        float tY = fmap(applyDeadband(elrs.getChannelUs(CH_YAW)), 1000.f,2000.f,
                        -MAX_RATE_YAW_DPS, MAX_RATE_YAW_DPS);

        pid.setOuterSetpoint(AXIS_ROLL,  tR);
        pid.setOuterFeedback(AXIS_ROLL,  roll);
        pid.setInnerFeedback(AXIS_ROLL,  gyroX);

        pid.setOuterSetpoint(AXIS_PITCH, tP);
        pid.setOuterFeedback(AXIS_PITCH, pitch);
        pid.setInnerFeedback(AXIS_PITCH, gyroY);

        pid.setInnerSetpoint(AXIS_YAW, tY);
        pid.setInnerFeedback(AXIS_YAW, gyroZ);

        pidOut[AXIS_ROLL]  = pid.compute(AXIS_ROLL,  dt);
        pidOut[AXIS_PITCH] = pid.compute(AXIS_PITCH, dt);
        pidOut[AXIS_YAW]   = pid.computeInnerOnly(AXIS_YAW, dt);

    }
    // ── ★ Horizon 모드: Angle ↔ Acro 블렌딩 ──
    // 스틱 편향에 따라 자동 수평(Angle)과 레이트(Acro)를 선형 혼합
    // 스틱 중앙 → levelStr=1.0 → 완전 Angle (자동 수평)
    // 스틱 풀편향 → levelStr=0.0 → 완전 Acro (Flip 가능)
    else if (flightMode == FLIGHT_MODE_HORIZON) {
        // 스틱 정규화 (-1.0 ~ +1.0)
        float rollStick  = fmap(applyDeadband(elrs.getRollUs()),  1000.f,2000.f,-1.f,1.f);
        float pitchStick = fmap(applyDeadband(elrs.getPitchUs()), 1000.f,2000.f,-1.f,1.f);
        float tY = fmap(applyDeadband(elrs.getChannelUs(CH_YAW)), 1000.f,2000.f,
                        -MAX_RATE_YAW_DPS, MAX_RATE_YAW_DPS);

        // 레벨링 강도: 스틱 편향이 클수록 Acro로 전환
        float rollLvl  = constrain(1.f - fabsf(rollStick)  / HORIZON_SENSITIVITY, 0.f, 1.f);
        float pitchLvl = constrain(1.f - fabsf(pitchStick) / HORIZON_SENSITIVITY, 0.f, 1.f);

        // Angle 컴포넌트: Outer 루프 실행 → 수평 복원 레이트 계산
        float angleTargetR = rollStick  * MAX_ANGLE_ROLL_DEG;
        float angleTargetP = pitchStick * MAX_ANGLE_PITCH_DEG;
        pid.setOuterSetpoint(AXIS_ROLL,  angleTargetR);
        pid.setOuterFeedback(AXIS_ROLL,  roll);
        pid.setOuterSetpoint(AXIS_PITCH, angleTargetP);
        pid.setOuterFeedback(AXIS_PITCH, pitch);
        float angleRateR = pid.computeOuterOnly(AXIS_ROLL,  dt);
        float angleRateP = pid.computeOuterOnly(AXIS_PITCH, dt);

        // Acro 컴포넌트: 스틱 → 레이트 목표
        float acroRateR = rollStick  * MAX_RATE_ROLL_DPS;
        float acroRateP = pitchStick * MAX_RATE_PITCH_DPS;

        // 블렌딩: level=1 → Angle, level=0 → Acro
        float blendR = rollLvl  * angleRateR + (1.f - rollLvl)  * acroRateR;
        float blendP = pitchLvl * angleRateP + (1.f - pitchLvl) * acroRateP;

        pid.setInnerSetpoint(AXIS_ROLL,  blendR);
        pid.setInnerFeedback(AXIS_ROLL,  gyroX);
        pid.setInnerSetpoint(AXIS_PITCH, blendP);
        pid.setInnerFeedback(AXIS_PITCH, gyroY);
        pid.setInnerSetpoint(AXIS_YAW,   tY);
        pid.setInnerFeedback(AXIS_YAW, gyroZ);

        pidOut[AXIS_ROLL]  = pid.computeInnerOnly(AXIS_ROLL,  dt);
        pidOut[AXIS_PITCH] = pid.computeInnerOnly(AXIS_PITCH, dt);
        pidOut[AXIS_YAW]   = pid.computeInnerOnly(AXIS_YAW,   dt);
    }
    // ── Acro 모드 ──
    else {
        float tR = fmap(applyDeadband(elrs.getRollUs()),  1000.f,2000.f,
                        -MAX_RATE_ROLL_DPS,  MAX_RATE_ROLL_DPS);
        float tP = fmap(applyDeadband(elrs.getPitchUs()), 1000.f,2000.f,
                        -MAX_RATE_PITCH_DPS, MAX_RATE_PITCH_DPS);
        float tY = fmap(applyDeadband(elrs.getChannelUs(CH_YAW)), 1000.f,2000.f,
                        -MAX_RATE_YAW_DPS, MAX_RATE_YAW_DPS);

        pid.setInnerSetpoint(AXIS_ROLL,  tR);
        pid.setInnerFeedback(AXIS_ROLL,  gyroX);
        pid.setInnerSetpoint(AXIS_PITCH, tP);
        pid.setInnerFeedback(AXIS_PITCH, gyroY);
        pid.setInnerSetpoint(AXIS_YAW,   tY);
        pid.setInnerFeedback(AXIS_YAW, gyroZ);

        pidOut[AXIS_ROLL]  = pid.computeInnerOnly(AXIS_ROLL,  dt);
        pidOut[AXIS_PITCH] = pid.computeInnerOnly(AXIS_PITCH, dt);
        pidOut[AXIS_YAW]   = pid.computeInnerOnly(AXIS_YAW,   dt);
    }

    // ── X자형 모터 믹싱 ──
    //
    //         FRONT (기수)
    //  M4(FL,CW)   M2(FR,CCW)
    //       \         /
    //       /         \
    //  M3(RL,CCW)  M1(RR,CW)
    //         REAR
    //
    //  M1(RR,CW) : Thr - Roll + Pitch - Yaw
    //  M2(FR,CCW): Thr - Roll - Pitch + Yaw
    //  M3(RL,CCW): Thr + Roll + Pitch + Yaw
    //  M4(FL,CW) : Thr + Roll - Pitch - Yaw

    if (thrLow) {
        motorOut[0] = 0;
        motorOut[1] = 0;
        motorOut[2] = 0;
        motorOut[3] = 0;
    } else {
        const float dsMin = (float)DSHOT_THROTTLE_IDLE;
        const float dsMax = (float)DSHOT_THROTTLE_MAX;
        float thr = (float)DShot_ESP32::usToDShot(throttleUs);

        float m1 = thr - pidOut[AXIS_ROLL] + pidOut[AXIS_PITCH] - pidOut[AXIS_YAW];
        float m2 = thr - pidOut[AXIS_ROLL] - pidOut[AXIS_PITCH] + pidOut[AXIS_YAW];
        float m3 = thr + pidOut[AXIS_ROLL] + pidOut[AXIS_PITCH] + pidOut[AXIS_YAW];
        float m4 = thr + pidOut[AXIS_ROLL] - pidOut[AXIS_PITCH] - pidOut[AXIS_YAW];

        // ── ★ Betaflight 분석 적용: motorMixRange 기반 비례 스케일링 ──
        // 기존 시프트 로직은 "시프트로 담을 수 있는 경우"는 잘 처리하지만,
        // Roll/Pitch/Yaw 요구 차동값의 폭(range) 자체가 출력 가능 범위보다
        // 크면 시프트만으로는 4개 모터를 모두 담을 수 없어, 이후 constrain()이
        // 일부 모터만 개별적으로 잘라내며 자세 균형이 깨지는 문제가 있었다.
        // Betaflight의 motorMixRange 정규화 패턴을 적용해, 이런 극단적 상황에서는
        // thr을 제외한 차동(Roll/Pitch/Yaw) 성분 자체를 비례적으로 축소하여
        // 반드시 출력 범위 안에 담기도록 한다 — 토크의 "절대크기"는 줄어들지만
        // "상대비율(자세 균형)"은 그대로 보존된다.
        {
            float d1 = m1 - thr, d2 = m2 - thr, d3 = m3 - thr, d4 = m4 - thr;
            float dMax = d1; if (d2>dMax) dMax=d2; if (d3>dMax) dMax=d3; if (d4>dMax) dMax=d4;
            float dMin = d1; if (d2<dMin) dMin=d2; if (d3<dMin) dMin=d3; if (d4<dMin) dMin=d4;
            float mixRange = dMax - dMin;
            float availableRange = (float)DSHOT_THROTTLE_MAX - (float)DSHOT_THROTTLE_IDLE;
            if (mixRange > availableRange && mixRange > 0.0f) {
                float scale = availableRange / mixRange;
                m1 = thr + d1 * scale;
                m2 = thr + d2 * scale;
                m3 = thr + d3 * scale;
                m4 = thr + d4 * scale;
            }
        }

        // ── 모터 포화 보호 (Motor Saturation Protection) ──
        // 전체 스로틀을 시프트하여 PID 차등 출력(토크)을 최대한 보전.
        // ★ 스로틀 비례 상한 캡: 저스로틀에서 시프트로 인한 모터 과속 방지
        //   예) thr=248(1100µs) 상한 = 248+500 = 748 → 최고속의 37%로 제한
        float mArr[4] = {m1, m2, m3, m4};
        float maxM = mArr[0], minM = mArr[0];
        for (int k = 1; k < 4; k++) {
            if (mArr[k] > maxM) maxM = mArr[k];
            if (mArr[k] < minM) minM = mArr[k];
        }

        // 1단계: 최소 모터가 하한 미달 → 전체 상향 시프트 (단, 스로틀+한계 초과 금지)
        if (minM < dsMin) {
            float shift = dsMin - minM;
            // ★ 핵심: 최대 시프트는 (스로틀+PID한계-현재최대) 이하로 제한
            float capMax = thr + (float)PID_INNER_LIMIT;
            if (capMax > dsMax) capMax = dsMax;
            if (maxM + shift > capMax) shift = capMax - maxM;
            if (shift < 0.0f) shift = 0.0f;
            for (int k = 0; k < 4; k++) mArr[k] += shift;
            maxM += shift;
        }

        // 2단계: 최대 모터가 상한 초과 → 전체 하향 시프트
        if (maxM > dsMax) {
            float shift = maxM - dsMax;
            for (int k = 0; k < 4; k++) mArr[k] -= shift;
        }

        motorOut[0] = (uint16_t)constrain(mArr[0], dsMin, dsMax);
        motorOut[1] = (uint16_t)constrain(mArr[1], dsMin, dsMax);
        motorOut[2] = (uint16_t)constrain(mArr[2], dsMin, dsMax);
        motorOut[3] = (uint16_t)constrain(mArr[3], dsMin, dsMax);

        // ── ★ 독립적인 최종 안전 게이트 (armed 기반) ──────────────────────
        // 기존에는 thrLow(스로틀 위치)만으로 모터를 0으로 만들었는데,
        // armed 상태 자체를 직접 확인하는 게이트가 없었다. 재부팅 직후
        // 조종기 스위치가 마침 ARM 위치에 있으면 즉시 재시동되는 사례가
        // 확인되었으므로, thrLow 로직과 무관하게 !armed이면 무조건
        // 모터를 정지값으로 강제하는 두 번째(독립적인) 방어선을 추가한다.
        if (!armed) {
            motorOut[0] = motorOut[1] = motorOut[2] = motorOut[3] = (uint16_t)dsMin;
        }
    }

#ifdef USE_DSHOT
    // ── ★ v8: RPM 필터 계수 갱신 (DShot 출력 직전) ──
    // motors.writeThrottle()이 내부적으로 이전 RX 처리 → TX → RX 시작을 수행함.
    // RPM 피드백은 TX 후에 오므로, 이번 루프의 RPM은 "이미 motors.getRPM()에 저장됨".
    // 따라서 writeThrottle() 직전에 RPM 필터 계수를 갱신하는 것이 올바른 순서임.
    //
    // 흐름:
    //   [루프 N]:   writeThrottle()내부에서 이전 RX 처리 → 최신 RPM 저장
    //               → rpmFilter.updateRPM() 으로 노치 계수 갱신
    //               → TX 프레임 전송 → RX 시작
    //   [루프 N+1]: 위 과정 반복 (약 1ms 지연으로 RPM 추종)
#ifdef USE_RPM_FILTER
    // writeThrottle()을 호출하면 내부에서 processRx()가 먼저 실행되어
    // erpm_[], rpmFilt_[] 가 갱신됨.
    // writeThrottle() 호출 전에는 아직 이전 값이 남아있으므로
    // writeThrottle() 이후에 RPM 필터 계수를 갱신해야 함.
    //
    // ★ 실제 순서:
    //   1) motors.writeThrottle() → 내부에서 processRx() 실행 → getRPM() 갱신
    //   2) rpmFilter.updateRPM() 호출 → 갱신된 RPM 사용
    //   3) 다음 readIMU()에서 갱신된 계수로 필터 적용
    motors.writeThrottle(motorOut);

    for (int m = 0; m < 4; m++) {
        if (motors.isRPMValid(m)) {
            rpmFilter.updateRPM(m, motors.getRPM(m));
        }
    }
#else
    motors.writeThrottle(motorOut);
#endif
    delayMicroseconds(500);
#endif // USE_DSHOT
}

// ============================================================================
//  doArm / doDisarm / handleFailsafe  (v7과 동일)
// ============================================================================
void doArm()
{
    if (armed) return;
    if (elrs.getThrottleUs() > (THROTTLE_ARM_MIN + 50.0f)) {
        Serial.println("[시동 거부] 스로틀을 최저 위치로 내리세요!");
        return;
    }
    // ★ 진단 마커: 재부팅이 doArm() 도중 발생한다면, 시리얼에서 마지막으로
    // 출력된 [ARM-n] 줄을 보고 정확히 어느 단계에서 멈췄는지 알 수 있다.
    Serial.println("[ARM-1] doArm 시작");
    pid.resetAll();
    Serial.println("[ARM-2] pid.resetAll 완료");    pid.resetYawITerm();  // ★ Yaw I항 초기화: 이전 와인드업 제거
    Serial.println("[ARM-3] resetYawITerm 완료");
#ifdef USE_SONAR
    sonar.resetAltHold();
    Serial.println("[ARM-4] sonar.resetAltHold 완료");
#endif
    armed     = true;
    armTimeMs = millis();
    failsafeLandThr = 0.0f;
    Serial.println("[ARM-5] armed=true 설정 완료");

    // GPS 홈 저장은 loop()의 GPS 섹션에서 처리
    // (Arduino IDE 프로토타입 생성기 버그 우회 — USE_GPS #ifdef 불필요)

    Serial.print("[시동] Armed!  배터리=");
    Serial.printf("%.2fV", battVoltage);
#ifdef DSHOT_BIDIR
    Serial.print("  [Bidir DShot ON]");
#endif
    Serial.println();
    Serial.println("       ★ 스로틀을 천천히 올리세요!");
    Serial.println("[ARM-6] 시동 메시지 출력 완료");
    buzzerMode = BZR_ARMED;   // ★ 시동 확인음 (비차단: updateBuzzer가 처리)
    buzzerTick = 0;
    Serial.println("[ARM-7] doArm 완전히 종료");
}

void doDisarm()
{
    if (!armed) return;
    armed = false;
    failsafeRTHStarted = false;
#ifdef USE_DSHOT
    motors.stopAll();
#endif
    pid.resetAll();
#ifdef USE_GPS
    gps.stopRTH();
#endif
    // ★ 해제 확인음 (비차단)
    buzzerMode = BZR_DISARMED;
    buzzerTick = 0;
    Serial.println("[해제] Disarmed.");
}

// ============================================================================
//  handleFailsafe — 신호 두절 시 안전 처리
//  ─────────────────────────────────────────────────────────────────────────
//  DroneConfig.h의 FAILSAFE_ACTION으로 동작 선택:
// ============================================================================
//  handleFailsafe — Failsafe 동작 선택
// ─────────────────────────────────────────────────────────────────────────
//  DroneConfig.h FAILSAFE_ACTION으로 선택:
//
//  FAILSAFE_CUT  : 항상 즉시 모터 정지
//                  (GPS/기압계 불필요)
//
//  FAILSAFE_LAND : 항상 FAILSAFE_LAND_US 스로틀로 자세 유지 착륙
//                  (GPS 불필요 — 기압계만으로 동작)
//
//  FAILSAFE_RTH  : GPS Fix + 홈 있음 → GPS 홈 복귀
//                  GPS Fix 없음       → LAND 폴백 (기압계로 착륙)
// ============================================================================
void handleFailsafe()
{
#if FAILSAFE_ACTION == FAILSAFE_CUT
    // ── CUT: 즉시 모터 정지 (GPS/기압계 불필요) ──
    Serial.println("[페일세이프] 모터 정지 (CUT)");
    doDisarm();

#elif FAILSAFE_ACTION == FAILSAFE_LAND
    // ── LAND: 기압계로 자세 유지 착륙 (GPS 불필요) ──
    {
        static bool landMsgPrinted = false;
        if (!landMsgPrinted) {
            Serial.printf("[페일세이프] LAND — 착륙 스로틀=%.0f µs\n", FAILSAFE_LAND_US);
            landMsgPrinted = true;
        }
        failsafeLandThr = FAILSAFE_LAND_US;
    }

#elif FAILSAFE_ACTION == FAILSAFE_RTH
    // ── RTH: GPS Fix 있으면 홈 복귀, 없으면 LAND 폴백 ──
#ifdef USE_GPS
    if (gps.hasHome() && gps.hasValidFix()) {
        if (!gps.isRTHActive()) {
            Serial.println("[페일세이프] GPS RTH 시작!");
            failsafeRTHStarted = true;
            gps.startRTH();
        }
        return;  // RTH 활성 → 메인루프 computeRTH()가 처리
    }
#endif
    // GPS Fix 없음 → LAND 폴백 (기압계로 안전 착륙)
    {
        static bool rthFallbackPrinted = false;
        if (!rthFallbackPrinted) {
            Serial.printf("[페일세이프] GPS Fix 없음 → LAND 폴백 (%.0f µs)\n",
                          FAILSAFE_LAND_US);
            rthFallbackPrinted = true;
        }
        failsafeLandThr = FAILSAFE_LAND_US;
    }
#endif
}

// ============================================================================
//  sendTelemetry — FC 텔레메트리 전송 (20Hz 호출)
//  ─────────────────────────────────────────────────────────────────────────
//  ★ 전송 순서 (Betaflight 방식):
//    1) Flight Mode (0x21) — ELRS가 FC 텔레메트리 스트림 인식에 필수
//    2) Attitude   (0x1E) — Pitch / Roll / Yaw
//    3) Battery    (0x08) — 전압 / 전류 (100ms마다)
//
//  ★ 텔레메트리가 조종기에 표시되지 않을 때 체크리스트:
//    [1] ELRS Lua → Other Devices → FC → Telemetry Ratio ≠ Off
//        (RQly 등 ELRS 내부 센서는 Ratio=Off에도 동작 → 별개 경로)
//    [2] FC GPIO17(TX) → ELRS RX CRSF RX 핀 연결 확인
//    [3] EdgeTX: Model → Telemetry → Delete all → Discover new sensors
// ============================================================================
void updateBuzzer()
{
    uint32_t now = millis();

    // ── 우선순위 1: CH10 추락 탐색 비퍼 ──
    // ★ Disarmed 상태에서만 동작 (추락 후 해제 상태에서 위치 탐색)
    // Betaflight 동일: 비행 중(Armed)에는 비퍼 OFF, 추락·해제 후 ON
    bool searchOn = (elrs.getChannelUs(CH_BUZZER) > 1500) && !armed;
    if (searchOn) {
        if (now - buzzerMs >= 300) {
            buzzerPhase = !buzzerPhase;
            digitalWrite(PIN_BUZZER, buzzerPhase ? HIGH : LOW);
            buzzerMs = now;
        }
        return;
    }

    // ── 우선순위 2: 배터리 경고 ──
    if (armed && battVoltage > 1.0f && battVoltage < BATT_WARN_VOLTAGE) {
        if (now - buzzerMs >= 200) {
            buzzerPhase = !buzzerPhase;
            digitalWrite(PIN_BUZZER, buzzerPhase ? HIGH : LOW);
            buzzerMs = now;
        }
        return;
    }

    // ── 우선순위 3: Failsafe 경고 ──
    if (failsafeActive) {
        if (now - buzzerMs >= 100) {
            buzzerPhase = !buzzerPhase;
            digitalWrite(PIN_BUZZER, buzzerPhase ? HIGH : LOW);
            buzzerMs = now;
        }
        return;
    }

    // ── 우선순위 4-5: 시동/해제 확인음 ──
    switch (buzzerMode) {
    case BZR_ARMED:
        // 삐삐 2회: 80ms ON-OFF 4틱
        if (buzzerTick < 4) {
            if (now - buzzerMs >= 80) {
                buzzerPhase = !buzzerPhase;
                digitalWrite(PIN_BUZZER, buzzerPhase ? HIGH : LOW);
                buzzerMs = now;
                buzzerTick++;
            }
        } else {
            digitalWrite(PIN_BUZZER, LOW);
            buzzerMode = BZR_OFF;
        }
        break;
    case BZR_DISARMED:
        // 삐- 1회: 200ms ON
        if (buzzerTick == 0) {
            digitalWrite(PIN_BUZZER, HIGH);
            buzzerMs = now; buzzerTick = 1;
        } else if (now - buzzerMs >= 200) {
            digitalWrite(PIN_BUZZER, LOW);
            buzzerMode = BZR_OFF;
        }
        break;
    default:
    case BZR_OFF:
        digitalWrite(PIN_BUZZER, LOW);
        break;
    }
}

void sendTelemetry()
{
#ifdef USE_ELRS
    // ── Flight Mode: ELRS 텔레메트리 스트림 인식 트리거 ──
    const char *modeStr = "ANGLE";
    if      (flightMode == FLIGHT_MODE_ACRO)    modeStr = "ACRO";
    else if (flightMode == FLIGHT_MODE_HORIZON) modeStr = "HORIZON";
    else if (flightMode == FLIGHT_MODE_ALTHOLD) modeStr = "ALTHOLD";
    if (failsafeActive)                         modeStr = "FAILSAFE";

#ifdef USE_GPS
    // ★ 비시동 + GPS Fix 상태: FM 필드에 십진도 좌표를 2초마다 교번 표시
    // 기체 분실 시 조종기에서 좌표를 바로 읽어 구글맵 검색 가능
    // 예: "N37.50833" ↔ "E126.86877"
    static uint32_t gpsDispToggleMs = 0;
    static bool     gpsDispShowLat  = true;
    static char     gpsFmBuf[16];
    if (!armed && gps.hasValidFix()) {
        if (millis() - gpsDispToggleMs >= 2000) {
            gpsDispToggleMs = millis();
            gpsDispShowLat  = !gpsDispShowLat;
        }
        if (gpsDispShowLat)
            snprintf(gpsFmBuf, sizeof(gpsFmBuf), "N%.5f", gps.getLat());
        else
            snprintf(gpsFmBuf, sizeof(gpsFmBuf), "E%.5f", gps.getLon());
        elrs.sendFlightModeTelemetry(gpsFmBuf);
    } else {
        elrs.sendFlightModeTelemetry(modeStr);
    }
#else
    elrs.sendFlightModeTelemetry(modeStr);
#endif

    // ── Attitude: Pitch / Roll / Yaw ──
    elrs.sendAttitudeTelemetry(roll, pitch, yaw);

    // ── Battery: 100ms마다 ──
    static uint32_t batTelemMs = 0;
    if (millis() - batTelemMs >= 100) {
        batTelemMs = millis();
        static uint32_t mAhUsed = 0;
        elrs.sendBatteryTelemetry(battVoltage, 0.0f, mAhUsed);
    }

#ifdef USE_GPS
    if (gps.hasValidFix()) {
        uint8_t gpsPay[15];
        uint8_t gpsLen = 0;
        gps.fillCRSF_GPS(gpsPay, gpsLen);
        elrs.sendGPSTelemetry(gpsPay, gpsLen);
    }
#endif
#endif
}

// ============================================================================
//  printRPMDebug — ★ v8 신규: RPM + 수신 성공률 시리얼 출력 (2초마다)
//  ─────────────────────────────────────────────────────────────────────────
//  시리얼 모니터에서 Bidir DShot 동작 상태를 확인합니다.
//  수신 성공률이 낮으면 (< 80%) ESC 설정 또는 풀업 저항을 확인하세요.
// ============================================================================
#ifdef DSHOT_BIDIR
void printRPMDebug()
{
    Serial.print("[RPM] ");
    for (int m = 0; m < 4; m++) {
        float rpm  = motors.getRPM(m);
        bool  valid= motors.isRPMValid(m);
        float rate = motors.getRPMSuccessRate(m) * 100.0f;
        Serial.printf("M%d=%5.0f%s(%.0f%%) ",
                      m+1, rpm, valid ? "" : "!", rate);
    }
    Serial.println();

#ifdef USE_RPM_FILTER
    // 필터 상태도 같이 출력 (5초마다)
    static uint32_t filterDebugTimer = 0;
    if (millis() - filterDebugTimer >= 5000) {
        filterDebugTimer = millis();
        rpmFilter.debugPrint(0);  // period_ms=0 → 강제 출력
    }
#endif
}
#endif

// ============================================================================
//  readBatteryVoltage / ledBlink  (v7과 동일)
// ============================================================================
float readBatteryVoltage()
{
    // ★ 다중 샘플 평균 (8회): pinMode 버그 수정으로 근본 원인은 제거했지만,
    //   배선이 ELRS/I2C 등 고속 디지털 신호선과 물리적으로 가까이 지나간다면
    //   여전히 약간의 용량성 결합(crosstalk) 노이즈가 섞일 수 있다. 8회
    //   평균은 이런 잔여 노이즈를 추가로 완화하면서도, 1kHz 루프에서
    //   호출 비용이 무시할 만한 수준(수십 마이크로초)으로 유지된다.
    uint32_t sum = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) sum += analogRead(PIN_BATTERY_ADC);
    float adcRaw = (float)sum / N;
    float adcV   = adcRaw / 4095.0f * 3.3f;
    return adcV * BATTERY_DIVIDER;
}

void ledBlink(uint32_t ms)
{
    static uint32_t lastT = 0;
    if (millis() - lastT > ms) {
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
        lastT = millis();
    }
}

// ============================================================================
//  ★ v8: PID 실시간 튜너 함수들
// ============================================================================
#ifdef USE_PID_TUNER

// ── applyTunerGains — 튜너 변수 → PID 객체에 실제 적용 ──
void applyTunerGains()
{
    // Pitch 축
    pid.setOuterGains(AXIS_PITCH, tg.opi, tg.oii, tg.odi);
    pid.setInnerGains(AXIS_PITCH, tg.ipi, tg.iii, tg.idi);
    // Roll 축 (Pitch와 동일한 게인 사용)
    pid.setOuterGains(AXIS_ROLL,  tg.opi, tg.oii, tg.odi);
    pid.setInnerGains(AXIS_ROLL,  tg.ipi, tg.iii, tg.idi);
    // Yaw 축
    pid.setInnerGains(AXIS_YAW,   tg.ipy, tg.iiy, 0.0f);
    pid.setOuterGains(AXIS_YAW,   OUTER_KP_YAW, 0.0f, 0.0f);
}

// ── processTunerInput — 시리얼 명령 파싱 + 즉시 적용 ──
// 명령 형식: "명령키 값\n"  예) "ipi 4.0\n"
void processTunerInput()
{
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toLowerCase();
    if (line.length() == 0) return;

    // ── "plot on" / "plot off" ──
    if (line == "plot on")  { plotMode_ = true;  Serial.println("[튜너] 플로터 모드 ON (plot off 로 해제)"); return; }
    if (line == "plot off") { plotMode_ = false; Serial.println("[튜너] 플로터 모드 OFF"); return; }

    // ── "show" : 현재 게인 출력 ──
    if (line == "show") { printTunerGains(); return; }

    // ── "reset" : PID I항 리셋 ──
    if (line == "reset") { pid.resetAll(); Serial.println("[튜너] PID 상태 초기화 (I항 리셋)"); return; }

    // ── ★ "forcearm" / "forcedisarm" : TX 스위치를 거치지 않고 직접 아밍 ──
    // 진단 목적: 아밍 시 재부팅이 TX/ELRS 채널 전환과 관련된 것인지,
    // 아니면 doArm() 내부 로직(PID리셋·부저·GPS체크 등) 자체의 문제인지
    // 구분하기 위함. 반드시 프롭을 제거한 상태에서만 사용할 것.
    if (line == "forcearm") {
        Serial.println("[진단] ★ TX 스위치 우회 — 강제 아밍 시도 (프롭 제거 확인!)");
        doArm();
        return;
    }
    if (line == "forcedisarm") {
        Serial.println("[진단] TX 스위치 우회 — 강제 해제");
        doDisarm();
        return;
    }

    // ── 게인 변경 명령: "키 값" 형식 ──
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println("[튜너] 형식 오류. 예: 'ipi 4.0'"); return; }

    String key = line.substring(0, sp);
    float  val = line.substring(sp + 1).toFloat();

    bool changed = true;
    if      (key == "opi") tg.opi = val;
    else if (key == "oii") tg.oii = val;
    else if (key == "odi") tg.odi = val;
    else if (key == "ipi") tg.ipi = val;
    else if (key == "iii") tg.iii = val;
    else if (key == "idi") tg.idi = val;
    else if (key == "ipy") tg.ipy = val;
    else if (key == "iiy") tg.iiy = val;
    else { Serial.printf("[튜너] 알 수 없는 명령: '%s'\n", key.c_str()); changed = false; }

    if (changed) {
        applyTunerGains();
        Serial.printf("[튜너] %-4s = %.4f  (즉시 적용)\n", key.c_str(), val);
        // 변경 후 전체 게인 요약 출력
        Serial.printf("  Outer: KP=%.2f KI=%.3f KD=%.3f\n", tg.opi, tg.oii, tg.odi);
        Serial.printf("  Inner: KP=%.2f KI=%.3f KD=%.3f\n", tg.ipi, tg.iii, tg.idi);
    }
}

// ── printTunerGains — 현재 게인 전체 출력 ──
void printTunerGains()
{
    Serial.println("[PID 튜너] ── 현재 게인 ────────────────────────");
    Serial.printf("  Pitch/Roll Outer : KP=%.2f  KI=%.3f  KD=%.3f\n", tg.opi, tg.oii, tg.odi);
    Serial.printf("  Pitch/Roll Inner : KP=%.2f  KI=%.3f  KD=%.3f\n", tg.ipi, tg.iii, tg.idi);
    Serial.printf("  Yaw Inner        : KP=%.2f  KI=%.3f\n",           tg.ipy, tg.iiy);
    Serial.println("[PID 튜너] ── 명령어 ────────────────────────────");
    Serial.println("  opi/oii/odi [값]  → Outer KP/KI/KD");
    Serial.println("  ipi/iii/idi [값]  → Inner KP/KI/KD");
    Serial.println("  ipy/iiy [값]      → Yaw Inner KP/KI");
    Serial.println("  show / reset / plot on / plot off");
    Serial.println("──────────────────────────────────────────────────");
}

// ── printPlotterData — Arduino IDE Serial Plotter 호환 출력 ──
// 형식: "레이블:값 레이블:값 ..."  → 각 값이 별도 선으로 표시됨
//
// ★ Rod 테스트 시 확인 포인트:
//   TgtP  : 스틱 입력 목표 각도 [deg]
//   Pitch : 실제 Pitch 각도 [deg]  → TgtP와 얼마나 빠르게 일치하는가
//   PRate : Pitch 각속도 [deg/s]   → 진동이 있으면 여기서 고주파 발생
//   OOut  : Outer 루프 출력 (목표 각속도)
//   IOut  : Inner 루프 출력 (모터 출력 변화량)
void printPlotterData()
{
    // 스틱 목표 각도 계산
    float tgtPitch = 0.0f;
#ifdef USE_ELRS
    if (!elrs.isFailsafe()) {
        tgtPitch = fmap(elrs.getPitchUs(), 1000.f, 2000.f,
                        -MAX_ANGLE_PITCH_DEG, MAX_ANGLE_PITCH_DEG);
    }
#endif

    Serial.printf("TgtP:%.1f Pitch:%.1f PRate:%.1f OOut:%.1f IOut:%.1f M1:%d M2:%d M3:%d M4:%d\n",
                  tgtPitch,
                  pitch,
                  gyroY,
                  pid.getOuterOutput(AXIS_PITCH),
                  pid.getInnerOutput(AXIS_PITCH),
                  (int)motorOut[0],
                  (int)motorOut[1],
                  (int)motorOut[2],
                  (int)motorOut[3]);
}

#endif // USE_PID_TUNER
