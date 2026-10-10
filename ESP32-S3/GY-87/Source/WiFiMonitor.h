// ============================================================================
//  WiFiMonitor.h  —  QuadDrone ESP32-S3 WiFi PID 모니터링 대시보드
//
//  기능:
//    - ESP32-S3 내장 WiFi로 Access Point 생성 (SSID: QuadDrone-v8)
//    - 브라우저에서 192.168.4.1 접속 → 실시간 PID 모니터링 + 조정
//    - FreeRTOS Core 0에서 동작 → 비행 제어 루프(Core 1)에 영향 없음
//
//  사용법:
//    1. DroneConfig.h에서 USE_WIFI_MONITOR 주석 해제
//    2. 빌드 후 업로드
//    3. PC/스마트폰에서 WiFi "QuadDrone-v8" (비밀번호 없음) 접속
//    4. 브라우저에서 http://192.168.4.1 접속
// ============================================================================
#pragma once
#include <stdint.h>   // uint16_t 등 정수 타입
#ifdef USE_WIFI_MONITOR

// ── 공유 데이터 구조체 (Core 1 → Core 0 방향) ─────────────────────────────
struct WifiMonSnapshot {
    float  roll, pitch, yaw;       // 자세각 [°]
    float  gx, gy, gz;             // 자이로 [°/s]
    uint16_t m1, m2, m3, m4;       // 모터 출력 [100~1000 DShot]
    bool   armed;
    float  battV;                  // 배터리 전압 [V]
    float  minBattV;                // ★ 부팅 이후 관측된 최저 전압 (순간 강하 포착용)
    uint32_t uptimeMs;              // ★ 부팅 후 경과시간 [ms] — 재부팅/리셋 감지용
    char     resetReason[40];       // ★ 마지막 부팅의 재시작 원인 (esp_reset_reason)
    // PID 파라미터 (현재 적용 중인 값)
    float  outerKP, outerKI, outerKD;   // ★ Outer 3개
    float  innerKP, innerKI, innerKD;
    float  yawKP,   yawKI;
};

// ── PID 명령 구조체 (Core 0 → Core 1 방향) ────────────────────────────────
struct WifiPidCmd {
    volatile bool pending;
    float  outerKP, outerKI, outerKD;   // ★ Outer 3개
    float  innerKP, innerKI, innerKD;
    float  yawKP,   yawKI;
};

extern WifiMonSnapshot g_wifiSnap;   // Core 1이 매 루프 업데이트
extern WifiPidCmd      g_wifiPidCmd; // WiFi가 설정 → Core 1이 적용

// ── ★ Betaflight 분석 적용: seqlock 패턴 (acceleration.c accelSeq 참고) ──
// 뮤텍스 없이 Core1(쓰기)↔Core0(읽기) 간 구조체 공유 시 "torn read"
// (일부 필드만 갱신된 도중에 읽는 현상)를 방지한다.
// 짝수: 안정 상태 / 홀수: 쓰기 진행 중 → 읽는 쪽은 짝수일 때만 신뢰.
extern volatile uint32_t g_wifiSnapSeq;

// Core1에서 호출: 쓰기 시작 전후로 반드시 짝으로 호출
inline void wifiSnapWriteBegin() { g_wifiSnapSeq++; }   // 짝수→홀수
inline void wifiSnapWriteEnd()   { g_wifiSnapSeq++; }   // 홀수→짝수

// Core0에서 호출: 일관된 스냅샷을 얻을 때까지 재시도 (최대 4회)
inline WifiMonSnapshot wifiSnapReadSafe() {
    WifiMonSnapshot copy;
    for (int attempt = 0; attempt < 4; attempt++) {
        uint32_t seq1 = g_wifiSnapSeq;
        if (seq1 & 1) continue;          // 쓰기 도중 → 재시도
        copy = g_wifiSnap;                // 구조체 복사
        uint32_t seq2 = g_wifiSnapSeq;
        if (seq1 == seq2) return copy;    // 복사 중 변경 없었음 → 신뢰 가능
    }
    return copy;   // 4회 재시도해도 불안정하면 마지막 값이라도 반환 (최선)
}

// ── API ───────────────────────────────────────────────────────────────────
void initWiFiMonitor();              // setup()에서 1회 호출

#endif // USE_WIFI_MONITOR
