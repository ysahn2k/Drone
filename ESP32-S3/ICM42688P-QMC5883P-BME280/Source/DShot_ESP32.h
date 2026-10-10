// ============================================================================
//  DShot_ESP32.h  —  DShot300 모터 제어 드라이버  (v8)
//
//  ★ v8 변경사항 (v7 대비)
//  ─────────────────────────────────────────────────────────────────────────
//  [1] Bidir DShot 지원 (#define DSHOT_BIDIR 활성화 시)
//      - TX 신호 반전 (invert_out=1): ESC가 정상 DShot으로 수신
//      - TX 완료 후 GPIO INPUT 전환 → RMT RX로 ESC 응답 수신
//      - GCR(4b5b) 디코딩 → eRPM 계산
//      - getRPM(motor): 기계적 RPM 반환
//
//  [2] GCR 디코딩 파이프라인
//      RMT 심볼 → 20비트 스트림 재구성 → NRZI 언폴딩 → 4b5b 니블 디코딩
//      → 12비트 기간 + 4비트 CRC → eRPM → 기계적 RPM
//
//  ★ Betaflight 표준 모터 배치 (v7과 동일)
//  ─────────────────────────────────────────────────────────────────────────
//          FRONT (기수)
//   M4(FL,CW)    M2(FR,CCW)
//        \           /
//        /           \
//   M3(RL,CCW)   M1(RR,CW)
//          REAR
//
//  M1 = Rear  Right (뒤-오른쪽, CW)   → GPIO4
//  M2 = Front Right (앞-오른쪽, CCW)  → GPIO5
//  M3 = Rear  Left  (뒤-왼쪽,  CCW)  → GPIO6
//  M4 = Front Left  (앞-왼쪽,  CW)   → GPIO7
//
//  ★ v7에서 유지된 수정사항
//  ─────────────────────────────────────────────────────────────────────────
//  [1] RMT 해상도: 40MHz
//  [2] CRC: XOR & 0xF (NOT 없음)
//  [3] Reset Gap: 17번째 심볼 LOW 5µs
//  [4] delayMicroseconds(1000) 사용
//  [5] buildFrame()에 telem 파라미터
// ============================================================================
#pragma once
#ifndef DSHOT_ESP32_H
#define DSHOT_ESP32_H

#include <Arduino.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "DroneConfig.h"

// ── DShot 특수 명령 ──
#define DSHOT_CMD_STOP           0
#define DSHOT_CMD_BEEP1          1
#define DSHOT_CMD_BEEP2          2
#define DSHOT_CMD_BEEP3          3
#define DSHOT_CMD_BEEP4          4
#define DSHOT_CMD_BEEP5          5
#define DSHOT_CMD_SPIN_NORMAL    7
#define DSHOT_CMD_SPIN_REVERSE   8
#define DSHOT_CMD_SAVE_SETTINGS 12

// DSHOT_NUM_MOTORS는 DroneConfig.h [14]에 정의됨

// ── RMT 타이밍 (40MHz 기준) ──
// 비트 주기 = 40MHz / 300kHz = 133틱 ≈ 3325ns
#define DS_RMT_RESOLUTION_HZ  40000000UL  // 40MHz (1틱 = 25ns)
#define DS_TICKS_PER_BIT      133          // 133틱 = 3325ns
#define DS_T1H                 99           // '1' HIGH: 99틱 = 2475ns (75%)
#define DS_T1L                 34           // '1' LOW:  34틱 =  850ns
#define DS_T0H                 49           // '0' HIGH: 49틱 = 1225ns (37%)
#define DS_T0L                 84           // '0' LOW:  84틱 = 2100ns
#define DS_RESET_TICKS        200           // Reset Gap: 200틱 = 5µs

// 프레임 크기: 16비트 데이터 + 1 Reset Gap
#define DS_FRAME_SYMBOLS      17

// ESP32-S3 RMT 메모리: 전체 384 심볼, 8채널(TX4+RX4) 공유
// TX 4채널 × 48 = 192, RX 4채널 × 48 = 192, 합계 384 (딱 맞음)
#define DS_MEM_BLOCK_SYMBOLS  48

// ── GCR 디코딩 상수 ──
// GCR 5비트 → 4비트 니블 매핑 (인덱스 = 5비트 코드, 값 = 4비트 니블)
// 0xFF = 유효하지 않은 코드
#define GCR_INVALID  0xFF

// GCR 응답 값 특수 케이스
#define GCR_PERIOD_STOPPED   0x0000  // 모터 정지
#define GCR_PERIOD_MAX       0x0FFF  // 최대 기간값 (최저속도)
#define GCR_PERIOD_INVALID   0x0FFF  // 유효하지 않은 마커

// eRPM → 기계적 RPM 변환 (DroneConfig의 MOTOR_POLES 사용)
// RPM = eRPM / pole_pairs = eRPM / (MOTOR_POLES / 2) = eRPM * 2 / MOTOR_POLES
#define ERPM_TO_RPM(erpm)  ((float)(erpm) * 2.0f / (float)MOTOR_POLES)

// ============================================================================
//  DShot_ESP32 클래스
// ============================================================================
class DShot_ESP32 {
public:
    DShot_ESP32();
    ~DShot_ESP32();

    // ── 초기화 ──
    // pins[4] = {M1핀, M2핀, M3핀, M4핀}
    // DSHOT_BIDIR 활성 시: TX 채널(인버티드) + RX 채널을 같은 핀에 구성
    bool begin(const int pins[4]);

    // ── 4모터 동시 전송 ──
    // throttle[4]: 0=정지, 48~2047=속도
    // DSHOT_BIDIR 활성 시: 이전 RX 결과 처리 → TX → GPIO INPUT 전환 → RX 시작
    void writeThrottle(const uint16_t throttle[4]);

    // ── 단일 모터 전송 ──
    void writeMotor(uint8_t motor, uint16_t throttle);

    // ── ESC Arming ──
    void arm(uint32_t arm_ms = 3000);

    // ── 전체 정지 ──
    void stopAll();

    // ── 특수 명령 전송 ──
    void sendCommand(uint8_t motor, uint8_t cmd, uint8_t repeats = 10);



    // ── 단위 변환 ──
    static uint16_t usToDShot  (float us);
    static uint16_t normToDShot(float norm);

#ifdef DSHOT_BIDIR
    // ── Bidir DShot: RPM 조회 ──
    // motor: 0~3 (M1~M4)
    // 반환: 기계적 RPM (LPF 적용). 유효하지 않으면 0.
    float    getRPM (uint8_t motor) const;

    // eRPM 조회 (전기적 RPM = 기계적 RPM × 극쌍수)
    uint32_t getERPM(uint8_t motor) const;

    // 최신 수신 성공 여부 (디버깅용)
    bool     isRPMValid(uint8_t motor) const;

    // RPM 수신 성공률 [0.0~1.0] (디버깅용, 최근 32프레임 기준)
    float    getRPMSuccessRate(uint8_t motor) const;
#endif

    // ── 진단: TX 타임아웃 발생 횟수 (Betaflight TX_DONE 가드 적용 후 모니터링용) ──
    // 0이 유지되어야 정상. 증가한다면 RMT 하드웨어 이상 의심.
    uint32_t getTxTimeoutCount() const { return txTimeoutCount_; }

private:
    // ── TX 채널 ──
    rmt_channel_handle_t  txChan_  [DSHOT_NUM_MOTORS];
    rmt_encoder_handle_t  encoder_ [DSHOT_NUM_MOTORS];
    rmt_symbol_word_t     symbols_ [DSHOT_NUM_MOTORS][DS_FRAME_SYMBOLS];
    int                   pins_    [DSHOT_NUM_MOTORS];  // GPIO 핀 저장
    bool                  ready_;
    uint32_t              txTimeoutCount_ = 0;   // ★ TX_DONE 타임아웃 누적 횟수

    void    buildFrame(rmt_symbol_word_t *sym, uint16_t throttle,
                       bool telem = false);
    void    txStart   (uint8_t motor);
    void    txWait    (uint8_t motor);
    static uint8_t calcCRC(uint16_t packet12);

#ifdef DSHOT_BIDIR
    // ── RX 채널 (Bidir 전용) ──
    rmt_channel_handle_t  rxChan_  [DSHOT_NUM_MOTORS];
    rmt_symbol_word_t     rxBuf_   [DSHOT_NUM_MOTORS][DS_RX_BUF_SYMBOLS];

    // ISR에서 설정하는 플래그 (volatile 필수)
    volatile bool         rxDone_  [DSHOT_NUM_MOTORS];
    volatile size_t       rxSymCnt_[DSHOT_NUM_MOTORS];

    // RPM 데이터
    uint32_t              erpm_    [DSHOT_NUM_MOTORS];  // 최신 eRPM
    float                 rpmFilt_ [DSHOT_NUM_MOTORS];  // LPF 적용 기계적 RPM
    bool                  rpmValid_[DSHOT_NUM_MOTORS];  // 최신 수신 유효 여부

    // 통계 (성공률 계산용)
    uint32_t              rxTotalFrames_  [DSHOT_NUM_MOTORS];
    uint32_t              rxGoodFrames_   [DSHOT_NUM_MOTORS];

    // GCR 4b5b 디코딩 테이블 [5비트 인덱스 → 4비트 니블]
    // 0xFF = 유효하지 않은 코드
    static const uint8_t GCR_DECODE_TABLE[32];

    // ── Bidir 내부 함수 ──

    // RX 채널 초기화 (begin()에서 호출)
    bool initRxChannels();

    // RX 시작 (writeThrottle() 내부에서 TX 완료 후 호출)
    void startRx(uint8_t motor);

    // 이전 RX 결과 처리 (writeThrottle() 진입 시 호출)
    void processRx(uint8_t motor);

    // RMT 심볼 → 20비트 NRZ 스트림 재구성
    // 반환: true=성공(bits20에 결과), false=심볼 부족 또는 타이밍 오류
    static bool symsToBits(const rmt_symbol_word_t* syms, size_t numSyms,
                            uint32_t& bits20);

    // GCR 4b5b 디코딩: 20비트 → 16비트 데이터
    // bits20: NRZI 언폴딩 후의 20비트 GCR 스트림
    // value16: 출력 16비트 (상위12=기간, 하위4=CRC)
    // 반환: true=CRC 검증 통과, false=오류
    static bool decodeGCR(uint32_t bits20, uint16_t& value16);

    // 16비트 값에서 eRPM 계산
    // value: decodeGCR 출력 (상위 12비트=기간)
    // 반환: eRPM (0=정지 또는 유효하지 않음)
    static uint32_t erpmFromValue(uint16_t value);

    // RX 완료 ISR (정적 콜백)
    static bool IRAM_ATTR rxDoneISR(rmt_channel_handle_t ch,
                                     const rmt_rx_done_event_data_t* ev,
                                     void* userCtx);
#endif // DSHOT_BIDIR
};

#endif // DSHOT_ESP32_H
