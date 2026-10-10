// ============================================================================
//  DShot_ESP32.cpp  —  DShot300 + Bidir DShot 구현부  (v8)
//
//  ★ v8 Bidir 구현 핵심
//  ─────────────────────────────────────────────────────────────────────────
//  [TX] invert_out=1: FC가 인버티드 DShot 전송 → ESC는 정상 DShot으로 수신
//  [RX] TX 완료 후 GPIO → INPUT 전환 → RMT RX로 ESC GCR 응답 수신
//  [GCR] 20비트 스트림 → NRZI 언폴딩 → 4b5b 니블 디코딩 → eRPM
//
//  ★ GCR 4b5b 인코딩 (Betaflight 표준)
//  ─────────────────────────────────────────────────────────────────────────
//  ESC가 보내는 20비트 = 4개의 5비트 코드 (MSB 먼저)
//  각 5비트 코드 → 4비트 니블로 디코딩
//  최종 16비트: [15:4]=12비트 eRPM 기간, [3:0]=4비트 XOR CRC
//
//  eRPM 기간 단위: 1.25µs (5/4 µs)
//  eRPM 계산: eRPM = 60,000,000 / (기간 × 1.25µs)
//
//  ★ NRZI 언폴딩
//  ─────────────────────────────────────────────────────────────────────────
//  ESC 응답은 NRZI 인코딩: transmitted[i] = data[i] XOR transmitted[i-1]
//  디코딩: data[i] = transmitted[i] XOR transmitted[i-1]
//  코드:    bits20 = raw ^ (raw >> 1)
//
//  ★ 타이밍 참고 (DShot300)
//  ─────────────────────────────────────────────────────────────────────────
//  TX 프레임: 17심볼 × 3.33µs ≈ 56.5µs
//  ESC 처리 지연: ~26µs
//  GCR 응답: 20비트 × 3.33µs ≈ 66.5µs
//  합계: ~149µs → 1kHz(1000µs) 루프에서 충분한 여유
// ============================================================================
#include "DShot_ESP32.h"
#include <string.h>
#include "driver/gpio.h"

// ============================================================================
//  GCR 4b5b 디코딩 테이블 (정적 멤버 초기화)
//  인덱스 = 5비트 코드(0~31), 값 = 4비트 니블 (0xFF=유효하지 않음)
//
//  인코딩 매핑 (Betaflight 표준):
//    4비트 0x0 → 5비트 0x19(11001)    0x1 → 0x1B(11011)
//    0x2 → 0x12(10010)                0x3 → 0x13(10011)
//    0x4 → 0x1D(11101)                0x5 → 0x15(10101)
//    0x6 → 0x16(10110)                0x7 → 0x17(10111)
//    0x8 → 0x1A(11010)                0x9 → 0x09(01001)
//    0xA → 0x0A(01010)                0xB → 0x0B(01011)
//    0xC → 0x1E(11110)                0xD → 0x0D(01101)
//    0xE → 0x0E(01110)                0xF → 0x0F(01111)
// ============================================================================
#ifdef DSHOT_BIDIR
const uint8_t DShot_ESP32::GCR_DECODE_TABLE[32] = {
    // 인덱스: 0     1     2     3     4     5     6     7
    GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID,
    GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID,
    // 인덱스: 8     9     10    11    12    13    14    15
    GCR_INVALID, 0x09, 0x0A, 0x0B, GCR_INVALID, 0x0D, 0x0E, 0x0F,
    // 인덱스: 16    17    18    19    20    21    22    23
    GCR_INVALID, GCR_INVALID, 0x02, 0x03, GCR_INVALID, 0x05, 0x06, 0x07,
    // 인덱스: 24    25    26    27    28    29    30    31
    GCR_INVALID, 0x00, 0x08, 0x01, GCR_INVALID, 0x04, 0x0C, GCR_INVALID
};
#endif

// ============================================================================
//  생성자 / 소멸자
// ============================================================================
DShot_ESP32::DShot_ESP32() : ready_(false)
{
    memset(txChan_,  0, sizeof(txChan_));
    memset(encoder_, 0, sizeof(encoder_));
    memset(symbols_, 0, sizeof(symbols_));
    memset(pins_,    0, sizeof(pins_));

#ifdef DSHOT_BIDIR
    memset(rxChan_,   0, sizeof(rxChan_));
    memset(rxBuf_,    0, sizeof(rxBuf_));
    memset((void*)rxDone_,   0, sizeof(rxDone_));    // volatile → void* 캐스팅
    memset((void*)rxSymCnt_, 0, sizeof(rxSymCnt_));  // volatile → void* 캐스팅
    memset(erpm_,     0, sizeof(erpm_));
    memset(rpmFilt_,  0, sizeof(rpmFilt_));
    memset(rpmValid_, 0, sizeof(rpmValid_));
    memset(rxTotalFrames_, 0, sizeof(rxTotalFrames_));
    memset(rxGoodFrames_,  0, sizeof(rxGoodFrames_));
#endif
}

DShot_ESP32::~DShot_ESP32()
{
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        if (encoder_[m]) { rmt_del_encoder(encoder_[m]); encoder_[m] = nullptr; }
        if (txChan_[m])  { rmt_disable(txChan_[m]); rmt_del_channel(txChan_[m]); txChan_[m] = nullptr; }
#ifdef DSHOT_BIDIR
        if (rxChan_[m])  { rmt_disable(rxChan_[m]); rmt_del_channel(rxChan_[m]); rxChan_[m] = nullptr; }
#endif
    }
}

// ============================================================================
//  begin — RMT TX (+ Bidir: RX) 초기화
// ============================================================================
bool DShot_ESP32::begin(const int pins[4])
{
#ifdef DSHOT_BIDIR
    Serial.println("[DShot] RMT 초기화 (40MHz, Bidir ON, mem=48×8채널)");
#else
    Serial.println("[DShot] RMT 초기화 (40MHz, 단방향, mem=48)");
#endif

    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        pins_[m] = pins[m];  // GPIO 핀 저장 (Bidir GPIO 전환에 필요)

        // ── TX 채널 설정 ──
        rmt_tx_channel_config_t chCfg = {};
        chCfg.gpio_num          = (gpio_num_t)pins[m];
        chCfg.clk_src           = RMT_CLK_SRC_DEFAULT;
        chCfg.resolution_hz     = DS_RMT_RESOLUTION_HZ;   // 40MHz
        chCfg.mem_block_symbols = DS_MEM_BLOCK_SYMBOLS;   // 48
        chCfg.trans_queue_depth = 4;
#ifdef DSHOT_BIDIR
        // Bidir: TX 신호 반전 필수
        // 인버티드 DShot → ESC는 정상 DShot으로 수신
        // 유휴 상태 = HIGH (인버티드), ESC가 응답 후에도 HIGH 유지
        chCfg.flags.invert_out  = 1;
#else
        chCfg.flags.invert_out  = 0;
#endif
        chCfg.flags.with_dma    = 0;

        esp_err_t err = rmt_new_tx_channel(&chCfg, &txChan_[m]);
        if (err != ESP_OK) {
            Serial.printf("[DShot 오류] 모터%d TX 채널 실패: %s\n", m+1, esp_err_to_name(err));
            return false;
        }

        // ── 인코더 (Copy 방식) ──
        rmt_copy_encoder_config_t encCfg = {};
        err = rmt_new_copy_encoder(&encCfg, &encoder_[m]);
        if (err != ESP_OK) {
            Serial.printf("[DShot 오류] 모터%d 인코더 실패: %s\n", m+1, esp_err_to_name(err));
            return false;
        }

        err = rmt_enable(txChan_[m]);
        if (err != ESP_OK) {
            Serial.printf("[DShot 오류] 모터%d TX 활성화 실패: %s\n", m+1, esp_err_to_name(err));
            return false;
        }
        Serial.printf("[DShot] 모터%d TX (GPIO%d) OK\n", m+1, pins[m]);
    }

#ifdef DSHOT_BIDIR
    // ── Bidir: RX 채널 초기화 ──
    if (!initRxChannels()) {
        Serial.println("[DShot 오류] Bidir RX 채널 초기화 실패!");
        return false;
    }
#endif

    ready_ = true;

#ifdef DSHOT_BIDIR
    Serial.println("[DShot] 4채널 TX + 4채널 RX 초기화 완료 (Bidir)");
    Serial.printf("        모터 극수: %d → 기계적 RPM = eRPM / %d\n",
                  MOTOR_POLES, MOTOR_POLES / 2);
    Serial.println("        ★ ESC에서 Bidirectional DSHOT = ON 설정 필요");
#else
    Serial.println("[DShot] 4채널 TX 초기화 완료 (단방향)");
#endif
    return true;
}

// ============================================================================
//  buildFrame — DShot 프레임 생성
//  Bidir 모드: invert_out=1로 신호 자체가 반전되므로 buildFrame은 v7과 동일
// ============================================================================
void DShot_ESP32::buildFrame(rmt_symbol_word_t *sym,
                              uint16_t throttle, bool telem)
{
    if (throttle > 2047) throttle = 2047;

    uint16_t packet = (uint16_t)(throttle << 1) | (telem ? 1u : 0u);
    uint16_t crc    = calcCRC(packet);
    uint16_t word   = (uint16_t)((packet << 4) | crc);

    for (int i = 0; i < 16; i++) {
        if (word & (1u << (15 - i))) {
            sym[i].level0 = 1;  sym[i].duration0 = DS_T1H;
            sym[i].level1 = 0;  sym[i].duration1 = DS_T1L;
        } else {
            sym[i].level0 = 1;  sym[i].duration0 = DS_T0H;
            sym[i].level1 = 0;  sym[i].duration1 = DS_T0L;
        }
    }
    // Reset Gap: LOW 5µs (invert_out=1이면 HIGH 5µs로 전송됨)
    sym[16].level0    = 0;
    sym[16].duration0 = DS_RESET_TICKS;
    sym[16].level1    = 0;
    sym[16].duration1 = 0;
}

void DShot_ESP32::txStart(uint8_t motor)
{
    if (!ready_ || !txChan_[motor]) return;
    rmt_transmit_config_t cfg = {};
    cfg.loop_count = 0;
    rmt_transmit(txChan_[motor], encoder_[motor],
                 symbols_[motor],
                 sizeof(rmt_symbol_word_t) * DS_FRAME_SYMBOLS, &cfg);
}

void DShot_ESP32::txWait(uint8_t motor)
{
    if (!ready_ || !txChan_[motor]) return;
    // ★ Betaflight 패턴 적용: portMAX_DELAY(무한대기) 대신 유한 타임아웃 사용.
    //   기존에는 RMT 하드웨어에 이상이 생기면 이 호출이 영원히 반환되지 않아
    //   1kHz 비행 루프 전체가 멈추는 치명적 위험이 있었음.
    //   ESP-IDF rmt_tx_wait_all_done()의 timeout 단위는 ms이므로,
    //   1000µs 루프 예산을 고려해 2ms로 설정 — 정상 시 TX는 수십~수백µs 내 완료되므로
    //   2ms 초과는 명백한 이상 상황으로 간주해 즉시 다음 루프로 넘어간다.
    esp_err_t err = rmt_tx_wait_all_done(txChan_[motor], 2 /* ms */);
    if (err != ESP_OK) {
        txTimeoutCount_++;   // ★ 진단용 카운터 — getTxTimeoutCount()로 모니터링
    }
}

// ============================================================================
//  writeThrottle — 4모터 동시 전송
//  Bidir: ① 이전 RX 결과 처리 → ② TX → ③ GPIO INPUT → ④ RX 시작
// ============================================================================
void DShot_ESP32::writeThrottle(const uint16_t throttle[4])
{
    if (!ready_) return;

#ifdef DSHOT_BIDIR
    // ── ① 이전 루프의 RX 결과 처리 ──
    // 1ms 전에 시작한 RX가 완료되어 있어야 함 (ESC 응답 ~150µs)
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        processRx(m);
    }

    // ── ② TX 전: GPIO를 OUTPUT으로 재설정 ──
    // (이전 루프에서 INPUT으로 전환했으므로)
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        gpio_set_direction((gpio_num_t)pins_[m], GPIO_MODE_OUTPUT);
    }
#endif

    // ── 프레임 빌드 ──
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++)
        buildFrame(symbols_[m], throttle[m], false);

    // ── 비동기 동시 TX 시작 ──
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) txStart(m);

    // ── TX 완료 대기 ──
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) txWait(m);

#ifdef DSHOT_BIDIR
    // ── ③ GPIO INPUT 전환 ──
    // TX 완료 후 핀을 해제해야 ESC가 응답을 보낼 수 있음
    // 풀업 저항(10kΩ)이 라인을 HIGH로 유지
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        gpio_set_direction((gpio_num_t)pins_[m], GPIO_MODE_INPUT);
    }

    // ── ④ RX 시작 ──
    // ESC 응답을 캡처하기 위해 즉시 RX 활성화
    // 실제 ESC 응답은 ~26µs 후 도착하므로 충분한 여유 있음
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        startRx(m);
    }
#endif
}

void DShot_ESP32::writeMotor(uint8_t motor, uint16_t throttle)
{
    if (!ready_ || motor >= DSHOT_NUM_MOTORS) return;
    buildFrame(symbols_[motor], throttle, false);
    txStart(motor);
    txWait(motor);
}

// ============================================================================
//  arm — ESC Arming
// ============================================================================
void DShot_ESP32::arm(uint32_t arm_ms)
{
    Serial.printf("[DShot] ESC Arming (%lu ms)...\n", arm_ms);
    uint16_t z[4] = {0, 0, 0, 0};
    uint32_t t0 = millis();
    while (millis() - t0 < arm_ms) {
        writeThrottle(z);
        delayMicroseconds(1000);
    }
    Serial.println("[DShot] ESC Arming 완료");
}

void DShot_ESP32::stopAll()
{
    uint16_t z[4] = {0, 0, 0, 0};
    writeThrottle(z);
}

// ============================================================================
//  sendCommand — 특수 명령 전송 (v7과 동일)
// ============================================================================
void DShot_ESP32::sendCommand(uint8_t motor, uint8_t cmd, uint8_t repeats)
{
    if (!ready_ || motor >= DSHOT_NUM_MOTORS) return;

    uint32_t t0 = millis();
    while (millis() - t0 < 300) {
        for (int m = 0; m < DSHOT_NUM_MOTORS; m++)
            buildFrame(symbols_[m], 0, false);
        for (int m = 0; m < DSHOT_NUM_MOTORS; m++) txStart(m);
        for (int m = 0; m < DSHOT_NUM_MOTORS; m++) txWait(m);
        delayMicroseconds(1000);
    }

    for (uint8_t r = 0; r < repeats; r++) {
        bool sendCmd = (r % 2 == 1);
        for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
            if (sendCmd && m == (int)motor)
                buildFrame(symbols_[m], (uint16_t)cmd, true);
            else
                buildFrame(symbols_[m], 0, false);
        }
        for (int m = 0; m < DSHOT_NUM_MOTORS; m++) txStart(m);
        for (int m = 0; m < DSHOT_NUM_MOTORS; m++) txWait(m);
        delayMicroseconds(1000);
    }
}

// ============================================================================
//  usToDShot / normToDShot / calcCRC (v7과 동일)
// ============================================================================
uint16_t DShot_ESP32::usToDShot(float us)
{
    if (us <= 1000.0f) return 0;
    float ds = 48.0f + (us - 1000.0f) * (2047.0f - 48.0f) / 1000.0f;
    if (ds < 48.0f)   return 48;
    if (ds > 2047.0f) return 2047;
    return (uint16_t)ds;
}

uint16_t DShot_ESP32::normToDShot(float n)
{
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return (uint16_t)(48.0f + n * (2047.0f - 48.0f));
}

uint8_t DShot_ESP32::calcCRC(uint16_t packet12)
{
    uint16_t csum = 0;
    uint16_t data = packet12;
    for (int i = 0; i < 3; i++) {
        csum ^= data;
        data >>= 4;
    }
    return (uint8_t)(csum & 0x0F);
}

// ============================================================================
//  ★★★ Bidir DShot 전용 구현부 ★★★
// ============================================================================
#ifdef DSHOT_BIDIR

// ============================================================================
//  initRxChannels — Bidir RX 채널 초기화 (begin()에서 호출)
// ============================================================================
bool DShot_ESP32::initRxChannels()
{
    Serial.println("[DShot Bidir] RX 채널 초기화...");

    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        // ── RX 채널 설정 ──
        // TX와 같은 GPIO 핀 사용 (양방향 단선 통신)
        rmt_rx_channel_config_t rxCfg = {};
        rxCfg.gpio_num          = (gpio_num_t)pins_[m];
        rxCfg.clk_src           = RMT_CLK_SRC_DEFAULT;
        rxCfg.resolution_hz     = DS_RMT_RESOLUTION_HZ;   // 40MHz (TX와 동일 해상도)
        rxCfg.mem_block_symbols = DS_MEM_BLOCK_SYMBOLS;   // 48 (TX와 동일)
        rxCfg.flags.invert_in   = 0;  // ESC 응답은 정상 극성
        rxCfg.flags.with_dma    = 0;

        esp_err_t err = rmt_new_rx_channel(&rxCfg, &rxChan_[m]);
        if (err != ESP_OK) {
            Serial.printf("[DShot 오류] 모터%d RX 채널 실패: %s\n", m+1, esp_err_to_name(err));
            return false;
        }

        // ── RX 완료 콜백 등록 ──
        rmt_rx_event_callbacks_t rxEvt = {};
        rxEvt.on_recv_done = rxDoneISR;
        err = rmt_rx_register_event_callbacks(rxChan_[m], &rxEvt, this);
        if (err != ESP_OK) {
            Serial.printf("[DShot 오류] 모터%d RX 콜백 등록 실패: %s\n", m+1, esp_err_to_name(err));
            return false;
        }

        // ── RX 채널 활성화 ──
        err = rmt_enable(rxChan_[m]);
        if (err != ESP_OK) {
            Serial.printf("[DShot 오류] 모터%d RX 활성화 실패: %s\n", m+1, esp_err_to_name(err));
            return false;
        }

        Serial.printf("[DShot Bidir] 모터%d RX (GPIO%d) OK\n", m+1, pins_[m]);
    }
    return true;
}

// ============================================================================
//  rxDoneISR — RX 완료 인터럽트 핸들러 (IRAM 상주)
//  ─────────────────────────────────────────────────────────────────────────
//  RMT RX가 완료되면 (EOF 펄스 감지) 이 함수가 ISR 컨텍스트에서 호출됨.
//  여기서는 최소한의 작업만: 플래그 설정 + 심볼 수 저장.
//  실제 GCR 디코딩은 메인 루프의 processRx()에서 수행.
// ============================================================================
bool IRAM_ATTR DShot_ESP32::rxDoneISR(rmt_channel_handle_t ch,
                                       const rmt_rx_done_event_data_t* ev,
                                       void* userCtx)
{
    DShot_ESP32* self = (DShot_ESP32*)userCtx;

    // 어느 모터의 RX가 완료됐는지 찾아서 플래그 설정
    for (int m = 0; m < DSHOT_NUM_MOTORS; m++) {
        if (self->rxChan_[m] == ch) {
            self->rxSymCnt_[m] = ev->num_symbols;
            self->rxDone_[m]   = true;
            break;
        }
    }
    return false;  // 고우선순위 태스크 깨우기 없음
}

// ============================================================================
//  startRx — 단일 모터 RX 시작 (writeThrottle에서 TX 완료 후 호출)
// ============================================================================
void DShot_ESP32::startRx(uint8_t motor)
{
    if (!rxChan_[motor]) return;

    rxDone_[motor]   = false;
    rxSymCnt_[motor] = 0;

    rmt_receive_config_t rxCfg = {};
    // 최소 펄스: 잡음 필터링 (1µs 미만 무시)
    rxCfg.signal_range_min_ns = DS_RX_MIN_PULSE_NS;
    // 최대 펄스: 이보다 길면 EOF (수신 완료 트리거)
    // GCR 최장 연속 펄스 < 4×3.33µs ≈ 13µs < 14µs
    rxCfg.signal_range_max_ns = DS_RX_MAX_PULSE_NS;

    rmt_receive(rxChan_[motor],
                rxBuf_[motor],
                sizeof(rmt_symbol_word_t) * DS_RX_BUF_SYMBOLS,
                &rxCfg);
}

// ============================================================================
//  processRx — 이전 RX 결과를 메인 루프에서 처리 (GCR 디코딩)
//  ─────────────────────────────────────────────────────────────────────────
//  1ms 이전에 startRx()로 시작한 수신이 완료됐을 경우 처리.
//  rxDone_[motor] 플래그로 완료 여부 확인.
// ============================================================================
void DShot_ESP32::processRx(uint8_t motor)
{
    if (!rxDone_[motor]) {
        // 아직 RX 미완료 (첫 번째 루프 또는 ESC 무응답)
        rpmValid_[motor] = false;
        return;
    }

    rxTotalFrames_[motor]++;

    uint32_t bits20 = 0;
    uint16_t value16 = 0;

    // ── 심볼 → 20비트 스트림 변환 ──
    if (!symsToBits(rxBuf_[motor], rxSymCnt_[motor], bits20)) {
        rpmValid_[motor] = false;
        return;
    }

    // ── GCR 4b5b 디코딩 ──
    if (!decodeGCR(bits20, value16)) {
        rpmValid_[motor] = false;
        return;
    }

    // ── eRPM 계산 ──
    uint32_t erpm = erpmFromValue(value16);
    erpm_[motor]     = erpm;
    rpmValid_[motor] = true;
    rxGoodFrames_[motor]++;

    // ── 기계적 RPM LPF 적용 ──
    // LPF 계수: α = 2π×fc×dt / (1 + 2π×fc×dt)
    // fc = RPM_FILTER_LPF_HZ, dt = 0.001s (1kHz)
    // 예) fc=50Hz → α ≈ 0.239
    const float dt = 0.001f;
    const float kAlpha = (2.0f * 3.14159f * RPM_FILTER_LPF_HZ * dt)
                       / (1.0f + 2.0f * 3.14159f * RPM_FILTER_LPF_HZ * dt);
    float rawRPM = ERPM_TO_RPM(erpm);
    rpmFilt_[motor] = kAlpha * rawRPM + (1.0f - kAlpha) * rpmFilt_[motor];
}

// ============================================================================
//  symsToBits — RMT 심볼 → 20비트 NRZ 스트림 재구성
//  ─────────────────────────────────────────────────────────────────────────
//  RMT RX는 연속 동일 레벨 펄스를 하나의 심볼로 병합함.
//  예) '1','1','0' → HIGH×2(6.67µs), LOW×1(3.33µs)
//  따라서 각 심볼의 지속 시간 ÷ 비트 주기 = 해당 비트 수를 계산.
//
//  ★ bits20 출력 형식: MSB가 bit19(첫 수신 비트)
// ============================================================================
bool DShot_ESP32::symsToBits(const rmt_symbol_word_t* syms, size_t numSyms,
                               uint32_t& bits20)
{
    if (numSyms == 0) return false;

    bits20 = 0;
    uint32_t bitPos = 0;  // 현재까지 채운 비트 수

    for (size_t i = 0; i < numSyms && bitPos < 20; i++) {
        // ── 상단(level0, duration0) ──
        if (syms[i].duration0 > 0) {
            // 반올림: 틱 수 → 비트 수 (DS_TICKS_PER_BIT = 133)
            uint32_t cnt = (syms[i].duration0 + DS_TICKS_PER_BIT / 2) / DS_TICKS_PER_BIT;
            if (cnt == 0) cnt = 1;  // 최소 1비트

            for (uint32_t b = 0; b < cnt && bitPos < 20; b++) {
                if (syms[i].level0) {
                    bits20 |= (1u << (19 - bitPos));
                }
                bitPos++;
            }
        }

        // ── 하단(level1, duration1) ──
        if (syms[i].duration1 > 0 && bitPos < 20) {
            uint32_t cnt = (syms[i].duration1 + DS_TICKS_PER_BIT / 2) / DS_TICKS_PER_BIT;
            if (cnt == 0) cnt = 1;

            for (uint32_t b = 0; b < cnt && bitPos < 20; b++) {
                if (syms[i].level1) {
                    bits20 |= (1u << (19 - bitPos));
                }
                bitPos++;
            }
        }
    }

    // 최소 16비트 이상이어야 유효 (노이즈 필터)
    return (bitPos >= 16);
}

// ============================================================================
//  decodeGCR — GCR 20비트 → 16비트 데이터 + CRC 검증
//  ─────────────────────────────────────────────────────────────────────────
//  Step 1: NRZI 언폴딩 — bits20 ^= (bits20 >> 1)
//          (ESC는 data[i] XOR data[i-1] 형태로 전송 = NRZI 인코딩)
//  Step 2: 5비트 × 4 = 20비트를 4개 니블로 변환 (MSB 먼저)
//  Step 3: 16비트 = 상위12(eRPM 기간) + 하위4(CRC)
//  Step 4: XOR CRC 검증
// ============================================================================
bool DShot_ESP32::decodeGCR(uint32_t bits20, uint16_t& value16)
{
    // ── Step 1: NRZI 언폴딩 ──
    // NRZI decode: data[i] = transmitted[i] XOR transmitted[i-1]
    // 벡터 표현: decoded = raw XOR (raw >> 1)
    uint32_t decoded = bits20 ^ (bits20 >> 1);

    // ── Step 2: 4개 5비트 코드 → 4개 4비트 니블 ──
    // decoded의 비트 배치: [19:15]=니블3(MSN), [14:10]=니블2, [9:5]=니블1, [4:0]=니블0(LSN)
    uint16_t nibble[4];
    for (int n = 3; n >= 0; n--) {
        uint8_t code5 = (decoded >> (n * 5)) & 0x1F;
        uint8_t nib   = GCR_DECODE_TABLE[code5];
        if (nib == GCR_INVALID) {
            return false;  // 유효하지 않은 5비트 코드
        }
        nibble[3 - n] = nib;
    }

    // ── Step 3: 16비트 값 조합 ──
    // nibble[0]=최상위 니블, nibble[3]=최하위 니블
    value16 = (uint16_t)((nibble[0] << 12) | (nibble[1] << 8) |
                          (nibble[2] << 4) | nibble[3]);

    // ── Step 4: XOR CRC 검증 ──
    // CRC = 12비트 기간의 3개 4비트 니블 XOR
    uint16_t period = value16 >> 4;      // 상위 12비트
    uint8_t  rxCRC  = value16 & 0xF;    // 하위 4비트 (수신된 CRC)
    uint8_t  calcCRC = ((period >> 8) ^ (period >> 4) ^ period) & 0xF;

    if (rxCRC != calcCRC) {
        return false;  // CRC 불일치
    }

    return true;
}

// ============================================================================
//  erpmFromValue — 16비트 값 → eRPM 계산
//  ─────────────────────────────────────────────────────────────────────────
//  value 상위 12비트 = eRPM 기간 (단위: 1.25µs = 5/4µs)
//  기간 = 0      → 모터 정지 (eRPM = 0)
//  기간 = 0xFFF  → 유효하지 않음 (eRPM = 0)
//  eRPM = 60,000,000µs/min ÷ (기간 × 1.25µs)
// ============================================================================
uint32_t DShot_ESP32::erpmFromValue(uint16_t value)
{
    uint16_t period = value >> 4;  // 상위 12비트

    if (period == 0 || period == GCR_PERIOD_INVALID) {
        return 0;  // 정지 또는 유효하지 않음
    }

    // 기간(µs) = period × 5/4 = period × 1.25
    // eRPM = 60,000,000 / period_µs
    //      = 60,000,000 / (period × 5 / 4)
    //      = 60,000,000 × 4 / (period × 5)
    //      = 48,000,000 / period
    return (uint32_t)(48000000UL / (uint32_t)period);
}

// ============================================================================
//  getRPM / getERPM / isRPMValid / getRPMSuccessRate
// ============================================================================
float DShot_ESP32::getRPM(uint8_t motor) const
{
    if (motor >= DSHOT_NUM_MOTORS || !rpmValid_[motor]) return 0.0f;
    return rpmFilt_[motor];
}

uint32_t DShot_ESP32::getERPM(uint8_t motor) const
{
    if (motor >= DSHOT_NUM_MOTORS) return 0;
    return erpm_[motor];
}

bool DShot_ESP32::isRPMValid(uint8_t motor) const
{
    if (motor >= DSHOT_NUM_MOTORS) return false;
    return rpmValid_[motor];
}

float DShot_ESP32::getRPMSuccessRate(uint8_t motor) const
{
    if (motor >= DSHOT_NUM_MOTORS) return 0.0f;
    if (rxTotalFrames_[motor] == 0) return 0.0f;
    return (float)rxGoodFrames_[motor] / (float)rxTotalFrames_[motor];
}

#endif // DSHOT_BIDIR
