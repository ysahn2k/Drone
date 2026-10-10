// ============================================================================
//  ELRS_Receiver.cpp  —  ELRS/CRSF 수신기 드라이버 구현  (v8)
//
//  ★ v8 수정 이력
//  ─────────────────────────────────────────────────────────────────────────
//  [1] CRSF 파서: 0xC8(FC) 외에 0x00(브로드캐스트), 0xEA(RX) 주소도 수신
//      → DEVICE_PING(0x28) 등 일부 프레임이 0x00으로 전달되어 유실 방지
//  [2] sendCRSF: 첫 바이트를 0xC8 → 0xEE(TX 모듈 주소)로 수정
//      → TX 모듈은 자신 주소(0xEE)로 온 패킷만 처리함
//  [3] sendCRSF: CRC 계산에 TYPE 바이트 포함 (CRSF 규격 준수)
//      → 이전: crc(PAYLOAD) / 수정: crc(TYPE + PAYLOAD)
//  [4] parseLink: 10바이트 전체 파싱 + lastPktMs_ 갱신
//  [5] DEVICE_PING(0x28) 수신 → DEVICE_INFO(0x29) 응답 구현
//      → Model Match OFF 정상 동작 보장
// ============================================================================
#include "ELRS_Receiver.h"

// ============================================================================
//  생성자
// ============================================================================
ELRS_Receiver::ELRS_Receiver()
    : ser_(nullptr), rxIdx_(0), inPacket_(false),
      lastPktMs_(0), fsTimeoutMs_(FAILSAFE_TIMEOUT_MS), newFrame_(false)
{
    for (int i = 0; i <= ELRS_NUM_CHANNELS; i++)
        ch_[i] = (uint16_t)CRSF_CENTER;
    ch_[CH_THROTTLE] = (uint16_t)CRSF_MIN;
}

// ============================================================================
//  begin — UART 초기화
// ============================================================================
void ELRS_Receiver::begin(HardwareSerial &serial,
                           int rxPin, int txPin, uint32_t baud)
{
    ser_ = &serial;
    serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    Serial.printf("[ELRS] UART 시작: RX=GPIO%d TX=GPIO%d baud=%lu\n",
                  rxPin, txPin, baud);
}

// ============================================================================
//  update — UART 버퍼에서 CRSF 패킷 파싱 (매 루프 호출, 비블로킹)
//  ─────────────────────────────────────────────────────────────────────────
//  CRSF 패킷 구조:
//  [ADDR(1) | LEN(1) | TYPE(1) | PAYLOAD(LEN-2) | CRC(1)]
//
//  수신 가능한 ADDR(첫 바이트):
//    0xC8 = FC 주소     → RC 채널(0x16), 링크 통계(0x14) 등
//    0x00 = 브로드캐스트 → DEVICE_PING(0x28) 등
//    0xEA = RX 모듈     → 일부 extended 프레임
// ============================================================================
bool ELRS_Receiver::update()
{
    if (!ser_) return false;
    newFrame_ = false;

    while (ser_->available()) {
        uint8_t b = (uint8_t)ser_->read();

        if (!inPacket_) {
            if (b == CRSF_SYNC_BYTE     ||   // 0xC8 FC 주소
                b == CRSF_ADDR_BROADCAST ||   // 0x00 브로드캐스트
                b == CRSF_ADDR_RX_MODULE) {   // 0xEA RX 모듈
                rxBuf_[0] = b;
                rxIdx_    = 1;
                inPacket_ = true;
            }
            continue;
        }

        if (rxIdx_ >= CRSF_MAX_PKT_LEN) {
            inPacket_ = false;
            rxIdx_    = 0;
            continue;
        }
        rxBuf_[rxIdx_++] = b;

        if (rxIdx_ == 2) {
            if (rxBuf_[1] < 2 || rxBuf_[1] > (CRSF_MAX_PKT_LEN - 2)) {
                inPacket_ = false;
                rxIdx_    = 0;
            }
            continue;
        }

        uint8_t expected = rxBuf_[1] + 2;
        if (rxIdx_ >= expected) {
            uint8_t calcCrc = crc8dvbs2(&rxBuf_[2], rxBuf_[1] - 1);
            uint8_t recvCrc = rxBuf_[rxIdx_ - 1];
            if (calcCrc == recvCrc) {
                parsePacket(rxBuf_[1] - 2, rxBuf_[2], &rxBuf_[3]);
            }
            inPacket_ = false;
            rxIdx_    = 0;
        }
    }
    return newFrame_;
}

// ============================================================================
//  parsePacket — 패킷 타입별 파서 분기
// ============================================================================
void ELRS_Receiver::parsePacket(uint8_t payLen, uint8_t type,
                                 const uint8_t *pay)
{
    switch (type) {
    case CRSF_TYPE_RC:
        if (payLen >= 22) {
            parseRC(pay);
            lastPktMs_ = millis();
            newFrame_  = true;
        }
        break;

    case CRSF_TYPE_LINK:
        if (payLen >= 8) parseLink(pay);
        break;

    case CRSF_TYPE_DEVICE_PING:
        // ELRS TX 가 FC 장치 탐색 → DEVICE_INFO 응답
        // Model Match OFF 모드에서 핸드셰이크 완료에 필수
        sendDeviceInfo();
        break;

    case CRSF_TYPE_HEARTBEAT:
        lastPktMs_ = millis();
        break;

    default:
        break;
    }
}

// ============================================================================
//  parseRC — 16채널 × 11비트 언패킹 (22바이트 → 채널 값 배열)
// ============================================================================
void ELRS_Receiver::parseRC(const uint8_t *p)
{
    ch_[1]  = (uint16_t)( (p[0]        | ((uint16_t)p[1]  << 8)) & 0x7FF);
    ch_[2]  = (uint16_t)(((p[1]  >> 3) | ((uint16_t)p[2]  << 5)) & 0x7FF);
    ch_[3]  = (uint16_t)(((p[2]  >> 6) | ((uint16_t)p[3]  << 2) | ((uint16_t)p[4] << 10)) & 0x7FF);
    ch_[4]  = (uint16_t)(((p[4]  >> 1) | ((uint16_t)p[5]  << 7)) & 0x7FF);
    ch_[5]  = (uint16_t)(((p[5]  >> 4) | ((uint16_t)p[6]  << 4)) & 0x7FF);
    ch_[6]  = (uint16_t)(((p[6]  >> 7) | ((uint16_t)p[7]  << 1) | ((uint16_t)p[8] << 9)) & 0x7FF);
    ch_[7]  = (uint16_t)(((p[8]  >> 2) | ((uint16_t)p[9]  << 6)) & 0x7FF);
    ch_[8]  = (uint16_t)(((p[9]  >> 5) | ((uint16_t)p[10] << 3)) & 0x7FF);
    ch_[9]  = (uint16_t)( (p[11]       | ((uint16_t)p[12] << 8)) & 0x7FF);
    ch_[10] = (uint16_t)(((p[12] >> 3) | ((uint16_t)p[13] << 5)) & 0x7FF);
    ch_[11] = (uint16_t)(((p[13] >> 6) | ((uint16_t)p[14] << 2) | ((uint16_t)p[15] << 10)) & 0x7FF);
    ch_[12] = (uint16_t)(((p[15] >> 1) | ((uint16_t)p[16] << 7)) & 0x7FF);
    ch_[13] = (uint16_t)(((p[16] >> 4) | ((uint16_t)p[17] << 4)) & 0x7FF);
    ch_[14] = (uint16_t)(((p[17] >> 7) | ((uint16_t)p[18] << 1) | ((uint16_t)p[19] << 9)) & 0x7FF);
    ch_[15] = (uint16_t)(((p[19] >> 2) | ((uint16_t)p[20] << 6)) & 0x7FF);
    ch_[16] = (uint16_t)(((p[20] >> 5) | ((uint16_t)p[21] << 3)) & 0x7FF);
}

// ============================================================================
//  parseLink — 링크 통계 파싱 (CRSF 0x14, 10바이트)
//  Byte 0: 업링크 RSSI1  Byte 1: RSSI2  Byte 2: LQ[%]  Byte 3: SNR[dB]
//  Byte 4: 활성 안테나   Byte 5: RF모드  Byte 6: TX출력
//  Byte 7: 다운링크RSSI  Byte 8: 다운LQ  Byte 9: 다운SNR
// ============================================================================
void ELRS_Receiver::parseLink(const uint8_t *p)
{
    link_.rssi1       = p[0];
    link_.rssi2       = p[1];
    link_.linkQuality = p[2];
    link_.snr         = (int8_t)p[3];
    link_.txPower     = p[6];
    lastPktMs_ = millis();
}

// ============================================================================
//  채널 값 접근 함수
// ============================================================================
uint16_t ELRS_Receiver::getChannelRaw(uint8_t ch) const
{
    if (ch < 1 || ch > ELRS_NUM_CHANNELS) return (uint16_t)CRSF_CENTER;
    return ch_[ch];
}

float ELRS_Receiver::getChannelUs(uint8_t ch) const
{
    float raw = (float)getChannelRaw(ch);
    return (raw - (float)CRSF_MIN) * (2000.0f - 1000.0f)
           / ((float)CRSF_MAX - (float)CRSF_MIN) + 1000.0f;
}

float ELRS_Receiver::getChannelNorm(uint8_t ch) const
{
    float raw = (float)getChannelRaw(ch);
    return (raw - (float)CRSF_MIN) * 2.0f
           / ((float)CRSF_MAX - (float)CRSF_MIN) - 1.0f;
}

bool ELRS_Receiver::getAuxHigh(uint8_t ch) const
{
    return getChannelUs(ch) > 1600.0f;
}

bool ELRS_Receiver::isArmed()       const { return getAuxHigh(CH_ARM);       }
bool ELRS_Receiver::isIMUCalibReq() const { return getAuxHigh(CH_IMU_CALIB); }
bool ELRS_Receiver::isRTHActive()   const { return getAuxHigh(CH_RTH);       }

SwPos3 ELRS_Receiver::getSwitch3(uint8_t ch) const
{
    float us = getChannelUs(ch);
    if      (us <= SW_LOW_MAX)  return SW3_LOW;
    else if (us <= SW_MID_MAX)  return SW3_MID;
    else                        return SW3_HIGH;
}

SwPos3 ELRS_Receiver::getFlightModeSwitch() const { return getSwitch3(CH_FLIGHTMODE); }
SwPos3 ELRS_Receiver::getAuxFuncSwitch()    const { return getSwitch3(CH_AUX_FUNC);   }

bool ELRS_Receiver::isFailsafe() const
{
    return (millis() - lastPktMs_) > fsTimeoutMs_;
}

uint32_t ELRS_Receiver::timeSinceLastPacketMs() const
{
    return millis() - lastPktMs_;
}

// ============================================================================
//  sendCRSF — CRSF 프레임 공통 송신 헬퍼
//  ─────────────────────────────────────────────────────────────────────────
//  ★ Betaflight 동일 방식:
//    일반 텔레메트리 (0x1E, 0x08, 0x21, 0x02): 첫 바이트 = 0xC8 (FC/SYNC)
//    DEVICE_INFO 응답 (0x29):                   첫 바이트 = 0xEE (TX 모듈)
//
//  이전에 모든 프레임을 0xEE로 보냈다가 텔레메트리가 ELRS에 전달 안 됐음.
//  ELRS RX는 0xC8로 시작하는 텔레메트리 프레임을 TX 방향으로 포워딩함.
//
//  패킷 구조: [DEST_ADDR | LEN | TYPE | PAYLOAD... | CRC8]
//  CRC 범위: TYPE + PAYLOAD (DEST_ADDR, LEN 제외)
// ============================================================================
void ELRS_Receiver::sendCRSF(uint8_t type,
                               const uint8_t *payload, uint8_t payLen)
{
    sendCRSFWithAddr(CRSF_SYNC_BYTE, type, payload, payLen);  // 0xC8
}

void ELRS_Receiver::sendCRSFWithAddr(uint8_t destAddr, uint8_t type,
                                      const uint8_t *payload, uint8_t payLen)
{
    if (!ser_) return;

    uint8_t len = payLen + 2;   // TYPE(1) + PAYLOAD + CRC(1)

    // CRC: TYPE + PAYLOAD 포함
    uint8_t crcBuf[66];
    crcBuf[0] = type;
    if (payLen > 0 && payload) memcpy(&crcBuf[1], payload, payLen);
    uint8_t crc = crc8dvbs2(crcBuf, 1 + payLen);

    ser_->write(destAddr);
    ser_->write(len);
    ser_->write(type);
    if (payLen > 0 && payload) ser_->write(payload, payLen);
    ser_->write(crc);
}

// ============================================================================
//  sendFlightModeTelemetry — 비행 모드 텔레메트리 (CRSF 0x21)
//  ─────────────────────────────────────────────────────────────────────────
//  ELRS가 FC 텔레메트리 스트림을 인식하기 위해 필요한 핵심 프레임.
//  Betaflight도 항상 이 프레임을 attitude/battery보다 먼저 전송한다.
//  페이로드: NULL 종료 ASCII 문자열 (최대 15자)
//  예: "ANGLE" / "ACRO" / "ALTHOLD" / "FAILSAFE"
// ============================================================================
void ELRS_Receiver::sendFlightModeTelemetry(const char *mode)
{
    if (!mode) return;
    uint8_t len = (uint8_t)strlen(mode);
    if (len > 15) len = 15;

    uint8_t pay[16];
    memcpy(pay, mode, len);
    pay[len] = '\0';

    sendCRSF(0x21, pay, len + 1);  // NULL 포함 전송
}

// ============================================================================
//  sendAttitudeTelemetry — 자세 텔레메트리 (CRSF 0x1E)
//  페이로드: Pitch(2B) Roll(2B) Yaw(2B), 단위: 라디안 × 10000 (빅엔디안)
// ============================================================================
void ELRS_Receiver::sendAttitudeTelemetry(float rollDeg, float pitchDeg,
                                           float yawDeg)
{
    int16_t pitchR = (int16_t)(pitchDeg * DEG_TO_RAD * 10000.0f);
    int16_t rollR  = (int16_t)(rollDeg  * DEG_TO_RAD * 10000.0f);
    int16_t yawR   = (int16_t)(yawDeg   * DEG_TO_RAD * 10000.0f);

    uint8_t pay[6];
    pay[0] = (uint8_t)(pitchR >> 8);  pay[1] = (uint8_t)(pitchR & 0xFF);
    pay[2] = (uint8_t)(rollR  >> 8);  pay[3] = (uint8_t)(rollR  & 0xFF);
    pay[4] = (uint8_t)(yawR   >> 8);  pay[5] = (uint8_t)(yawR   & 0xFF);

    sendCRSF(0x1E, pay, sizeof(pay));
}

// ============================================================================
//  sendBatteryTelemetry — 배터리 텔레메트리 (CRSF 0x08)
//  페이로드: 전압(2B cV) 전류(2B cA) 용량(3B mAh) 잔량(1B %)
// ============================================================================
void ELRS_Receiver::sendBatteryTelemetry(float voltV, float currA,
                                          uint32_t mAh)
{
    uint16_t v = (uint16_t)(voltV * 10.0f);  // ★ CRSF 규격: 0.1V 단위 (×100→120V 오류 수정)
    uint16_t c = (uint16_t)(currA * 10.0f);  // ★ CRSF 규격: 0.1A 단위

    uint8_t pay[8];
    pay[0] = (uint8_t)(v   >> 8);  pay[1] = (uint8_t)(v   & 0xFF);
    pay[2] = (uint8_t)(c   >> 8);  pay[3] = (uint8_t)(c   & 0xFF);
    pay[4] = (uint8_t)(mAh >> 16);
    pay[5] = (uint8_t)(mAh >>  8);
    pay[6] = (uint8_t)(mAh &  0xFF);
    pay[7] = 0;

    sendCRSF(0x08, pay, sizeof(pay));
}

// ============================================================================
//  sendGPSTelemetry — GPS 텔레메트리 (CRSF 0x02)
// ============================================================================
void ELRS_Receiver::sendGPSTelemetry(const uint8_t *payload, uint8_t payLen)
{
    if (!payload || payLen == 0) return;
    sendCRSF(0x02, payload, payLen);
}

// ============================================================================
//  crc8dvbs2 — CRC-8/DVB-S2 (다항식 0xD5)
// ============================================================================
uint8_t ELRS_Receiver::crc8dvbs2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80)
                  ? (uint8_t)((crc << 1) ^ 0xD5)
                  : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// ============================================================================
//  sendDeviceInfo — DEVICE_PING(0x28) 응답
//  ─────────────────────────────────────────────────────────────────────────
//  Model Match OFF 동작을 위한 CRSF 핸드셰이크 응답.
//  TX 모듈이 FC 장치 정보를 확인한 후 RC 데이터 전달을 시작함.
//
//  페이로드: [destAddr][srcAddr][devName\0][serial4B][HW버전4B][SW버전4B]
//            [파라미터수1B][파라미터버전1B]
// ============================================================================
void ELRS_Receiver::sendDeviceInfo()
{
    const char    devName[] = "QuadDrone v8";
    const uint8_t nameLen   = sizeof(devName);  // null 포함

    const uint8_t payLen = 2 + nameLen + 4 + 4 + 4 + 1 + 1;
    uint8_t pay[payLen];
    uint8_t i = 0;

    pay[i++] = CRSF_ADDR_TX_MODULE;  // dest
    pay[i++] = CRSF_ADDR_FC;         // src

    memcpy(&pay[i], devName, nameLen);
    i += nameLen;

    pay[i++] = 0x00; pay[i++] = 0x00; pay[i++] = 0x00; pay[i++] = 0x01; // serial
    pay[i++] = 0x00; pay[i++] = 0x00; pay[i++] = 0x00; pay[i++] = 0x01; // HW 1.0
    pay[i++] = 0x00; pay[i++] = 0x00; pay[i++] = 0x00; pay[i++] = 0x08; // SW 8.0
    pay[i++] = 0x00;  // 파라미터 수
    pay[i++] = 0x00;  // 파라미터 버전

    // DEVICE_INFO는 TX 모듈에 직접 응답 → 0xEE 사용
    sendCRSFWithAddr(CRSF_ADDR_TX_MODULE, CRSF_TYPE_DEVICE_INFO, pay, payLen);
}
