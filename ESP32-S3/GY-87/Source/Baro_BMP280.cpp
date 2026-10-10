// ============================================================================
//  Baro_BMP280.cpp  —  BMP280 기압 센서 SPI 드라이버 구현  (v7)
// ============================================================================
#include "Baro_BMP280.h"

// ============================================================================
//  생성자
// ============================================================================
Baro_BMP280::Baro_BMP280()
    : csPin_(PIN_CS_BMP280), spi_(&SPI), spiFreq_(SPI_FREQ_BMP280),
      ready_(false),
      temperature_(0), pressure_(101325.0f), altitude_(0),
      seaLevelPa_(101325.0f), t_fine_(0)
{}

// ============================================================================
//  begin — SPI 초기화 + BMP280 설정
//  ─────────────────────────────────────────────────────────────────────────
//  호출 순서:
//  1) CS 핀을 HIGH로 설정 (비활성화 상태로 시작)
//  2) SPI 버스는 메인에서 이미 begin() 했으므로 여기서는 시작 안 함
//  3) BMP280 칩 ID 확인
//  4) 소프트 리셋
//  5) 교정 계수 읽기
//  6) 측정 설정 (오버샘플링, 필터, 모드)
// ============================================================================
bool Baro_BMP280::begin(int csPin, uint32_t spiFreqHz, SPIClass &spiInst)
{
    csPin_   = csPin;
    spi_     = &spiInst;
    spiFreq_ = spiFreqHz;

    // CS 핀 초기화: OUTPUT, 기본 HIGH (비활성)
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    delay(10);

    // ── 칩 ID 확인 (0xD0 레지스터) ──
    // 같은 SPI 인터페이스를 사용하는 두 센서의 칩 ID:
    //   BMP280 : 0x60  (기압 + 온도만)
    //   BME280 : 0x58  (기압 + 온도 + 습도)
    // 두 칩은 레지스터 맵과 교정 공식이 동일하므로 같은 드라이버로 동작합니다.
    // 단, BME280의 습도 기능은 이 드라이버에서 사용하지 않습니다.
    uint8_t chipId = spiRead(BMP280_REG_ID);
    if (chipId == 0x60) {
        Serial.println("[Baro] BMP280 감지 (칩 ID: 0x60)");
    } else if (chipId == 0x58) {
        Serial.println("[Baro] BME280 감지 (칩 ID: 0x58) — 기압/온도만 사용");
    } else {
        Serial.printf("[Baro] 칩 ID 오류: 0x%02X (예상: 0x60=BMP280 또는 0x58=BME280)\n", chipId);
        Serial.println("       배선 확인: CS=GPIO" + String(csPin_) +
                       " SCK=GPIO" + String(PIN_SPI_SCK) +
                       " MISO=GPIO" + String(PIN_SPI_MISO) +
                       " MOSI=GPIO" + String(PIN_SPI_MOSI));
        return false;
    }

    // 소프트 리셋 (0xE0 레지스터에 0xB6 쓰기)
    spiWrite(BMP280_REG_RESET, 0xB6);
    delay(10);   // 리셋 완료 대기

    // 교정 계수 읽기
    if (!readCalibration()) {
        Serial.println("[BMP280] 교정 데이터 읽기 실패");
        return false;
    }

    // 측정 설정
    configure();

    ready_ = true;
    Serial.println("[BMP280] SPI 초기화 완료 (CS=GPIO" +
                   String(csPin_) + " freq=" + String(spiFreqHz/1000000) + "MHz)");
    return true;
}

// ============================================================================
//  configure — BMP280 측정 모드 설정
//  ─────────────────────────────────────────────────────────────────────────
//  ctrl_meas (0xF4):
//    [7:5] osrs_t = 온도 오버샘플링 (010 = ×2)
//    [4:2] osrs_p = 기압 오버샘플링 (101 = ×16, 최고 정밀도)
//    [1:0] mode   = 동작 모드 (11 = Normal, 자동 반복 측정)
//
//  config (0xF5):
//    [7:5] t_sb   = 대기 시간 (000 = 0.5ms, 최소)
//    [4:2] filter = IIR 필터 계수 (010 = 4, 노이즈 감소)
//    [0]   spi3w  = 3선 SPI 모드 (0 = 비활성, 4선 SPI 사용)
// ============================================================================
void Baro_BMP280::configure()
{
    // ctrl_meas: osrs_t=010(×2), osrs_p=101(×16), mode=11(Normal)
    // 바이너리: 01001111 = 0x4F
    uint8_t ctrl = (uint8_t)((BMP280_OSS_TEMP  << 5) |
                              (BMP280_OSS_PRESS << 2) |
                              0x03);   // Normal 모드
    spiWrite(BMP280_REG_CTRL_MEAS, ctrl);

    // config: t_sb=000(0.5ms), filter=010(계수4), spi3w=0
    // 바이너리: 00010000 = 0x10
    uint8_t cfg = (uint8_t)((BMP280_FILTER << 2));
    spiWrite(BMP280_REG_CONFIG, cfg);
}

// ============================================================================
//  readCalibration — 교정 계수 읽기 (begin에서 1회 호출)
//  ─────────────────────────────────────────────────────────────────────────
//  BMP280은 0x88~0x9F 주소에 24바이트의 교정 계수를 저장합니다.
//  리틀 엔디안(작은 바이트가 먼저) 형식으로 저장되어 있습니다.
// ============================================================================
bool Baro_BMP280::readCalibration()
{
    uint8_t buf[24];
    spiReadN(BMP280_REG_CALIB, 24, buf);

    // 2바이트를 합쳐 16비트 정수로 변환 (리틀 엔디안)
    // buf[1:0] → 상위바이트:하위바이트 순서로 결합
    cal_.dig_T1 = (uint16_t)((buf[1]  << 8) | buf[0]);
    cal_.dig_T2 = (int16_t) ((buf[3]  << 8) | buf[2]);
    cal_.dig_T3 = (int16_t) ((buf[5]  << 8) | buf[4]);
    cal_.dig_P1 = (uint16_t)((buf[7]  << 8) | buf[6]);
    cal_.dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
    cal_.dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
    cal_.dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
    cal_.dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
    cal_.dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
    cal_.dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
    cal_.dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
    cal_.dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);

    return true;
}

// ============================================================================
//  update — 기압·온도 데이터 읽기 및 고도 계산 (매 루프 호출)
//  ─────────────────────────────────────────────────────────────────────────
//  Normal 모드에서 BMP280은 자동으로 측정을 반복합니다.
//  status 레지스터의 measuring 비트(bit3)가 0이면 데이터 준비 완료입니다.
//
//  데이터 레지스터 구조 (0xF7~0xFC, 6바이트):
//    0xF7(PRESS_MSB), 0xF8(PRESS_LSB), 0xF9(PRESS_XLSB) → 기압 20비트
//    0xFA(TEMP_MSB),  0xFB(TEMP_LSB),  0xFC(TEMP_XLSB)  → 온도 20비트
// ============================================================================
void Baro_BMP280::update()
{
    if (!ready_) return;

    // 측정 완료 확인 (status 레지스터 bit3 = measuring)
    uint8_t status = spiRead(BMP280_REG_STATUS);
    if (status & 0x08) return;   // 아직 측정 중이면 이번 루프는 건너뜀

    // 6바이트 연속 읽기 (기압 3바이트 + 온도 3바이트)
    uint8_t raw[6];
    spiReadN(BMP280_REG_PRESS_MSB, 6, raw);

    // 20비트 ADC 값 조립 (상위 3바이트 = 20비트, 하위 4비트 무시)
    // 구조: [MSB 8bit | LSB 8bit | XLSB 4bit | 0000]
    int32_t adc_P = ((int32_t)raw[0] << 12) |
                    ((int32_t)raw[1] <<  4) |
                    ((int32_t)raw[2] >>  4);
    int32_t adc_T = ((int32_t)raw[3] << 12) |
                    ((int32_t)raw[4] <<  4) |
                    ((int32_t)raw[5] >>  4);

    // 온도 보정 (t_fine_ 내부 계산 포함 → 기압 보정에 필요)
    int32_t rawTemp  = compensateTemp(adc_T);
    temperature_ = (float)rawTemp / 100.0f;   // 0.01°C → °C

    // 기압 보정 (Q24.8 고정소수점 반환, 256으로 나누면 Pa)
    uint32_t rawPress = compensatePress(adc_P);
    pressure_ = (float)rawPress / 256.0f;     // Q24.8 → Pa

    // 기압 → 고도 계산
    altitude_ = pressureToAltitude(pressure_);
}

// ============================================================================
//  compensateTemp — 온도 보정 (Bosch BMP280 데이터시트 4.2.3 수식)
//  ─────────────────────────────────────────────────────────────────────────
//  반환값: 온도 × 100 (예: 2350 = 23.50°C)
//  t_fine_ 변수도 함께 계산됨 (기압 보정에 필요)
// ============================================================================
int32_t Baro_BMP280::compensateTemp(int32_t adc_T)
{
    int32_t var1 = (((adc_T >> 3) - ((int32_t)cal_.dig_T1 << 1)) *
                    (int32_t)cal_.dig_T2) >> 11;

    int32_t var2 = (((((adc_T >> 4) - (int32_t)cal_.dig_T1) *
                       ((adc_T >> 4) - (int32_t)cal_.dig_T1)) >> 12) *
                    (int32_t)cal_.dig_T3) >> 14;

    t_fine_ = var1 + var2;   // 기압 보정에 사용되는 내부 변수

    // 최종 온도 = (t_fine × 5 + 128) >> 8  → 0.01°C 단위
    return (t_fine_ * 5 + 128) >> 8;
}

// ============================================================================
//  compensatePress — 기압 보정 (Bosch BMP280 데이터시트 4.2.3 수식)
//  ─────────────────────────────────────────────────────────────────────────
//  반환값: Q24.8 고정소수점 (256으로 나누면 Pa)
//  64비트 정수 연산 사용 (32비트 오버플로 방지)
// ============================================================================
uint32_t Baro_BMP280::compensatePress(int32_t adc_P)
{
    int64_t var1 = (int64_t)t_fine_ - 128000;
    int64_t var2 = var1 * var1 * (int64_t)cal_.dig_P6;
    var2 += (var1 * (int64_t)cal_.dig_P5) << 17;
    var2 += ((int64_t)cal_.dig_P4) << 35;
    var1  = ((var1 * var1 * (int64_t)cal_.dig_P3) >> 8) +
            ((var1 * (int64_t)cal_.dig_P2) << 12);
    var1  = (((int64_t)1 << 47) + var1) * (int64_t)cal_.dig_P1 >> 33;

    if (var1 == 0) return 0;   // 0 나누기 방지

    int64_t p = 1048576LL - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)cal_.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)cal_.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)cal_.dig_P7 << 4);

    return (uint32_t)p;
}

// ============================================================================
//  pressureToAltitude — 기압 → 고도 계산 (국제 표준 기압 공식)
//  ─────────────────────────────────────────────────────────────────────────
//  공식: h = 44330 × (1 - (P/P0)^(1/5.255))
//    P0: 해수면 기압 (seaLevelPa_)
//    P:  현재 기압 (pressurePa)
//  정확도: ±1m (맑은 날, 안정된 기압 조건)
// ============================================================================
float Baro_BMP280::pressureToAltitude(float pressurePa) const
{
    return 44330.0f * (1.0f - powf(pressurePa / seaLevelPa_, 1.0f / 5.255f));
}

// ============================================================================
//  SPI 저수준 통신 함수
//  ─────────────────────────────────────────────────────────────────────────
//  BMP280 SPI 프로토콜:
//    읽기: CS=LOW → 주소(MSB=1) 전송 → 데이터 수신 → CS=HIGH
//    쓰기: CS=LOW → 주소(MSB=0) 전송 → 데이터 전송 → CS=HIGH
//
//  SPI Mode 0: CPOL=0(클럭 평소 LOW), CPHA=0(상승 엣지에서 샘플링)
// ============================================================================

// 1바이트 레지스터 쓰기
void Baro_BMP280::spiWrite(uint8_t reg, uint8_t val)
{
    spi_->beginTransaction(SPISettings(spiFreq_, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);          // CS 활성화
    spi_->transfer(BMP280_SPI_WRITE(reg));  // 레지스터 주소 (MSB=0)
    spi_->transfer(val);                    // 데이터
    digitalWrite(csPin_, HIGH);         // CS 비활성화
    spi_->endTransaction();
}

// 1바이트 레지스터 읽기
uint8_t Baro_BMP280::spiRead(uint8_t reg)
{
    spi_->beginTransaction(SPISettings(spiFreq_, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    spi_->transfer(BMP280_SPI_READ(reg));  // 레지스터 주소 (MSB=1)
    uint8_t val = spi_->transfer(0xFF);    // 더미 바이트 전송 → 데이터 수신
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    return val;
}

// n바이트 연속 읽기 (버스트 읽기, 한 번의 CS 활성화로 여러 바이트)
void Baro_BMP280::spiReadN(uint8_t reg, uint8_t n, uint8_t *buf)
{
    spi_->beginTransaction(SPISettings(spiFreq_, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    spi_->transfer(BMP280_SPI_READ(reg));  // 시작 주소 (MSB=1, 자동 증가)
    for (uint8_t i = 0; i < n; i++) {
        buf[i] = spi_->transfer(0xFF);     // 각 바이트 수신
    }
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
}
