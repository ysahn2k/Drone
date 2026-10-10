// ============================================================================
//  GPS_RTH.cpp  —  u-blox GPS 파서 + RTH 내비게이션 구현  (v6)
// ============================================================================
#include "GPS_RTH.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
//  생성자
// ============================================================================
GPS_RTH::GPS_RTH()
    : ser_(nullptr),
      homeLat_(0), homeLon_(0), homeAlt_(0), hasHome_(false),
      nmeaIdx_(0), inSentence_(false),
      rthState_(RTH_IDLE), rthTargetAlt_(0), rthTimer_(0)
{
    memset(nmeaBuf_, 0, sizeof(nmeaBuf_));
}

// ============================================================================
//  begin — GPS UART 초기화
// ============================================================================
void GPS_RTH::begin(HardwareSerial &serial, uint32_t baud)
{
    ser_ = &serial;
    serial.begin(baud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.printf("[GPS] UART 시작 (RX=%d TX=%d baud=%lu)\n",
                  PIN_GPS_RX, PIN_GPS_TX, baud);
    // ★ NVS 자동 로드 제거: 이전 위치(다른 장소)가 홈으로 설정되는 버그 방지
    // 홈은 반드시 Arming 시 GPS Fix 확인 후 저장 (INO에서 doArm() 시 처리)
}

// ============================================================================
//  update — UART 버퍼에서 NMEA 문장 파싱 (매 루프 호출)
//  ─────────────────────────────────────────────────────────────────────────
//  NMEA 문장 형식:
//  $GPRMC,094512.00,A,3729.12345,N,12700.67890,E,0.5,,280424,,,A*XX\r\n
//  │      │         │ │          │ │            │  │   │      │   체크섬
//  │      │         │ │          │ │            속도 방위 날짜
//  │      │         │ 위도       │ 경도
//  │      시간(UTC)  상태(A=유효)
//  문장 유형
// ============================================================================
void GPS_RTH::update()
{
    if (!ser_) return;

    while (ser_->available()) {
        char c = (char)ser_->read();

        if (c == '$') {
            // 새 문장 시작
            inSentence_ = true;
            nmeaIdx_    = 0;
            nmeaBuf_[nmeaIdx_++] = c;
        } else if (inSentence_) {
            if (c == '\n' || c == '\r') {
                // 문장 종료 → 파싱
                if (nmeaIdx_ > 0) {
                    nmeaBuf_[nmeaIdx_] = '\0';
                    if (verifyChecksum(nmeaBuf_)) {
                        parseNMEA(nmeaBuf_);
                    }
                }
                inSentence_ = false;
                nmeaIdx_    = 0;
            } else if (nmeaIdx_ < NMEA_MAX_LEN - 1) {
                nmeaBuf_[nmeaIdx_++] = c;
            } else {
                // 버퍼 오버플로 → 리셋
                inSentence_ = false;
                nmeaIdx_    = 0;
            }
        }
    }
}

// ============================================================================
//  parseNMEA — 문장 유형 판별 후 적절한 파서 호출
// ============================================================================
void GPS_RTH::parseNMEA(const char *s)
{
    // strncmp: 앞 6문자 비교 ($GPRMC, $GPGGA, $GPGSA)
    if      (strncmp(s, "$GPRMC", 6) == 0) parseGPRMC(s);
    else if (strncmp(s, "$GPGGA", 6) == 0) parseGPGGA(s);
    else if (strncmp(s, "$GPGSA", 6) == 0) parseGPGSA(s);
    // $GNRMC, $GNGGA (멀티 GNSS) 도 지원
    else if (strncmp(s, "$GNRMC", 6) == 0) parseGPRMC(s);
    else if (strncmp(s, "$GNGGA", 6) == 0) parseGPGGA(s);
}

// ============================================================================
//  parseGPRMC — RMC 문장 파싱 (핵심: 위도·경도·속도·방위·날짜)
//
//  형식: $GPRMC,HHMMSS.ss,A,DDMM.mmm,N,DDDMM.mmm,E,speed,course,DDMMYY,,,mode*CS
//  필드:         1         2  3        4  5         6  7     8      9
// ============================================================================
void GPS_RTH::parseGPRMC(const char *s)
{
    const char *p = s;

    p = nextField(p); // 1: 시간 HHMMSS.ss
    if (*p) {
        char tb[10]; strncpy(tb, p, 6); tb[6]='\0';
        data_.hour   = (tb[0]-'0')*10 + (tb[1]-'0');
        data_.minute = (tb[2]-'0')*10 + (tb[3]-'0');
        data_.second = (tb[4]-'0')*10 + (tb[5]-'0');
    }

    p = nextField(p); // 2: 상태 A=유효, V=무효
    bool active = (*p == 'A');

    p = nextField(p); // 3: 위도 DDMM.mmmmm
    const char *latStr = p;
    p = nextField(p); // 4: N/S
    const char *latDir = p;

    p = nextField(p); // 5: 경도 DDDMM.mmmmm
    const char *lonStr = p;
    p = nextField(p); // 6: E/W
    const char *lonDir = p;

    if (active && *latStr && *lonStr) {
        data_.lat = parseLatLon(latStr, latDir);
        data_.lon = parseLatLon(lonStr, lonDir);
    }

    p = nextField(p); // 7: 속도 [노트]
    if (*p) {
        data_.speedMps = parseFloat(p) * 0.5144f;  // 노트 → m/s
    }

    p = nextField(p); // 8: 방위각 [도]
    if (*p) data_.courseDeg = parseFloat(p);

    p = nextField(p); // 9: 날짜 DDMMYY
    if (*p && strlen(p) >= 6) {
        data_.day   = (p[0]-'0')*10 + (p[1]-'0');
        data_.month = (p[2]-'0')*10 + (p[3]-'0');
        data_.year  = 2000 + (p[4]-'0')*10 + (p[5]-'0');
    }

    if (active) data_.lastFixMs = millis();
    updateValidity();
}

// ============================================================================
//  parseGPGGA — GGA 문장 파싱 (위성수·HDOP·고도)
//
//  형식: $GPGGA,HHMMSS,lat,N,lon,E,quality,numSV,HDOP,alt,M,sep,M,,*CS
//  필드:         1      2  3  4  5  6       7     8    9   10
// ============================================================================
void GPS_RTH::parseGPGGA(const char *s)
{
    const char *p = s;
    p = nextField(p); // 1: 시간
    p = nextField(p); // 2: 위도 (RMC에서 이미 읽음)
    p = nextField(p); // 3: N/S
    p = nextField(p); // 4: 경도
    p = nextField(p); // 5: E/W
    p = nextField(p); // 6: Fix Quality
    if (*p) data_.fixQuality = parseInt(p);
    p = nextField(p); // 7: 위성 수
    if (*p) data_.numSats = parseInt(p);
    p = nextField(p); // 8: HDOP
    if (*p) data_.hdop = parseFloat(p);
    p = nextField(p); // 9: 고도 [m MSL]
    if (*p) data_.altMsl = parseFloat(p);

    updateValidity();
}

// ============================================================================
//  parseGPGSA — GSA 문장 파싱 (Fix 모드·HDOP)
// ============================================================================
void GPS_RTH::parseGPGSA(const char *s)
{
    const char *p = s;
    p = nextField(p); // 1: 선택 모드 (A/M)
    p = nextField(p); // 2: Fix 모드 (1=NoFix, 2=2D, 3=3D)
    int fixMode = (*p) ? parseInt(p) : 1;
    if (fixMode < 2) {
        data_.fixQuality = GPS_FIX_INVALID;
        updateValidity();
    }
    // 3~14: 위성 PRN (건너뜀)
    for (int i = 0; i < 12; i++) p = nextField(p);
    p = nextField(p); // 15: PDOP
    p = nextField(p); // 16: HDOP
    if (*p) data_.hdop = parseFloat(p);
}

// ============================================================================
//  updateValidity — Fix 유효성 판정
//  조건: Fix Quality > 0 AND 위성 수 >= GPS_MIN_SATS AND HDOP <= GPS_HDOP_MAX
// ============================================================================
void GPS_RTH::updateValidity()
{
    data_.valid = (data_.fixQuality  > GPS_FIX_INVALID) &&
                  (data_.numSats    >= GPS_MIN_SATS)    &&
                  (data_.hdop       <= GPS_HDOP_MAX)    &&
                  (data_.lat != 0.0 || data_.lon != 0.0);
}

// ============================================================================
//  saveHome — 현재 GPS 위치 + BMP180 기압 고도를 홈으로 저장
//  ★ 고도는 GPS MSL 대신 BMP180 기압 고도 사용 (상대 고도 정밀도 우수)
// ============================================================================
bool GPS_RTH::saveHome(float baroAltM)
{
    if (!data_.valid) {
        Serial.println("[GPS] Fix 없음 — 홈 저장 불가. 위성 수: " +
                       String(data_.numSats));
        return false;
    }
    homeLat_ = data_.lat;
    homeLon_ = data_.lon;
    homeAlt_ = baroAltM;   // ★ GPS MSL → BMP180 기압 고도 (매번 덮어씀)
    hasHome_ = true;

    // NVS에 저장 (전원 꺼도 유지)
    prefs_.begin(NVS_NAMESPACE, false);
    prefs_.putDouble(NVS_KEY_HOME_LAT, homeLat_);
    prefs_.putDouble(NVS_KEY_HOME_LON, homeLon_);
    prefs_.putFloat (NVS_KEY_HOME_ALT, homeAlt_);
    prefs_.end();

    Serial.printf("[GPS] 홈 저장: lat=%.6f lon=%.6f baroAlt=%.1fm sats=%d hdop=%.1f\n",
                  homeLat_, homeLon_, homeAlt_, data_.numSats, data_.hdop);
    return true;
}

bool GPS_RTH::loadHome()
{
    prefs_.begin(NVS_NAMESPACE, true);
    if (prefs_.isKey(NVS_KEY_HOME_LAT)) {
        homeLat_ = prefs_.getDouble(NVS_KEY_HOME_LAT, 0.0);
        homeLon_ = prefs_.getDouble(NVS_KEY_HOME_LON, 0.0);
        homeAlt_ = prefs_.getFloat (NVS_KEY_HOME_ALT, 0.0f);
        hasHome_ = (homeLat_ != 0.0 || homeLon_ != 0.0);
        if (hasHome_) {
            Serial.printf("[GPS] NVS에서 홈 위치 로드: lat=%.6f lon=%.6f\n",
                          homeLat_, homeLon_);
        }
    }
    prefs_.end();
    return hasHome_;
}

void GPS_RTH::setHome(double lat, double lon, float alt)
{
    homeLat_ = lat;  homeLon_ = lon;  homeAlt_ = alt;
    hasHome_ = true;
}

// ============================================================================
//  startRTH / stopRTH — RTH 시작·중단
// ============================================================================
void GPS_RTH::startRTH()
{
    if (rthState_ != RTH_IDLE) return;

    if (!hasHome_) {
        Serial.println("[RTH] 홈 위치 없음 — RTH 불가");
        rthState_ = RTH_ERROR;
        return;
    }
    if (!data_.valid) {
        Serial.println("[RTH] GPS Fix 없음 — RTH 불가");
        rthState_ = RTH_ERROR;
        return;
    }

    Serial.println("[RTH] 시작! 안전 고도까지 상승합니다...");
    rthTargetAlt_ = homeAlt_ + RTH_ASCEND_ALT_M;
    rthState_ = RTH_ASCENDING;
    rthTimer_ = millis();
}

void GPS_RTH::stopRTH()
{
    if (rthState_ != RTH_IDLE) {
        Serial.println("[RTH] 중단 — 수동 조종으로 복귀");
        rthState_ = RTH_IDLE;
    }
}

// ============================================================================
//  computeRTH — RTH 내비게이션 계산 (매 루프 호출)
//  ─────────────────────────────────────────────────────────────────────────
//  상태 머신:
//  ASCENDING  : 안전 고도(홈고도+10m)까지 상승
//  NAVIGATING : 홈 방향으로 이동 (Pitch로 전진, Yaw로 방향 유지)
//  DESCENDING : 홈 위에서 수직 하강
//  LANDED     : 착륙 완료 → 자동 시동 해제
// ============================================================================
NavCommand GPS_RTH::computeRTH(float curRoll, float curPitch,
                                float curYaw,  float curAltBaro,
                                float dt)
{
    NavCommand cmd;
    cmd.state = rthState_;

    if (rthState_ == RTH_IDLE || rthState_ == RTH_ERROR ||
        rthState_ == RTH_LANDED) {
        return cmd;
    }

    if (!data_.valid && rthState_ != RTH_DESCENDING) {
        cmd.throttleUs = 1500.0f;
        return cmd;
    }

    // ★ 고도: 항상 BMP180 기압 고도 사용
    // homeAlt_도 arming 시 BMP180으로 저장 → 동일 기준 → 정확한 상대 고도
    float curAlt = curAltBaro;

    float distToHome = calcDistanceM(data_.lat, data_.lon, homeLat_, homeLon_);
    float bearToHome = calcBearingDeg(data_.lat, data_.lon, homeLat_, homeLon_);

    // ★ RTH 진행 상황 시리얼 출력 (2초마다)
    static uint32_t rthLogMs = 0;
    if (millis() - rthLogMs >= 2000) {
        rthLogMs = millis();
        Serial.printf("[RTH] 상태=%d 고도=%.1fm/목표=%.1fm 거리=%.1fm 방위=%.0f°\n",
                      rthState_, curAlt, rthTargetAlt_, distToHome, bearToHome);
        Serial.printf("[RTH] 현재위치: lat=%.6f lon=%.6f\n",
                      data_.lat, data_.lon);
    }

    switch (rthState_) {

    // ── 1단계: 안전 고도까지 상승 ──
    case RTH_ASCENDING:
        cmd.throttleUs = 1650.0f;  // 상승 스로틀 (튜닝 필요)
        cmd.rollCmd    = 0.0f;
        cmd.pitchCmd   = 0.0f;
        if (curAlt >= rthTargetAlt_ - 0.5f) {
            Serial.printf("[RTH] 안전 고도 도달 %.1fm — 홈으로 이동\n", curAlt);
            rthState_ = RTH_NAVIGATING;
        }
        break;

    // ── 2단계: 홈 방향으로 수평 이동 ──
    case RTH_NAVIGATING:
        // Yaw: 홈 방향으로 기체 회전
        {
            float yawErr = normalizeBearing(bearToHome - curYaw);
            float yawCmd = yawErr * 2.0f;  // 비례 제어 (튜닝 가능)
            if (yawCmd >  MAX_RATE_YAW_DPS) yawCmd =  MAX_RATE_YAW_DPS;
            if (yawCmd < -MAX_RATE_YAW_DPS) yawCmd = -MAX_RATE_YAW_DPS;
            cmd.yawCmd = yawCmd;

            // Pitch: 앞으로 전진 (최대 15도 기울기)
            float targetPitch = -12.0f;  // 음수=앞으로 기울기
            if (distToHome < 5.0f) targetPitch = 0.0f;  // 가까우면 감속
            cmd.pitchCmd = targetPitch;
            cmd.rollCmd  = 0.0f;

            // 고도 유지 (기압계 기반)
            float altErr     = rthTargetAlt_ - curAlt;
            float altAdj     = altErr * 30.0f;  // 스로틀 보정량
            cmd.throttleUs   = 1500.0f + altAdj;
            if (cmd.throttleUs < 1200.0f) cmd.throttleUs = 1200.0f;
            if (cmd.throttleUs > 1800.0f) cmd.throttleUs = 1800.0f;

            // 홈 도달 확인
            if (distToHome <= RTH_RADIUS_M) {
                Serial.printf("[RTH] 홈 도달 (거리=%.1fm) — 착륙 시작\n",
                              distToHome);
                rthState_ = RTH_DESCENDING;
            }
        }
        break;

    // ── 3단계: 수직 하강 ──
    case RTH_DESCENDING:
        cmd.rollCmd  = 0.0f;
        cmd.pitchCmd = 0.0f;
        cmd.yawCmd   = 0.0f;
        // 점진적 스로틀 감소
        cmd.throttleUs = 1350.0f;  // 하강 스로틀 (튜닝 필요)
        if (curAlt <= 0.3f) {
            Serial.println("[RTH] 착륙 완료!");
            rthState_ = RTH_LANDED;
        }
        break;

    default:
        break;
    }

    cmd.state = rthState_;
    return cmd;
}

// ============================================================================
//  calcDistanceM — Haversine 공식으로 두 좌표 사이 거리 계산 [m]
//  ─────────────────────────────────────────────────────────────────────────
//  Haversine 공식: 구면 위의 두 점 사이 최단 거리 (대원 거리)
//  지구를 완전한 구(반지름 6,371 km)로 가정합니다.
// ============================================================================
float GPS_RTH::calcDistanceM(double lat1, double lon1,
                              double lat2, double lon2)
{
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a    = sin(dLat/2) * sin(dLat/2) +
                  cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
                  sin(dLon/2) * sin(dLon/2);
    double c    = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(EARTH_RADIUS_M * c);
}

// ============================================================================
//  calcBearingDeg — 좌표1에서 좌표2 방향 계산 [도]
//  반환: 0=북, 90=동, 180=남, 270=서 (진북 기준)
// ============================================================================
float GPS_RTH::calcBearingDeg(double lat1, double lon1,
                               double lat2, double lon2)
{
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double lat1R = lat1 * DEG_TO_RAD;
    double lat2R = lat2 * DEG_TO_RAD;
    double y = sin(dLon) * cos(lat2R);
    double x = cos(lat1R) * sin(lat2R) - sin(lat1R) * cos(lat2R) * cos(dLon);
    float  bearing = (float)(atan2(y, x) * RAD_TO_DEG);
    if (bearing < 0) bearing += 360.0f;
    return bearing;
}

// 방위각 차이 정규화: -180 ~ +180 범위로 변환
float GPS_RTH::normalizeBearing(float b)
{
    while (b >  180.0f) b -= 360.0f;
    while (b < -180.0f) b += 360.0f;
    return b;
}

// ============================================================================
//  fillCRSF_GPS — ELRS 텔레메트리용 GPS 페이로드 생성
//  CRSF GPS 프레임 타입 0x02 형식
// ============================================================================
void GPS_RTH::fillCRSF_GPS(uint8_t *payload, uint8_t &len) const
{
    // 위도·경도: 1e7 단위 정수 (예: 37.123456 → 371234560)
    int32_t lat_ie7 = (int32_t)(data_.lat * 1e7);
    int32_t lon_ie7 = (int32_t)(data_.lon * 1e7);
    // 대지 속도: cm/s
    uint16_t speed_cms = (uint16_t)(data_.speedMps * 100.0f);
    // 방위각: 도 × 100
    uint16_t course_100 = (uint16_t)(data_.courseDeg * 100.0f);
    // 고도: m + 1000 오프셋 (MSL, 0m 기준)
    uint16_t alt_m = (uint16_t)(data_.altMsl + 1000.0f);
    // 위성 수
    uint8_t  sats = data_.numSats;

    uint8_t i = 0;
    payload[i++] = (lat_ie7    >> 24) & 0xFF;
    payload[i++] = (lat_ie7    >> 16) & 0xFF;
    payload[i++] = (lat_ie7    >>  8) & 0xFF;
    payload[i++] =  lat_ie7           & 0xFF;
    payload[i++] = (lon_ie7    >> 24) & 0xFF;
    payload[i++] = (lon_ie7    >> 16) & 0xFF;
    payload[i++] = (lon_ie7    >>  8) & 0xFF;
    payload[i++] =  lon_ie7           & 0xFF;
    payload[i++] = (speed_cms  >>  8) & 0xFF;
    payload[i++] =  speed_cms         & 0xFF;
    payload[i++] = (course_100 >>  8) & 0xFF;
    payload[i++] =  course_100        & 0xFF;
    payload[i++] = (alt_m      >>  8) & 0xFF;
    payload[i++] =  alt_m             & 0xFF;
    payload[i++] = sats;
    len = i;  // 15바이트
}

// ============================================================================
//  NMEA 유틸리티 함수들
// ============================================================================

// XOR 체크섬 검증 ($와 * 사이 모든 문자의 XOR)
bool GPS_RTH::verifyChecksum(const char *s)
{
    const char *p = s + 1;  // '$' 건너뜀
    uint8_t calc = 0;
    while (*p && *p != '*') calc ^= (uint8_t)*p++;
    if (*p != '*') return false;
    uint8_t recv = (uint8_t)strtol(p + 1, nullptr, 16);
    return calc == recv;
}

// DDMM.mmmmm (도분) + 방향 → 십진도 변환
double GPS_RTH::parseLatLon(const char *field, const char *dir)
{
    if (!field || !*field) return 0.0;
    double val   = atof(field);
    int    deg   = (int)(val / 100);
    double min   = val - deg * 100.0;
    double result= deg + min / 60.0;
    if (dir && (*dir == 'S' || *dir == 'W')) result = -result;
    return result;
}

float GPS_RTH::parseFloat(const char *f) {
    return f && *f ? (float)atof(f) : 0.0f;
}
int GPS_RTH::parseInt(const char *f) {
    return f && *f ? atoi(f) : 0;
}

// 다음 쉼표 필드 포인터 반환
const char* GPS_RTH::nextField(const char *p)
{
    while (*p && *p != ',') p++;
    if (*p == ',') p++;
    return p;
}
