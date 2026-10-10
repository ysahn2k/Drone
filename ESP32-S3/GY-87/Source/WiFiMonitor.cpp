// ============================================================================
//  WiFiMonitor.cpp  —  QuadDrone ESP32-S3 WiFi PID 모니터링 대시보드
// ============================================================================
#include "DroneConfig.h"
#ifdef USE_WIFI_MONITOR

#include "WiFiMonitor.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>  // Arduino 라이브러리 매니저에서 "ArduinoJson" 설치 필요

// ── 전역 공유 데이터 ─────────────────────────────────────────────────────
WifiMonSnapshot g_wifiSnap   = {};
WifiPidCmd      g_wifiPidCmd = {};
volatile uint32_t g_wifiSnapSeq = 0;   // ★ seqlock 시퀀스 카운터 (짝수=안정)

// ── 내부 변수 ────────────────────────────────────────────────────────────
static WebServer server(80);

// ============================================================================
//  HTML 대시보드 (PROGMEM에 저장 — Flash 사용, RAM 절약)
// ============================================================================
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QuadDrone PID Monitor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;font-size:14px;padding:8px}
h1{color:#58a6ff;font-size:1.3em;margin-bottom:8px;border-bottom:1px solid #30363d;padding-bottom:4px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:6px}
.grid4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:4px;margin-bottom:6px}
.card{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:8px}
.lbl{color:#8b949e;font-size:.8em;margin-bottom:2px}
.val{font-size:1.6em;font-weight:bold;color:#58a6ff}
.val.neg{color:#f85149}
.val.pos{color:#3fb950}
#status{transition:background .3s}
.armed-bg{background:#2d1515!important;border-color:#f85149!important}
.disarmed-bg{background:#152d15!important;border-color:#3fb950!important}
.bar-wrap{background:#21262d;border-radius:3px;height:16px;overflow:hidden;margin:3px 0}
.bar{height:100%;background:#58a6ff;transition:width .15s;border-radius:3px}
.bar.hi{background:#f85149}
.motor-lbl{font-size:.75em;color:#8b949e}
.motor-val{font-size:.9em;color:#e6edf3;text-align:right}
/* PID 편집기 */
.pid-table{width:100%;border-collapse:collapse}
.pid-table td{padding:3px 4px;vertical-align:middle}
.pid-table td:first-child{color:#8b949e;width:110px;font-size:.85em}
input[type=number]{width:90px;background:#0d1117;color:#58a6ff;border:1px solid #30363d;
  border-radius:3px;padding:3px 5px;font-family:'Courier New',monospace;font-size:.95em}
input[type=number]:focus{outline:none;border-color:#58a6ff}
.btn{padding:6px 14px;border:none;border-radius:4px;cursor:pointer;font-weight:bold;font-size:.9em;margin-right:6px}
.btn-apply{background:#1f6feb;color:#fff}
.btn-apply:hover{background:#388bfd}
.btn-apply:active{background:#0d419d}
.msg{font-size:.8em;color:#3fb950;margin-top:4px;min-height:1.2em}
.badge{display:inline-block;padding:2px 8px;border-radius:10px;font-weight:bold;font-size:.85em}
.badge-armed{background:#f85149;color:#fff}
.badge-dis{background:#3fb950;color:#000}
.refresh{font-size:.75em;color:#8b949e;float:right}
</style>
</head>
<body>
<h1>&#x1F680; QuadDrone v8 &mdash; PID Monitor
  <span class="refresh" id="hz">0 Hz</span>
</h1>

<!-- 상태 표시 -->
<div class="card" id="status" style="margin-bottom:6px;display:flex;align-items:center;gap:12px;flex-wrap:wrap">
  <span id="armed_badge" class="badge badge-dis">DISARMED</span>
  <span class="lbl">배터리</span>
  <span id="battV" class="val" style="font-size:1.1em">--</span>
  <span style="color:#8b949e;font-size:.85em">V</span>
  <span class="lbl" style="margin-left:10px">최저</span>
  <span id="minBattV" style="font-size:.95em;color:#f0a500">--</span>
  <span style="color:#8b949e;font-size:.8em">V</span>
  <span class="lbl" style="margin-left:14px">가동시간</span>
  <span id="uptime" class="val" style="font-size:1.1em">--</span>
  <span id="resetWarn" style="color:#f85149;font-size:.85em;font-weight:bold;display:none">⚠ 재부팅 감지!</span>
  <span class="lbl" style="margin-left:14px">최근 부팅원인</span>
  <span id="rstReason" style="font-size:.85em;color:#8b949e">--</span>
</div>

<!-- 자세각 -->
<div class="grid2" style="margin-bottom:6px">
  <div>
    <div class="grid2">
      <div class="card"><div class="lbl">Roll</div><div id="roll" class="val">--</div></div>
      <div class="card"><div class="lbl">Pitch</div><div id="pitch" class="val">--</div></div>
    </div>
  </div>
  <div>
    <div class="grid2">
      <div class="card"><div class="lbl">Yaw</div><div id="yaw" class="val">--</div></div>
      <div class="card"><div class="lbl">Gyro X/Y/Z <span style="font-size:.7em">(°/s)</span></div>
        <div id="gyro" style="font-size:.9em;color:#e6edf3;margin-top:2px">--</div>
      </div>
    </div>
  </div>
</div>

<!-- 모터 출력 -->
<div class="card" style="margin-bottom:6px">
  <div class="lbl" style="margin-bottom:4px">Motors (DShot)</div>
  <div class="grid4">
    <div>
      <div class="motor-lbl">M1 RR</div>
      <div class="bar-wrap"><div id="b1" class="bar" style="width:0%"></div></div>
      <div class="motor-val" id="m1">-</div>
    </div>
    <div>
      <div class="motor-lbl">M2 FR</div>
      <div class="bar-wrap"><div id="b2" class="bar" style="width:0%"></div></div>
      <div class="motor-val" id="m2">-</div>
    </div>
    <div>
      <div class="motor-lbl">M3 RL</div>
      <div class="bar-wrap"><div id="b3" class="bar" style="width:0%"></div></div>
      <div class="motor-val" id="m3">-</div>
    </div>
    <div>
      <div class="motor-lbl">M4 FL</div>
      <div class="bar-wrap"><div id="b4" class="bar" style="width:0%"></div></div>
      <div class="motor-val" id="m4">-</div>
    </div>
  </div>
</div>

<!-- PID 편집기 -->
<div class="card">
  <div class="lbl" style="margin-bottom:6px;font-size:.9em">PID 파라미터 &nbsp;
    <span style="color:#8b949e;font-size:.8em">(Pitch = Roll 공통 적용)</span>
  </div>
  <table class="pid-table">
    <tr><td style="color:#58a6ff;font-weight:bold">Outer KP</td>
        <td><input type="number" id="p_outerKP" step="0.5" min="0" max="30" value="10.0"></td></tr>
    <tr><td style="color:#58a6ff">Outer KI</td>
        <td><input type="number" id="p_outerKI" step="0.005" min="0" max="1" value="0.020"></td></tr>
    <tr><td style="color:#58a6ff">Outer KD</td>
        <td><input type="number" id="p_outerKD" step="0.005" min="0" max="0.5" value="0.030"></td></tr>
    <tr><td colspan="2"><hr style="border-color:#30363d;margin:3px 0"></td></tr>
    <tr><td>Inner KP</td>
        <td><input type="number" id="p_innerKP" step="0.1" min="0" max="20" value="4.5"></td></tr>
    <tr><td>Inner KI</td>
        <td><input type="number" id="p_innerKI" step="0.005" min="0" max="1" value="0.020"></td></tr>
    <tr><td>Inner KD</td>
        <td><input type="number" id="p_innerKD" step="0.001" min="0" max="0.1" value="0.005"></td></tr>
    <tr><td>Yaw Inner KP</td>
        <td><input type="number" id="p_yawKP" step="0.1" min="0" max="20" value="4.5"></td></tr>
    <tr><td>Yaw Inner KI</td>
        <td><input type="number" id="p_yawKI" step="0.005" min="0" max="1" value="0.020"></td></tr>
  </table>
  <div style="margin-top:8px">
    <button class="btn btn-apply" onclick="applyPID()">&#x2714; 즉시 적용</button>
    <button class="btn" style="background:#21262d;color:#e6edf3" onclick="reloadPID()">&#x21ba; 현재값 불러오기</button>
  </div>
  <div id="msg" class="msg"></div>
</div>

<script>
var _cnt=0,_t0=Date.now(),_pidLoaded=false,_lastUptime=null;
function g(id){return document.getElementById(id)}
function fv(id){return parseFloat(g(id).value)||0}

function setVal(id,v,dec){
  var el=g(id);if(!el)return;
  var s=(v>=0?'+':'')+v.toFixed(dec||1)+'°';
  el.textContent=s;
  el.className='val'+(v<-2?' neg':v>2?' pos':'');
}
function bar(bId,vId,v){
  var pct=Math.max(0,Math.min(100,(v-100)/9));
  g(bId).style.width=pct+'%';
  g(bId).className='bar'+(pct>85?' hi':'');
  g(vId).textContent=v>0?v:'-';
}
function msg(t,ok){
  var el=g('msg');el.textContent=t;
  el.style.color=ok?'#3fb950':'#f85149';
}

var _reqSeq = 0;          // ★ 요청 시퀀스 번호 — 응답이 섞여 들어와도 최신 요청만 신뢰
var _resetMissCount = 0;  // ★ 재부팅 의심 연속 횟수 (디바운스 — 1회성 글리치 무시)

function poll(){
  var mySeq = ++_reqSeq;   // 이 요청의 고유 번호
  fetch('/data').then(function(r){return r.json();}).then(function(d){
    // ★ 중첩요청 가드: 내가 가장 최신 요청이 아니라면(즉 내가 보낸 뒤에
    //   이미 더 최신 요청이 나갔다면) 이 응답은 버린다. setInterval 대신
    //   self-chaining으로 바꾸면서 이 가드는 사실상 발생하지 않아야
    //   정상이지만, 만일을 대비한 이중 방어선이다.
    if (mySeq !== _reqSeq) { scheduleNextPoll(); return; }

    _cnt++;
    if(Date.now()-_t0>=1000){
      g('hz').textContent=(_cnt/(((Date.now()-_t0)/1000)).toFixed(0))+' Hz';
      _cnt=0;_t0=Date.now();
    }
    setVal('roll',d.roll);setVal('pitch',d.pitch);
    setVal('yaw',d.yaw);
    g('gyro').textContent=d.gx.toFixed(0)+' / '+d.gy.toFixed(0)+' / '+d.gz.toFixed(0);
    g('battV').textContent=d.batt.toFixed(1);
    g('minBattV').textContent=d.minBatt.toFixed(1);

    // ★ 부팅 직후 2초간은 데이터가 아직 안정화되지 않았을 수 있으므로
    //   모터 표시를 숨기고 "초기화 중" 안내만 표시 (혼란스러운 garbage 값 방지)
    var booting = d.uptime < 2000;
    if (booting) {
      for (var bi=1; bi<=4; bi++){
        g('b'+bi).style.width='0%';
        g('m'+bi).textContent='초기화중';
      }
    }

    // ★ 재부팅 감지: uptime이 이전 값보다 갑자기 작아지면 리셋 의심.
    //   다만 단발성 통신 글리치(응답 지연/역전)로도 같은 증상이 나올 수
    //   있으므로, 연속 2회 이상 감소가 확인될 때만 진짜 재부팅으로 판정한다
    //   (디바운스). 한 번이라도 정상(증가)으로 돌아오면 카운트 리셋.
    var sec=Math.floor(d.uptime/1000);
    var mm=Math.floor(sec/60), ss=sec%60;
    g('uptime').textContent=mm+'분'+ss+'초';
    if(_lastUptime!==null && d.uptime < _lastUptime - 500){
      _resetMissCount++;
      if (_resetMissCount >= 2) {
        g('resetWarn').style.display='inline';
        console.warn('재부팅 감지(연속 확인됨): uptime '+_lastUptime+'ms → '+d.uptime+'ms');
      } else {
        console.warn('uptime 감소 1회 관측 (통신 글리치 가능성, 재확인 대기): '+_lastUptime+'ms → '+d.uptime+'ms');
      }
    } else {
      _resetMissCount = 0;
    }
    _lastUptime=d.uptime;

    // ★ 재시작 원인 표시 — POWERON/SW_RESET 외에는 빨간색으로 강조
    if(d.rstReason){
      var rEl=g('rstReason');
      rEl.textContent=d.rstReason;
      var isAbnormal = d.rstReason.indexOf('★')>=0;
      rEl.style.color = isAbnormal ? '#f85149' : '#8b949e';
      rEl.style.fontWeight = isAbnormal ? 'bold' : 'normal';
    }
    var ab=g('armed_badge'),st=g('status');
    if(d.armed){
      ab.className='badge badge-armed';ab.textContent='ARMED';
      st.className='card armed-bg';
    } else {
      ab.className='badge badge-dis';ab.textContent='DISARMED';
      st.className='card disarmed-bg';
    }
    if (!booting) {
      bar('b1','m1',d.m1);bar('b2','m2',d.m2);
      bar('b3','m3',d.m3);bar('b4','m4',d.m4);
    }
    if(!_pidLoaded&&d.pid){
      g('p_outerKP').value=d.pid.outerKP.toFixed(2);
      g('p_outerKI').value=d.pid.outerKI.toFixed(4);
      g('p_outerKD').value=d.pid.outerKD.toFixed(4);
      g('p_innerKP').value=d.pid.innerKP.toFixed(3);
      g('p_innerKI').value=d.pid.innerKI.toFixed(4);
      g('p_innerKD').value=d.pid.innerKD.toFixed(4);
      g('p_yawKP').value=d.pid.yawKP.toFixed(3);
      g('p_yawKI').value=d.pid.yawKI.toFixed(4);
      _pidLoaded=true;
    }
    scheduleNextPoll();
  }).catch(function(){ scheduleNextPoll(); });
}

// ★ setInterval 대신 self-chaining 방식 사용: 이전 요청(응답 또는 실패)이
// 완전히 끝난 뒤에만 다음 요청을 보낸다. 기존 setInterval(poll,200)은
// 응답이 지연되더라도 무조건 200ms마다 새 요청을 추가로 쏘아, 여러 요청이
// 동시에 떠 있다가 응답이 보낸 순서와 다르게 도착(역전)할 수 있었다.
// 이 경우 "예전" 응답이 나중에 도착해 uptime이 실제로는 줄지 않았는데도
// 줄어든 것처럼 보여 가짜 재부팅 경고(false positive)가 뜨는 원인이 된다.
function scheduleNextPoll(){ setTimeout(poll, 200); }

function applyPID(){
  var body=JSON.stringify({
    outerKP:fv('p_outerKP'),
    outerKI:fv('p_outerKI'),
    outerKD:fv('p_outerKD'),
    innerKP:fv('p_innerKP'),
    innerKI:fv('p_innerKI'),
    innerKD:fv('p_innerKD'),
    yawKP:fv('p_yawKP'),
    yawKI:fv('p_yawKI')
  });
  fetch('/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:body})
    .then(function(r){return r.text();})
    .then(function(t){msg(t,true);})
    .catch(function(){msg('연결 오류',false);});
}

function reloadPID(){
  _pidLoaded=false;
  msg('현재값 불러오는 중...',true);
}

poll();   // ★ self-chaining 시작 (setInterval 대신 — 위 scheduleNextPoll 참고)
poll();
</script>
</body>
</html>
)rawhtml";

// ============================================================================
//  HTTP 핸들러
// ============================================================================
static void handleRoot() {
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/html", DASHBOARD_HTML);
}

static void handleData() {
    // ★ seqlock으로 안전하게 스냅샷 복사 (Core 0에서 읽기, torn-read 방지)
    WifiMonSnapshot s = wifiSnapReadSafe();

    // JSON 응답 생성
    StaticJsonDocument<512> doc;
    doc["roll"]  = roundf(s.roll  * 10) / 10.0f;
    doc["pitch"] = roundf(s.pitch * 10) / 10.0f;
    doc["yaw"]   = roundf(s.yaw   * 10) / 10.0f;
    doc["gx"]    = roundf(s.gx);
    doc["gy"]    = roundf(s.gy);
    doc["gz"]    = roundf(s.gz);
    doc["m1"] = s.m1;  doc["m2"] = s.m2;
    doc["m3"] = s.m3;  doc["m4"] = s.m4;
    doc["armed"] = s.armed;
    doc["batt"]  = roundf(s.battV * 10) / 10.0f;
    doc["minBatt"] = roundf(s.minBattV * 10) / 10.0f;   // ★ 부팅 후 최저 전압
    doc["uptime"] = s.uptimeMs;   // ★ 재부팅 감지용
    doc["rstReason"] = s.resetReason;  // ★ 마지막 부팅 원인

    JsonObject pid = doc.createNestedObject("pid");
    pid["outerKP"] = s.outerKP;
    pid["outerKI"] = s.outerKI;
    pid["outerKD"] = s.outerKD;
    pid["innerKP"] = s.innerKP;
    pid["innerKI"] = s.innerKI;
    pid["innerKD"] = s.innerKD;
    pid["yawKP"]   = s.yawKP;
    pid["yawKI"]   = s.yawKI;

    String resp;
    serializeJson(doc, resp);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", resp);
}

static void handleSetPID() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "No body");
        return;
    }
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "text/plain", "JSON parse error");
        return;
    }
    // PID 명령 설정 (Core 1에서 다음 루프에 적용)
    // ★ seqlock으로 안전하게 현재값 읽기 (기본값 폴백용)
    WifiMonSnapshot cur = wifiSnapReadSafe();
    g_wifiPidCmd.outerKP = doc["outerKP"] | cur.outerKP;
    g_wifiPidCmd.outerKI = doc["outerKI"] | cur.outerKI;
    g_wifiPidCmd.outerKD = doc["outerKD"] | cur.outerKD;
    g_wifiPidCmd.innerKP = doc["innerKP"] | cur.innerKP;
    g_wifiPidCmd.innerKI = doc["innerKI"] | cur.innerKI;
    g_wifiPidCmd.innerKD = doc["innerKD"] | cur.innerKD;
    g_wifiPidCmd.yawKP   = doc["yawKP"]   | cur.yawKP;
    g_wifiPidCmd.yawKI   = doc["yawKI"]   | cur.yawKI;
    g_wifiPidCmd.pending = true;

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain",
        "PID 적용됨: outerKP=" + String(g_wifiPidCmd.outerKP, 2) +
        " innerKP=" + String(g_wifiPidCmd.innerKP, 3));
}

// ============================================================================
//  FreeRTOS Task — Core 0에서 WebServer 처리 (비행 제어 루프와 분리)
// ============================================================================
static void wifiServerTask(void *pvParameters) {
    for (;;) {
        server.handleClient();
        vTaskDelay(1);   // 1ms yield — Core 0 다른 태스크에 양보
    }
}

// ============================================================================
//  initWiFiMonitor — setup()에서 1회 호출
// ============================================================================
void initWiFiMonitor() {

    // Access Point 생성 (비밀번호 없음: 연결 편의성)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_MONITOR_SSID);   // DroneConfig.h에서 정의
    IPAddress ip = WiFi.softAPIP();

    Serial.printf("[WiFi] AP 시작: SSID='%s'  IP=%s\n",
                  WIFI_MONITOR_SSID, ip.toString().c_str());
    Serial.printf("[WiFi] 브라우저에서 http://%s 접속\n", ip.toString().c_str());

    // HTTP 라우트 등록
    server.on("/",       HTTP_GET,  handleRoot);
    server.on("/data",   HTTP_GET,  handleData);
    server.on("/pid",    HTTP_POST, handleSetPID);
    server.on("/pid",    HTTP_OPTIONS, [](){   // CORS preflight
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        server.send(200, "text/plain", "");
    });
    server.begin();

    // Core 0에서 WebServer 태스크 실행 (스택 8KB)
    xTaskCreatePinnedToCore(
        wifiServerTask,    // 함수
        "WiFiServer",      // 태스크 이름
        8192,              // 스택 크기 (bytes)
        NULL,              // 파라미터
        1,                 // 우선순위
        NULL,              // 핸들 (불필요)
        0                  // Core 0 고정
    );
}

#endif // USE_WIFI_MONITOR
