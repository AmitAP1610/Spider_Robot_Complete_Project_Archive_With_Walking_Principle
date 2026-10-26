#include "WebRemote.h"
#include "GaitPlanner.h"
#include "ServoDriver.h"
#include "FaceController.h"

// DNS port for captive portal redirection
static const byte DNS_PORT = 53;

// Responsive touch-friendly mobile remote HTML/CSS/JS (embedded in PROGMEM)
static const char REMOTE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Spider Robot Mobile Remote</title>
<style>
:root{--bg:#0b0f19;--card:#161f30;--card-border:#26354d;--text:#f1f5f9;--sub:#94a3b8;--blue:#3b82f6;--green:#10b981;--red:#ef4444;--purple:#8b5cf6;--amber:#f59e0b}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;padding:12px;padding-bottom:30px;user-select:none}
.container{max-width:480px;margin:0 auto}
header{display:flex;align-items:center;justify-content:space-between;padding:10px 0 16px;border-bottom:1px solid var(--card-border)}
h1{font-size:1.15rem;font-weight:700;display:flex;align-items:center;gap:8px}
.badge{font-size:0.75rem;padding:4px 10px;border-radius:12px;font-weight:600;background:#1e293b;color:var(--green);border:1px solid rgba(16,185,129,0.3)}
.card{background:var(--card);border:1px solid var(--card-border);border-radius:16px;padding:14px;margin-top:14px;box-shadow:0 4px 12px rgba(0,0,0,0.25)}
.card-title{font-size:0.8rem;text-transform:uppercase;letter-spacing:0.06em;color:var(--sub);font-weight:700;margin-bottom:12px;display:flex;align-items:center;justify-content:space-between}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,64px);gap:8px;max-width:280px;margin:4px auto}
.btn{background:#1e293b;color:var(--text);border:1px solid var(--card-border);border-radius:12px;font-size:0.9rem;font-weight:700;display:flex;flex-direction:column;align-items:center;justify-content:center;cursor:pointer;transition:background 0.1s,transform 0.08s;touch-action:manipulation}
.btn:active{transform:scale(0.94);background:#334155}
.btn-blue{background:linear-gradient(135deg,#2563eb,#1d4ed8);border-color:#3b82f6;color:#fff}
.btn-red{background:linear-gradient(135deg,#dc2626,#b91c1c);border-color:#ef4444;color:#fff}
.btn-green{background:linear-gradient(135deg,#059669,#047857);border-color:#10b981;color:#fff}
.btn-purple{background:linear-gradient(135deg,#7c3aed,#6d28d9);border-color:#8b5cf6;color:#fff}
.btn-amber{background:linear-gradient(135deg,#d97706,#b45309);border-color:#f59e0b;color:#fff}
.grid-2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.grid-3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.grid-4{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.btn-sm{padding:10px 6px;font-size:0.75rem;min-height:48px}
.status-pill{font-size:0.8rem;color:var(--sub);text-align:center;padding:4px}
.face-active{border:2px solid var(--purple)!important;background:rgba(139,92,246,0.25)!important}
</style>
</head>
<body>
<div class="container">
  <header>
    <h1><span>🕷️</span> Spider Robot</h1>
    <span id="badge" class="badge">Online</span>
  </header>

  <!-- LOCOMOTION CONTROLS -->
  <div class="card">
    <div class="card-title">Locomotion D-Pad <span id="robotState" style="color:var(--blue)">STAND</span></div>
    <div class="dpad">
      <div></div>
      <button class="btn btn-blue" onclick="sendCmd('walk')">⬆️<br>WALK</button>
      <div></div>
      <button class="btn btn-blue" onclick="sendCmd('left')">⬅️<br>LEFT</button>
      <button class="btn btn-red" onclick="sendCmd('stop')">🛑<br>STOP</button>
      <button class="btn btn-blue" onclick="sendCmd('right')">➡️<br>RIGHT</button>
      <div></div>
      <button class="btn btn-blue" onclick="sendCmd('back')">⬇️<br>BACK</button>
      <div></div>
    </div>
  </div>

  <!-- POSTURE & SPEED -->
  <div class="card">
    <div class="card-title">Posture & Speed</div>
    <div class="grid-2" style="margin-bottom:8px">
      <button class="btn btn-green btn-sm" onclick="sendCmd('stand')">🧍 STAND</button>
      <button class="btn btn-amber btn-sm" onclick="sendCmd('sit')">🧘 SIT</button>
    </div>
    <div class="grid-2">
      <button class="btn btn-sm" onclick="sendCmd('slower')">🐢 SLOWER (-)</button>
      <button class="btn btn-sm" onclick="sendCmd('faster')">🐇 FASTER (+)</button>
    </div>
  </div>

  <!-- FACE EXPRESSIONS -->
  <div class="card">
    <div class="card-title">OLED Face Expression <span id="faceName" style="color:var(--purple)">idle</span></div>
    <div class="grid-3">
      <button class="btn btn-sm" onclick="sendFace('happy')">😊 Happy</button>
      <button class="btn btn-sm" onclick="sendFace('cute')">🥺 Cute</button>
      <button class="btn btn-sm" onclick="sendFace('angry')">😠 Angry</button>
      <button class="btn btn-sm" onclick="sendFace('love')">💖 Love</button>
      <button class="btn btn-sm" onclick="sendFace('sleepy')">😴 Sleepy</button>
      <button class="btn btn-sm" onclick="sendFace('surprised')">😲 Surprised</button>
      <button class="btn btn-sm" onclick="sendFace('walk')">🚶 Walk</button>
      <button class="btn btn-sm" onclick="sendFace('dead')">😵 Dead</button>
      <button class="btn btn-purple btn-sm" onclick="sendFace('auto')">🔄 Auto Face</button>
    </div>
  </div>

  <!-- HARDWARE SERVO DIAGNOSTICS -->
  <div class="card">
    <div class="card-title">Hardware Diagnostics & Servo Test</div>
    <button class="btn btn-sm btn-amber" style="width:100%;margin-bottom:8px" onclick="sendCmd('test_all')">
      ⚡ TEST ALL 12 SERVOS SEQUENTIALLY
    </button>
    <div class="grid-4">
      <button class="btn btn-sm" onclick="sendTestLeg(0)">FL (0,1,2)</button>
      <button class="btn btn-sm" onclick="sendTestLeg(1)">FR (4,5,6)</button>
      <button class="btn btn-sm" onclick="sendTestLeg(2)">BL (8,9,10)</button>
      <button class="btn btn-sm" onclick="sendTestLeg(3)">BR (12,13,14)</button>
    </div>
    <button class="btn btn-sm" style="width:100%;margin-top:8px" onclick="sendCmd('center')">
      🎯 Reset All Servos to Center (90°)
    </button>
  </div>

  <div class="status-pill" id="footerText">Connected to Spider-Robot Wi-Fi (192.168.4.1)</div>
</div>

<script>
function sendCmd(action){
  fetch('/cmd?action=' + action).then(r=>r.json()).then(updateUI).catch(e=>{});
}
function sendFace(name){
  fetch('/cmd?face=' + name).then(r=>r.json()).then(updateUI).catch(e=>{});
}
function sendTestLeg(leg){
  fetch('/cmd?action=test_leg&leg=' + leg).then(r=>r.json()).then(updateUI).catch(e=>{});
}
function updateUI(data){
  if(!data)return;
  if(data.state) document.getElementById('robotState').innerText = data.state;
  if(data.face) document.getElementById('faceName').innerText = data.face;
}
setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(updateUI).catch(e=>{});
}, 1200);
</script>
</body>
</html>
)rawliteral";

WebRemote::WebRemote()
    : m_server(80),
      m_gaitPlanner(nullptr),
      m_servoDriver(nullptr),
      m_faceController(nullptr),
      m_ssid("Spider-Robot"),
      m_password(""),
      m_initialized(false)
{
}

bool WebRemote::init(GaitPlanner* gaitPlanner,
                     ServoDriver* servoDriver,
                     FaceController* faceController,
                     const char* apSsid,
                     const char* apPassword) {
    m_gaitPlanner = gaitPlanner;
    m_servoDriver = servoDriver;
    m_faceController = faceController;
    m_ssid = apSsid;
    m_password = apPassword;

    // Configure ESP32 Wi-Fi in Access Point mode
    WiFi.mode(WIFI_AP);
    bool apOk = (m_password && strlen(m_password) >= 8) ?
                WiFi.softAP(m_ssid, m_password) :
                WiFi.softAP(m_ssid);

    if (!apOk) {
        Serial.println(">> [WIFI AP ERROR] Failed to start SoftAP!");
        return false;
    }

    IPAddress apIP = WiFi.softAPIP();
    Serial.println();
    Serial.println("==================================================================");
    Serial.println("            SPIDER ROBOT PHONE WEB REMOTE INITIALIZED             ");
    Serial.println("==================================================================");
    Serial.printf("  Wi-Fi Network (SSID) : %s\n", m_ssid);
    Serial.printf("  Password             : %s\n", (m_password && strlen(m_password)) ? m_password : "None (Open Network)");
    Serial.printf("  Remote Web URL       : http://%s\n", apIP.toString().c_str());
    Serial.println("  How to connect on phone:");
    Serial.println("    1) Open Wi-Fi on phone & connect to 'Spider-Robot'");
    Serial.println("    2) Tap sign-in notification or open browser to: 192.168.4.1");
    Serial.println("==================================================================");

    // Start DNS server on port 53 to redirect all domains to local IP (Captive Portal)
    m_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    m_dnsServer.start(DNS_PORT, "*", apIP);

    // Setup HTTP routes
    setupRoutes();
    m_server.begin();

    m_initialized = true;
    return true;
}

void WebRemote::setupRoutes() {
    m_server.on("/", HTTP_GET, [this]() { handleRoot(); });
    m_server.on("/cmd", HTTP_GET, [this]() { handleCommand(); });
    m_server.on("/status", HTTP_GET, [this]() { handleStatus(); });

    // Captive portal detection endpoints for iOS, Android, and Windows
    m_server.on("/generate_204", HTTP_GET, [this]() { handleRoot(); });      // Android
    m_server.on("/gen_204", HTTP_GET, [this]() { handleRoot(); });           // Android fallback
    m_server.on("/hotspot-detect.html", HTTP_GET, [this]() { handleRoot(); });// Apple iOS/macOS
    m_server.on("/canonical.html", HTTP_GET, [this]() { handleRoot(); });     // Firefox
    m_server.on("/connecttest.txt", HTTP_GET, [this]() { handleRoot(); });    // Windows

    m_server.onNotFound([this]() { handleNotFound(); });
}

void WebRemote::handleRoot() {
    m_server.send_P(200, "text/html", REMOTE_HTML);
}

void WebRemote::handleCommand() {
    // 1. Locomotion & Posture Commands
    if (m_server.hasArg("action")) {
        String action = m_server.arg("action");
        action.toLowerCase();

        if (action == "walk") {
            if (m_gaitPlanner) m_gaitPlanner->cmdWalkForward();
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("walk", FACE_ANIM_LOOP);
            }
        } else if (action == "back") {
            if (m_gaitPlanner) m_gaitPlanner->cmdWalkBackward();
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("surprised", FACE_ANIM_LOOP);
            }
        } else if (action == "left") {
            if (m_gaitPlanner) m_gaitPlanner->cmdTurnLeft();
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("point", FACE_ANIM_LOOP);
            }
        } else if (action == "right") {
            if (m_gaitPlanner) m_gaitPlanner->cmdTurnRight();
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("point", FACE_ANIM_LOOP);
            }
        } else if (action == "stop") {
            if (m_gaitPlanner) m_gaitPlanner->cmdStop();
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("idle", FACE_ANIM_LOOP);
            }
        } else if (action == "stand") {
            if (m_gaitPlanner) m_gaitPlanner->cmdStand();
            if (m_servoDriver) m_servoDriver->commandStandPoseSmooth(400);
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("idle", FACE_ANIM_LOOP);
            }
        } else if (action == "sit") {
            if (m_gaitPlanner) m_gaitPlanner->cmdSit();
            if (m_faceController && m_faceController->isAutoReaction()) {
                m_faceController->setFaceWithMode("rest", FACE_ANIM_BOOMERANG);
            }
        } else if (action == "faster") {
            if (m_gaitPlanner) m_gaitPlanner->cmdIncreaseSpeed();
        } else if (action == "slower") {
            if (m_gaitPlanner) m_gaitPlanner->cmdDecreaseSpeed();
        } else if (action == "test_all") {
            if (m_faceController) m_faceController->setFaceWithMode("thinking", FACE_ANIM_LOOP);
            if (m_servoDriver) m_servoDriver->runHardwareDiagnosticTest();
            if (m_faceController) m_faceController->setFaceWithMode("idle", FACE_ANIM_LOOP);
        } else if (action == "test_leg" && m_server.hasArg("leg")) {
            uint8_t leg = (uint8_t)m_server.arg("leg").toInt();
            if (leg < 4 && m_servoDriver) {
                // Test all 3 joints of selected leg
                uint8_t base = leg * 3;
                for (uint8_t j = 0; j < 3; j++) {
                    m_servoDriver->performJointSweepTest(base + j);
                }
            }
        } else if (action == "center") {
            if (m_servoDriver) m_servoDriver->commandStandPoseSmooth(300);
        }
    }

    // 2. Face Expression Commands
    if (m_server.hasArg("face")) {
        String face = m_server.arg("face");
        face.toLowerCase();
        if (m_faceController) {
            if (face == "auto") {
                m_faceController->setAutoReaction(true);
            } else {
                m_faceController->setFaceManual(face);
            }
        }
    }

    handleStatus();
}

void WebRemote::handleStatus() {
    String stateStr = "STAND";
    if (m_gaitPlanner) {
        switch (m_gaitPlanner->getState()) {
            case 0: stateStr = "STAND"; break;
            case 1: stateStr = "WALK FWD"; break;
            case 2: stateStr = "WALK BACK"; break;
            case 3: stateStr = "TURN LEFT"; break;
            case 4: stateStr = "TURN RIGHT"; break;
            case 5: stateStr = "SIT"; break;
            case 6: stateStr = "STOPPING"; break;
            case 7: stateStr = "CALIBRATE"; break;
        }
    }

    String faceStr = (m_faceController) ? m_faceController->getCurrentFace() : "none";

    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"state\":\"" + stateStr + "\",";
    json += "\"face\":\"" + faceStr + "\"";
    json += "}";

    m_server.send(200, "application/json", json);
}

void WebRemote::handleNotFound() {
    // Redirect unknown captive portal requests to root remote page
    m_server.sendHeader("Location", "/", true);
    m_server.send(302, "text/plain", "");
}

void WebRemote::update() {
    if (!m_initialized) return;
    m_dnsServer.processNextRequest();
    m_server.handleClient();
}

bool WebRemote::hasClients() {
    return WiFi.softAPgetStationNum() > 0;
}

String WebRemote::getIPAddress() const {
    return WiFi.softAPIP().toString();
}
