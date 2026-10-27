#include "FaceController.h"
#include "face-bitmaps.h"

// Frame arrays for each face defined in FACE_LIST
#define MAKE_FACE_FRAMES(name) \
    const unsigned char* const face_##name##_frames[] = { \
        epd_bitmap_##name, epd_bitmap_##name##_1, epd_bitmap_##name##_2, \
        epd_bitmap_##name##_3, epd_bitmap_##name##_4, epd_bitmap_##name##_5 \
    };

#define X(name) MAKE_FACE_FRAMES(name)
FACE_LIST
#undef X
#undef MAKE_FACE_FRAMES

static const uint8_t MAX_FACE_FRAMES = 6;

struct FaceEntry {
    const char* name;
    const unsigned char* const* frames;
    uint8_t maxFrames;
};

static const FaceEntry faceEntries[] = {
#define X(name) { #name, face_##name##_frames, MAX_FACE_FRAMES },
    FACE_LIST
#undef X
    { "default", face_defualt_frames, MAX_FACE_FRAMES }
};

struct FaceFpsEntry {
    const char* name;
    uint8_t fps;
};

static const FaceFpsEntry faceFpsEntries[] = {
    { "walk", 2 },
    { "rest", 1 },
    { "swim", 2 },
    { "dance", 4 },
    { "wave", 3 },
    { "point", 5 },
    { "stand", 1 },
    { "cute", 2 },
    { "pushup", 2 },
    { "freaky", 2 },
    { "bow", 2 },
    { "worm", 2 },
    { "shake", 4 },
    { "shrug", 2 },
    { "dead", 2 },
    { "crab", 2 },
    { "idle", 1 },
    { "idle_blink", 7 },
    { "default", 1 },
    { "defualt", 1 },
    { "happy", 2 },
    { "talk_happy", 4 },
    { "sad", 2 },
    { "talk_sad", 4 },
    { "angry", 2 },
    { "talk_angry", 4 },
    { "surprised", 2 },
    { "talk_surprised", 4 },
    { "sleepy", 1 },
    { "talk_sleepy", 3 },
    { "love", 2 },
    { "talk_love", 4 },
    { "excited", 3 },
    { "talk_excited", 5 },
    { "confused", 2 },
    { "talk_confused", 4 },
    { "thinking", 2 },
    { "talk_thinking", 4 }
};

static uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames) {
    if (frames == nullptr) return 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < maxFrames; i++) {
        if (frames[i] != nullptr) count++;
        else break;
    }
    return count;
}

FaceController::FaceController()
    : m_display(128, 64, &Wire, -1),
      m_i2cMutex(nullptr),
      m_taskHandle(nullptr),
      m_i2cAddr(0x3C),
      m_sdaPin(21),
      m_sclPin(22),
      m_connected(false),
      m_autoReaction(true),
      m_currentFaceName("rest"),
      m_currentFaceFrames(nullptr),
      m_currentFaceFrameCount(0),
      m_currentFaceFrameIndex(0),
      m_currentFaceMode(FACE_ANIM_LOOP),
      m_faceFrameDirection(1),
      m_faceAnimFinished(false),
      m_faceFps(2),
      m_lastFrameTimeMs(0),
      m_nextIdleBlinkMs(0),
      m_inIdleBlink(false)
{
}

bool FaceController::init(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddr,
                          SemaphoreHandle_t i2cMutex) {
    m_sdaPin = sdaPin;
    m_sclPin = sclPin;
    m_i2cAddr = i2cAddr;
    m_i2cMutex = i2cMutex;

    // Check if I2C bus device responds at OLED address (0x3C or 0x3D)
    bool found = false;
    for (uint8_t addr : { (uint8_t)0x3C, (uint8_t)0x3D, m_i2cAddr }) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            m_i2cAddr = addr;
            found = true;
            break;
        }
    }

    if (!found) {
        m_connected = false;
        Serial.printf("\n>> [OLED NOTICE] SSD1306 OLED display not detected at address 0x%02X (SDA=%u, SCL=%u).\n",
                      m_i2cAddr, m_sdaPin, m_sclPin);
        Serial.println("   (Robot will operate in headless walking mode. Connect SSD1306 to enable facial expressions.)");
        return false;
    }

    if (!m_display.begin(SSD1306_SWITCHCAPVCC, m_i2cAddr)) {
        m_connected = false;
        Serial.println(">> [OLED ERROR] SSD1306 allocation failed (low RAM).");
        return false;
    }

    m_connected = true;
    m_display.clearDisplay();
    m_display.setTextColor(SSD1306_WHITE);

    // Boot splash screen
    showStartupScreen("SPIDER ROBOT", "12-Servo Controller");
    delay(300);

    // Initial neutral face
    setFaceWithMode("rest", FACE_ANIM_BOOMERANG);

    Serial.printf(">> [OLED READY] SSD1306 display initialized at I2C address 0x%02X.\n", m_i2cAddr);
    return true;
}

bool FaceController::startBackgroundTask(uint8_t core, UBaseType_t priority) {
    if (!m_connected) {
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        taskTrampoline,
        "FaceAnimTask",
        4096,
        this,
        priority,
        &m_taskHandle,
        core
    );

    if (result == pdPASS) {
        Serial.printf(">> [OLED TASK] Face animation engine pinned to Core %u (Priority %u).\n",
                      core, priority);
        return true;
    } else {
        Serial.println(">> [OLED TASK ERROR] Failed to create background animation task!");
        return false;
    }
}

void FaceController::taskTrampoline(void* param) {
    FaceController* self = static_cast<FaceController*>(param);
    for (;;) {
        self->update();
        vTaskDelay(pdMS_TO_TICKS(35)); // ~28 Hz task cycle
    }
}

void FaceController::update() {
    if (!m_connected) return;

    // Handle natural eye blinking when in idle face
    updateIdleBlink();

    if (m_currentFaceFrames == nullptr || m_currentFaceFrameCount <= 1) {
        return;
    }
    if (m_currentFaceMode == FACE_ANIM_ONCE && m_faceAnimFinished) {
        return;
    }

    uint32_t now = millis();
    uint8_t fps = m_faceFps > 0 ? m_faceFps : 2;
    uint32_t interval = 1000 / fps;

    if (now - m_lastFrameTimeMs >= interval) {
        m_lastFrameTimeMs = now;

        if (m_currentFaceMode == FACE_ANIM_LOOP) {
            m_currentFaceFrameIndex = (m_currentFaceFrameIndex + 1) % m_currentFaceFrameCount;
        } else if (m_currentFaceMode == FACE_ANIM_ONCE) {
            if (m_currentFaceFrameIndex + 1 >= m_currentFaceFrameCount) {
                m_currentFaceFrameIndex = m_currentFaceFrameCount - 1;
                m_faceAnimFinished = true;
            } else {
                m_currentFaceFrameIndex++;
            }
        } else if (m_currentFaceMode == FACE_ANIM_BOOMERANG) {
            if (m_faceFrameDirection > 0) {
                if (m_currentFaceFrameIndex + 1 >= m_currentFaceFrameCount) {
                    m_faceFrameDirection = -1;
                    if (m_currentFaceFrameIndex > 0) m_currentFaceFrameIndex--;
                } else {
                    m_currentFaceFrameIndex++;
                }
            } else {
                if (m_currentFaceFrameIndex == 0) {
                    m_faceFrameDirection = 1;
                    if (m_currentFaceFrameCount > 1) m_currentFaceFrameIndex++;
                } else {
                    m_currentFaceFrameIndex--;
                }
            }
        }

        renderBitmap(m_currentFaceFrames[m_currentFaceFrameIndex]);
    }
}

void FaceController::setFaceWithMode(const String& faceName, FaceAnimMode mode) {
    m_currentFaceMode = mode;
    m_faceFrameDirection = 1;
    m_faceAnimFinished = false;
    setFace(faceName);
}

void FaceController::setFace(const String& faceName) {
    if (faceName == m_currentFaceName && m_currentFaceFrames != nullptr) {
        return;
    }

    m_currentFaceName = faceName;
    m_currentFaceFrameIndex = 0;
    m_lastFrameTimeMs = 0;
    m_faceFrameDirection = 1;
    m_faceAnimFinished = false;
    m_faceFps = getFaceFpsForName(faceName);

    m_currentFaceFrames = face_defualt_frames;
    m_currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);

    size_t numEntries = sizeof(faceEntries) / sizeof(faceEntries[0]);
    for (size_t i = 0; i < numEntries; i++) {
        if (faceName.equalsIgnoreCase(faceEntries[i].name)) {
            m_currentFaceFrames = faceEntries[i].frames;
            m_currentFaceFrameCount = countFrames(faceEntries[i].frames, faceEntries[i].maxFrames);
            break;
        }
    }

    if (m_currentFaceFrameCount == 0) {
        m_currentFaceFrames = face_defualt_frames;
        m_currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
        m_currentFaceName = "default";
        m_faceFps = getFaceFpsForName(m_currentFaceName);
    }

    if (m_currentFaceFrameCount > 0 && m_currentFaceFrames[0] != nullptr) {
        renderBitmap(m_currentFaceFrames[0]);
    }
}

void FaceController::setFaceManual(const String& faceName) {
    m_autoReaction = false; // Disable automatic motion reaction until user commands 'face auto'
    setFaceWithMode(faceName, FACE_ANIM_LOOP);
    Serial.printf(">> Face set manually to: '%s' (Auto reaction paused. Send 'FACE AUTO' to re-enable)\n",
                  m_currentFaceName.c_str());
}

void FaceController::onRobotStateChange(uint8_t robotState, bool inCalibration) {
    if (!m_autoReaction) return;

    if (inCalibration) {
        if (m_currentFaceName != "confused") {
            setFaceWithMode("confused", FACE_ANIM_LOOP);
        }
        return;
    }

    switch (robotState) {
        case 0: // STATE_STAND
            if (m_currentFaceName != "idle" && m_currentFaceName != "idle_blink") {
                m_inIdleBlink = false;
                m_nextIdleBlinkMs = millis() + 2000;
                setFaceWithMode("idle", FACE_ANIM_LOOP);
            }
            break;

        case 1: // STATE_WALK_FORWARD
            if (m_currentFaceName != "walk") {
                setFaceWithMode("walk", FACE_ANIM_LOOP);
            }
            break;

        case 2: // STATE_WALK_BACKWARD
            if (m_currentFaceName != "surprised") {
                setFaceWithMode("surprised", FACE_ANIM_LOOP);
            }
            break;

        case 3: // STATE_TURN_LEFT
        case 4: // STATE_TURN_RIGHT
            if (m_currentFaceName != "point") {
                setFaceWithMode("point", FACE_ANIM_LOOP);
            }
            break;

        case 5: // STATE_SIT
            if (m_currentFaceName != "rest") {
                setFaceWithMode("rest", FACE_ANIM_BOOMERANG);
            }
            break;

        case 6: // STATE_STOPPING
            if (m_currentFaceName != "thinking") {
                setFaceWithMode("thinking", FACE_ANIM_LOOP);
            }
            break;

        default:
            break;
    }
}

void FaceController::updateIdleBlink() {
    if (m_currentFaceName != "idle" && m_currentFaceName != "idle_blink") {
        return;
    }

    uint32_t now = millis();
    if (!m_inIdleBlink) {
        if (m_nextIdleBlinkMs == 0) {
            m_nextIdleBlinkMs = now + 2500 + (esp_random() % 3000);
        }
        if (now >= m_nextIdleBlinkMs) {
            m_inIdleBlink = true;
            setFaceWithMode("idle_blink", FACE_ANIM_ONCE);
        }
    } else {
        if (m_faceAnimFinished) {
            m_inIdleBlink = false;
            setFaceWithMode("idle", FACE_ANIM_LOOP);
            m_nextIdleBlinkMs = now + 2500 + (esp_random() % 3500);
        }
    }
}

void FaceController::renderBitmap(const unsigned char* bitmap) {
    if (!m_connected || bitmap == nullptr) return;

    // FreeRTOS Mutex protection for shared I2C bus with PCA9685
    if (m_i2cMutex != nullptr) {
        if (xSemaphoreTake(m_i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return; // Skip this visual frame if servo bus is active to avoid stalling gait engine
        }
    }

    m_display.clearDisplay();
    m_display.drawBitmap(0, 0, bitmap, 128, 64, SSD1306_WHITE);
    m_display.display();

    if (m_i2cMutex != nullptr) {
        xSemaphoreGive(m_i2cMutex);
    }
}

int FaceController::getFaceFpsForName(const String& name) {
    size_t numFps = sizeof(faceFpsEntries) / sizeof(faceFpsEntries[0]);
    for (size_t i = 0; i < numFps; i++) {
        if (name.equalsIgnoreCase(faceFpsEntries[i].name)) {
            return faceFpsEntries[i].fps;
        }
    }
    return 2;
}

void FaceController::showStartupScreen(const char* title, const char* subtitle) {
    if (!m_connected) return;

    if (m_i2cMutex != nullptr) {
        xSemaphoreTake(m_i2cMutex, portMAX_DELAY);
    }

    m_display.clearDisplay();
    m_display.setTextSize(1);
    m_display.setTextColor(SSD1306_WHITE);
    m_display.setCursor(20, 20);
    m_display.println(title);
    m_display.setCursor(8, 36);
    m_display.println(subtitle);
    m_display.display();

    if (m_i2cMutex != nullptr) {
        xSemaphoreGive(m_i2cMutex);
    }
}

void FaceController::printAvailableFaces() {
    Serial.println();
    Serial.println("==================================================================");
    Serial.println("                 AVAILABLE OLED FACE EXPRESSIONS                  ");
    Serial.println("==================================================================");
    Serial.println(" Usage: Send 'FACE <name>' (e.g. 'FACE CUTE' or 'FACE HAPPY')");
    Serial.println(" Auto Mode: Send 'FACE AUTO' to restore movement reaction");
    Serial.println("------------------------------------------------------------------");

    size_t count = sizeof(faceEntries) / sizeof(faceEntries[0]);
    for (size_t i = 0; i < count; i++) {
        char buf[32];
        snprintf(buf, sizeof(buf), "  %-14s", faceEntries[i].name);
        Serial.print(buf);
        if ((i + 1) % 4 == 0) Serial.println();
    }
    Serial.println();
    Serial.println("==================================================================");
}
