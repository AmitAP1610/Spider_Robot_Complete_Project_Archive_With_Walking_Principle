#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/**
 * ============================================================================
 * FACE CONTROLLER MODULE (SSD1306 128x64 OLED)
 * ============================================================================
 * Handles facial expression rendering, multi-frame animation playback,
 * natural eye blinking, motion auto-synchronization, and thread-safe FreeRTOS
 * background rendering on ESP32 Core 0.
 * ============================================================================
 */

enum FaceAnimMode : uint8_t {
    FACE_ANIM_LOOP = 0,       // Loop continuously through all frames
    FACE_ANIM_ONCE = 1,       // Play once and freeze on last frame
    FACE_ANIM_BOOMERANG = 2   // Play forward then reverse (1-2-3-2-1)
};

class FaceController {
public:
    FaceController();

    /**
     * Initialize OLED SSD1306 display on I2C bus.
     * @param sdaPin I2C SDA pin (default 21)
     * @param sclPin I2C SCL pin (default 22)
     * @param i2cAddr OLED I2C address (default 0x3C, auto-probes 0x3D)
     * @param i2cMutex Shared FreeRTOS mutex for bus arbitration with PCA9685
     * @return true if SSD1306 display initialized, false otherwise.
     */
    bool init(uint8_t sdaPin = 21, uint8_t sclPin = 22, uint8_t i2cAddr = 0x3C,
              SemaphoreHandle_t i2cMutex = nullptr);

    /**
     * Start background FreeRTOS animation task pinned to Core 0.
     * Keeps Core 1 100% dedicated to walking control loop without jitter.
     * @param core CPU core (default 0)
     * @param priority Task priority (default 1)
     * @return true if task spawned successfully.
     */
    bool startBackgroundTask(uint8_t core = 0, UBaseType_t priority = 1);

    /**
     * Advance animation frame (called by background task or manual loop).
     */
    void update();

    /**
     * Set active face expression by name.
     */
    void setFace(const String& faceName);

    /**
     * Set active face expression with specific animation mode.
     */
    void setFaceWithMode(const String& faceName, FaceAnimMode mode);

    /**
     * Manual expression override from Serial Monitor or API.
     * Disables auto-reaction until 'face auto' is commanded.
     */
    void setFaceManual(const String& faceName);

    /**
     * Enable or disable automatic reaction to robot locomotion.
     */
    void setAutoReaction(bool enable) { m_autoReaction = enable; }
    bool isAutoReaction() const { return m_autoReaction; }

    /**
     * Synchronize face with robot movement state (Walk, Stand, Turn, Calibration, etc.)
     */
    void onRobotStateChange(uint8_t robotState, bool inCalibration);

    /**
     * Print all available expressions to Serial Monitor.
     */
    void printAvailableFaces();

    /**
     * Check if display hardware is detected and running.
     */
    bool isConnected() const { return m_connected; }

    /**
     * Get current face name.
     */
    String getCurrentFace() const { return m_currentFaceName; }

    /**
     * Set FPS override for animations.
     */
    void setFps(uint8_t fps) { m_faceFps = max((uint8_t)1, fps); }
    uint8_t getFps() const { return m_faceFps; }

    /**
     * Display a startup splash screen message.
     */
    void showStartupScreen(const char* title, const char* subtitle);

private:
    void renderBitmap(const unsigned char* bitmap);
    int getFaceFpsForName(const String& name);
    void updateIdleBlink();

    static void taskTrampoline(void* param);

    Adafruit_SSD1306 m_display;
    SemaphoreHandle_t m_i2cMutex;
    TaskHandle_t m_taskHandle;

    uint8_t m_i2cAddr;
    uint8_t m_sdaPin;
    uint8_t m_sclPin;
    bool m_connected;
    bool m_autoReaction;

    String m_currentFaceName;
    const unsigned char* const* m_currentFaceFrames;
    uint8_t m_currentFaceFrameCount;
    uint8_t m_currentFaceFrameIndex;
    FaceAnimMode m_currentFaceMode;
    int8_t m_faceFrameDirection;
    bool m_faceAnimFinished;
    uint8_t m_faceFps;
    uint32_t m_lastFrameTimeMs;

    // Natural idle blinking timing
    uint32_t m_nextIdleBlinkMs;
    bool m_inIdleBlink;
};
