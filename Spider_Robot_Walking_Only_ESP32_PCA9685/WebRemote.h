#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

class GaitPlanner;
class ServoDriver;
class FaceController;

/**
 * ============================================================================
 * WEB REMOTE CONTROLLER MODULE (Phone Wi-Fi SoftAP + Captive Portal)
 * ============================================================================
 * Creates a Wi-Fi Access Point ("Spider-Robot") and serves a responsive,
 * touch-optimized web remote control application for smartphones.
 * 
 * Features:
 *   - Zero app installation: connect to Wi-Fi and open browser at 192.168.4.1
 *   - Captive portal auto-detection for iOS and Android
 *   - Virtual D-pad directional walking controls (Forward, Back, Left, Right, Stop)
 *   - Posture selection (Stand, Sit)
 *   - Facial expression switcher (Happy, Cute, Angry, Love, Dead, Sleepy, etc.)
 *   - Real-time individual servo and leg hardware diagnostics test buttons
 * ============================================================================
 */

class WebRemote {
public:
    WebRemote();

    /**
     * Initialize Wi-Fi SoftAP, DNS captive portal, and HTTP web server.
     * @param gaitPlanner Pointer to active GaitPlanner
     * @param servoDriver Pointer to active ServoDriver
     * @param faceController Pointer to active FaceController
     * @param apSsid Access point SSID name (default "Spider-Robot")
     * @param apPassword Password (default "" for open network)
     */
    bool init(GaitPlanner* gaitPlanner,
              ServoDriver* servoDriver,
              FaceController* faceController,
              const bool* inCalibrationMode = nullptr,
              const char* apSsid = "Spider-Robot",
              const char* apPassword = "");

    /**
     * Non-blocking HTTP and DNS request processing (call every loop iteration).
     */
    void update();

    /**
     * Check if client phone is currently connected.
     */
    bool hasClients();

    /**
     * Get soft AP IP address string.
     */
    String getIPAddress() const;

private:
    void setupRoutes();
    void handleRoot();
    void handleCommand();
    void handleStatus();
    void handleNotFound();

    WebServer m_server;
    DNSServer m_dnsServer;

    GaitPlanner* m_gaitPlanner;
    ServoDriver* m_servoDriver;
    FaceController* m_faceController;
    const bool* m_inCalibrationMode;

    const char* m_ssid;
    const char* m_password;
    bool m_initialized;
};
