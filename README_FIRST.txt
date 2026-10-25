SPIDER / QUADRUPED ROBOT — PROJECT ARCHIVE
===========================================

This archive gathers spider/quadruped-related files found in the working workspace.
It preserves multiple project variants; these are NOT one interchangeable firmware set.

FOLDERS
-------
01_12_SERVO_ESP32/
  - QUADROPAD_12Servo_Integrated_Codebase.zip
  - Spider_Walk_12Servo.ino
  Intended for ESP32 + PCA9685 + 12 servos, using channels 0,1,2,4,5,6,8,9,10,12,13,14.
  The standalone sketch is a basic open-loop demonstration and has not been confirmed to compile
  or walk the physical robot. Calibrate every joint before floor testing.

02_8_SERVO_SESAME_VARIANTS/
  - Original and arranged Sesame-style firmware packages
  - Autonomous and VL53L0X packages
  - Original Sesame source archive and EasyEDA import archive
  These variants use different assumptions and may use an 8-servo channel map. Do not combine
  their movement header with the 12-servo sketch without porting and reviewing the code.

03_CALIBRATION_AND_CONTROL/
  - Calibration sketches, Bluetooth control sketch, channel-mapped movement header
  Note: the calibration controller and movement header use the older 8-servo mapping.

04_DOCUMENTATION_AND_BOM/
  - Research report, component list, quadruped report, spreadsheet research and budget sheet.
  Some reports describe an Arduino Uno / 8-servo design rather than the current ESP32 / 12-servo build.

05_WIRING_AND_MECHANICAL/
  - PCA9685/LM2596 wiring image, adapter plate STL and preview.

IMPORTANT ELECTRICAL SAFETY
---------------------------
- PCA9685 VCC is logic power; PCA9685 V+ is the servo power rail.
- LM2596 OUT+ (measured regulated 5.0 V) goes to PCA9685 V+; OUT- goes to servo-power GND.
- ESP32 GND, PCA9685 GND, and servo supply negative must share a common ground.
- Do not power servos from the ESP32 3.3 V pin. Check that the regulator and wiring can supply
  the combined current for the servo load. A small LM2596 module may be insufficient for 12 servos.
- Test with the robot lifted. Stop immediately if a servo buzzes, stalls, binds, or hits the frame.
- Tick values, neutral angles, and direction multipliers are starting assumptions and need calibration.

No included firmware is represented as hardware-validated. Compile and test the exact version for
which your board, channel mapping, and connected hardware are configured.
