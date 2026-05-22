#pragma once
// =============================================================
//  Per-robot configuration — selected at compile time via
//  -D ROBOT_ID=N in platformio.ini.
//
//  To add a new robot: copy an existing block, bump the ID,
//  and adjust the values. Nothing else needs to change.
// =============================================================

// IDE fallback: IntelliSense doesn't receive -D from build flags.
// This default is only active in the editor, never during real compilation.
#ifndef ROBOT_ID
#  define ROBOT_ID 1
#endif

// ── Robot 1 ──────────────────────────────────────────────────
#if ROBOT_ID == 1

#define ROBOT_NAME          "robot1"
#define WIFI_SSID           "ASUS_50"
#define WIFI_PASS           "autumn_3269"
#define AGENT_IP            "192.168.50.181"
#define AGENT_PORT          8888

// LED identity color (R, G, B)
#define LED_R  255
#define LED_G    0
#define LED_B    0   // red

// Motor & encoder polarity  (1 = normal, -1 = reversed)
#define MOTOR_A_DIR          1
#define MOTOR_B_DIR          1
#define ENC_A_DIR            1
#define ENC_B_DIR           -1

// Mechanical parameters
#define CFG_PPR              7.0f
#define CFG_GEAR_RATIO       (1.0f / 380.0f)
#define CFG_MAX_RPM          33.0f
#define CFG_WHEEL_BASE       0.106f   // metres, centre-to-centre
#define CFG_WHEEL_DIAMETER   0.040f   // metres

// PID gains  (A = left motor, B = right motor)
#define CFG_PA  50.0f
#define CFG_IA  25.0f
#define CFG_DA   0.8f
#define CFG_PB  50.0f
#define CFG_IB  25.0f
#define CFG_DB   0.8f

// ── Robot 2 ──────────────────────────────────────────────────
#elif ROBOT_ID == 2

#define ROBOT_NAME          "robot2"
#define WIFI_SSID           "ASUS_50"
#define WIFI_PASS           "autumn_3269"
#define AGENT_IP            "192.168.50.181"
#define AGENT_PORT          8888

#define LED_R    0
#define LED_G  255
#define LED_B    0   // green

#define MOTOR_A_DIR          1
#define MOTOR_B_DIR          1
#define ENC_A_DIR            1
#define ENC_B_DIR           -1

#define CFG_PPR              7.0f
#define CFG_GEAR_RATIO       (1.0f / 380.0f)
#define CFG_MAX_RPM          33.0f
#define CFG_WHEEL_BASE       0.106f
#define CFG_WHEEL_DIAMETER   0.040f

#define CFG_PA  50.0f
#define CFG_IA  25.0f
#define CFG_DA   0.8f
#define CFG_PB  50.0f
#define CFG_IB  25.0f
#define CFG_DB   0.8f

// ── Robot 3 ──────────────────────────────────────────────────
#elif ROBOT_ID == 3

#define ROBOT_NAME          "robot3"
#define WIFI_SSID           "ASUS_50"
#define WIFI_PASS           "autumn_3269"
#define AGENT_IP            "192.168.50.181"
#define AGENT_PORT          8888

#define LED_R    0
#define LED_G    0
#define LED_B  255   // blue

#define MOTOR_A_DIR          1
#define MOTOR_B_DIR          1
#define ENC_A_DIR            1
#define ENC_B_DIR           -1

#define CFG_PPR              7.0f
#define CFG_GEAR_RATIO       (1.0f / 380.0f)
#define CFG_MAX_RPM          33.0f
#define CFG_WHEEL_BASE       0.106f
#define CFG_WHEEL_DIAMETER   0.040f

#define CFG_PA  50.0f
#define CFG_IA  25.0f
#define CFG_DA   0.8f
#define CFG_PB  50.0f
#define CFG_IB  25.0f
#define CFG_DB   0.8f

// ── 继续按需添加 robot4, robot5 … ─────────────────────────────

#else
#  error "ROBOT_ID is not defined. Add  build_flags = -D ROBOT_ID=N  to platformio.ini."
#endif
