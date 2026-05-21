#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <FastLED.h>
#include "config.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#define ENCB1 34
#define ENCB2 35
#define ENCA1 32
#define ENCA2 33
#define MOTORBIN1 5
#define MOTORBIN2 18
#define MOTORAIN1 14
#define MOTORAIN2 4
#define MOTORBPWM 19
#define MOTORAPWM 23
#define MOTOR_STBY 13

// LED config
#define LED_PIN        16
#define NUM_LEDS       22
#define LED_TYPE       WS2812B
#define COLOR_ORDER    GRB
#define LED_BRIGHTNESS 180   // 0-255, caps peak current draw (~65 mA at 180 for 22 LEDs)

// Mechanical config — values come from include/config.h
int isMotorAReversed        = MOTOR_A_DIR;
int isMotorBReversed        = MOTOR_B_DIR;
int isMotorAEncoderReversed = ENC_A_DIR;
int isMotorBEncoderReversed = ENC_B_DIR;
float PPR           = CFG_PPR;
float GearRatio     = CFG_GEAR_RATIO;
float MaxRPM        = CFG_MAX_RPM;
float WheelBase     = CFG_WHEEL_BASE;
float WheelDiameter = CFG_WHEEL_DIAMETER;

// Motor Control variables
long prevTime = 0;
int prevPosA = 0;
int prevPosB = 0;
volatile int posA = 0;
volatile int posB = 0;

float vaFilt = 0;
float vaPrev = 0;
float vbFilt = 0;
float vbPrev = 0;

//Robot Control Variables
float targetRPMA = 0;
float targetRPMB = 0;

// PID values — from config.h
float pa = CFG_PA;
float ia = CFG_IA;
float da = CFG_DA;
float pb = CFG_PB;
float ib = CFG_IB;
float db = CFG_DB;

float eintegralA = 0;
float eintegralB = 0;
float eprevA = 0;
float eprevB = 0;

// Microros variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

// State machine for micro-ROS connection
enum AgentState { WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agentState = WAITING_FOR_AGENT;

long lastPingTime = 0;
bool agentConnected = false;  // confirmed only after first successful ping

// LED state
CRGB leds[NUM_LEDS];
CRGB robotColor = CRGB(LED_R, LED_G, LED_B);  // identity color from config.h
long lastLedUpdate = 0;
const long LED_UPDATE_US = 20000;      // 50 Hz refresh

// Timeout
long lastCmdTime = 0;
bool cmdActive = false;
const long CMD_TIMEOUT_US = 5000000;

// Set this robot's unique identity color (call once in setup or from serial).
// e.g. robot1=red(255,0,0)  robot2=green(0,255,0)  robot3=blue(0,0,255)
void setRobotColor(uint8_t r, uint8_t g, uint8_t b) {
    robotColor = CRGB(r, g, b);
}

// Call every loop iteration; handles both breathing and solid modes.
// Breathing: slow sine-wave on brightness when agent is disconnected.
// Solid:     full identity color when agent is connected.
void updateLEDs(bool connected) {
    if (connected) {
        fill_solid(leds, NUM_LEDS, robotColor);
    } else {
        // 2-second breathing cycle, minimum 5% brightness so color is always visible
        float t = micros() / 1000000.0f;
        float brightness = (sinf(2.0f * PI * t / 2.0f) + 1.0f) / 2.0f; // 0..1
        brightness = 0.05f + brightness * 0.95f;
        CRGB dimmed(
            (uint8_t)(robotColor.r * brightness),
            (uint8_t)(robotColor.g * brightness),
            (uint8_t)(robotColor.b * brightness)
        );
        fill_solid(leds, NUM_LEDS, dimmed);
    }
    FastLED.show();
}

void setMotor(int dir, int pwmval, int pwm, int in1, int in2) {
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwm, pwmval);
}

void readEncoderA() {
    int a = digitalRead(ENCA2);
    if (a == HIGH) {
        posA += 1 * isMotorAEncoderReversed;
    } else {
        posA -= 1 * isMotorAEncoderReversed;
    }
}

void readEncoderB() {
    int b = digitalRead(ENCB2);
    if (b == HIGH) {
        posB += 1 * isMotorBEncoderReversed;
    } else {
        posB -= 1 * isMotorBEncoderReversed;
    }
}

void drive(float v, float omega) {
    float vL = v - omega * WheelBase / 2;
    float vR = v + omega * WheelBase / 2;

    float maxWheel = max(fabs(vL), fabs(vR));
    float maxWheelSpeed = MaxRPM / 60.0 * PI * WheelDiameter;

    if (maxWheel > maxWheelSpeed) {
        float scale = maxWheelSpeed / maxWheel;
        vL *= scale;
        vR *= scale;
    }

    float rpmL = vL / (PI * WheelDiameter) * 60.0;
    float rpmR = vR / (PI * WheelDiameter) * 60.0;

    targetRPMA = rpmL;
    targetRPMB = rpmR;
}

void callback_robot_control(const void* msgin) {
    const geometry_msgs__msg__Twist* twist = 
        (const geometry_msgs__msg__Twist*) msgin;
    drive(twist->linear.x, twist->angular.z);
    lastCmdTime = micros();  
    cmdActive = true; 
}


// Initialize micro-ROS entities (called once agent is found)
bool initMicroROS() {
    allocator = rcl_get_default_allocator();
    if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &allocator)) return false;
    if (RCL_RET_OK != rclc_node_init_default(&node, ROBOT_NAME, "", &support)) return false;
    if (RCL_RET_OK != rclc_subscription_init_default(&subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/" ROBOT_NAME "/cmd_vel")) return false;
    if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 1, &allocator)) return false;
    if (RCL_RET_OK != rclc_executor_add_subscription(&executor, &subscriber, &msg,
        &callback_robot_control, ON_NEW_DATA)) return false;
    return true;
}

// Destroy micro-ROS entities (called when agent disconnects)
void destroyMicroROS() {
    rcl_subscription_fini(&subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);
}

void setup() {
    // ── 1. Serial（最先，方便调试）────────────────────────────────
    Serial.begin(115200);
    delay(200);  // 等串口稳定
    Serial.println("[1] Boot start");

    // ── 2. LED（在 WiFi 之前，给视觉反馈）────────────────────────
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(LED_BRIGHTNESS);
    fill_solid(leds, NUM_LEDS, CRGB::White);  // 白色自检
    FastLED.show();
    Serial.println("[2] FastLED OK");

    // ── 3. 电机 STBY 低（安全启动）───────────────────────────────
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, LOW);

    // ── 4. 编码器引脚 & 中断────────────────────────────────────
    pinMode(ENCB1, INPUT);
    pinMode(ENCB2, INPUT);
    pinMode(ENCA1, INPUT);
    pinMode(ENCA2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoderB, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoderA, RISING);

    // ── 5. 电机引脚────────────────────────────────────────────
    pinMode(MOTORBIN1, OUTPUT); pinMode(MOTORBIN2, OUTPUT); pinMode(MOTORBPWM, OUTPUT);
    pinMode(MOTORAIN1, OUTPUT); pinMode(MOTORAIN2, OUTPUT); pinMode(MOTORAPWM, OUTPUT);
    digitalWrite(MOTORBIN1, LOW); digitalWrite(MOTORBIN2, LOW); digitalWrite(MOTORBPWM, LOW);
    digitalWrite(MOTORAIN1, LOW); digitalWrite(MOTORAIN2, LOW); digitalWrite(MOTORAPWM, LOW);

    // ── 6. WiFi + micro-ROS transport（可能耗时，前面已有反馈）──
    Serial.print("[3] Connecting WiFi: ");
    Serial.println(WIFI_SSID);
    fill_solid(leds, NUM_LEDS, CRGB::Yellow);  // 黄色 = 正在连接 WiFi
    FastLED.show();

    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT);

    Serial.println("[4] WiFi transport ready");

    // ── 7. 电机 STBY 拉高（WiFi 连上才开启）──────────────────────
    digitalWrite(MOTOR_STBY, HIGH);

    // 熄灭自检白灯，进入呼吸等待模式
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    Serial.println("[5] Setup done. Waiting for micro-ROS agent...");
}

void loop() {
    long currentTime = micros();

    // sine wave to tune pid
    // float targetRPM = 30.0 * sin(currentTime / 1000000.0); 
    // targetRPMA = targetRPM;
    // targetRPMB = targetRPM;

    // testing speed limiting
    // if ((currentTime / 5000000) % 2 == 0) { 
    //     drive(0.05, 0.5);
    // } else {
    //     drive(0.1, 1.0);
    // }

    // ── State machine: WAITING_FOR_AGENT ↔ AGENT_CONNECTED ──────
    if (agentState == WAITING_FOR_AGENT) {
        // LED breathing first — before the blocking ping call
        if (currentTime - lastLedUpdate >= LED_UPDATE_US) {
            updateLEDs(false);
            lastLedUpdate = currentTime;
        }

        // Try to find the agent every 2 s (ping timeout 50 ms to minimise blocking)
        if (currentTime - lastPingTime > 2000000) {
            Serial.println("Pinging agent...");
            if (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) {
                if (initMicroROS()) {
                    agentState     = AGENT_CONNECTED;
                    agentConnected = true;
                    Serial.println("Agent connected — micro-ROS ready");
                }
            }
            lastPingTime = currentTime;
        }

        drive(0, 0);
        return;
    }

    // agentState == AGENT_CONNECTED
    // Ping every 500 ms; tear down if agent disappears
    if (currentTime - lastPingTime > 500000) {
        agentConnected = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
        if (!agentConnected) {
            Serial.println("Agent lost — resetting micro-ROS");
            destroyMicroROS();
            agentState = WAITING_FOR_AGENT;
            drive(0, 0);
            lastPingTime = currentTime;
            return;
        }
        lastPingTime = currentTime;
    }

    // LED: solid when connected (50Hz update)
    if (currentTime - lastLedUpdate >= LED_UPDATE_US) {
        updateLEDs(true);
        lastLedUpdate = currentTime;
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    if (cmdActive && (micros() - lastCmdTime > CMD_TIMEOUT_US)) {
        drive(0, 0);
        cmdActive = false;
    }

    if (currentTime - prevTime < 10000) return;

    // Read encoder values and calculate velocities
    int snapA, snapB;
    noInterrupts();
    snapA = posA;
    snapB = posB;
    interrupts();

    float deltaTime = (currentTime - prevTime) / 1000000.0; // Convert to seconds
    float velocityA = (snapA - prevPosA) / deltaTime / PPR * GearRatio * 60.0; // Speed in revolutions per second
    float velocityB = (snapB - prevPosB) / deltaTime / PPR * GearRatio * 60.0;

    prevPosA = snapA;
    prevPosB = snapB;
    prevTime = currentTime;

    // Low-pass filter, 25Hz cutoff
    vaFilt = 0.854*vaFilt + 0.0728*velocityA + 0.0728*vaPrev;
    vaPrev = velocityA;
    vbFilt = 0.854*vbFilt + 0.0728*velocityB + 0.0728*vbPrev;
    vbPrev = velocityB;

    // PID
    float integralLimit = 255.0 / ia;
    float ea = targetRPMA - vaFilt;
    eintegralA = eintegralA + ea * deltaTime;
    eintegralA = constrain(eintegralA, -integralLimit, integralLimit);
    float ederivativeA = (ea - eprevA) / deltaTime;
    eprevA = ea;
    float eb = targetRPMB - vbFilt;
    eintegralB = eintegralB + eb * deltaTime;
    eintegralB = constrain(eintegralB, -integralLimit, integralLimit);
    float ederivativeB = (eb - eprevB) / deltaTime;
    eprevB = eb;

    float ua = pa * ea + ia * eintegralA + da * ederivativeA;
    float ub = pb * eb + ib * eintegralB + db * ederivativeB;

    int dirA = 1;
    if (ua < 0) {
        dirA = -1;
    }
    int pwrA = (int) fabs(ua);
    if (pwrA > 255) {
        pwrA = 255;
    }
    int dirB = 1;
    if (ub < 0) {
        dirB = -1;
    }
    int pwrB = (int) fabs(ub);
    if (pwrB > 255) {
        pwrB = 255;
    }
    setMotor(dirA * isMotorAReversed, pwrA, MOTORAPWM, MOTORAIN1, MOTORAIN2);
    setMotor(dirB * isMotorBReversed, pwrB, MOTORBPWM, MOTORBIN1, MOTORBIN2);

    Serial.print(targetRPMA);
    Serial.print(",");
    Serial.print(vaFilt);
    Serial.print(",");
    Serial.print(targetRPMB);
    Serial.print(",");
    Serial.print(vbFilt);
    Serial.println();
} 
