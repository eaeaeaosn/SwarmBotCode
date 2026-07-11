#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <FastLED.h>
#include "config.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

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
unsigned long prevTime = 0;
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

// Real wheel-velocity feedback (RPM, output-shaft speed after gear ratio)
rcl_publisher_t wheel_vel_publisher;
std_msgs__msg__Float32MultiArray wheel_vel_msg;
static float wheel_vel_data[2];   // [0] = wheel A (left), [1] = wheel B (right)

// State machine for micro-ROS connection
enum AgentState { WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agentState = WAITING_FOR_AGENT;

unsigned long lastPingTime = 0;
bool agentConnected = false;            // confirmed only after first successful ping
uint8_t consecutivePingFailures = 0;   // hysteresis: disconnect only after 3 consecutive failures

// LED state
CRGB leds[NUM_LEDS];
CRGB robotColor = CRGB(LED_R, LED_G, LED_B);  // identity color from config.h
unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_US = 20000;      // 50 Hz refresh

// Timeout
unsigned long lastCmdTime = 0;
bool cmdActive = false;
const unsigned long CMD_TIMEOUT_US = 5000000;

// ── PID 离线调参模式 ──────────────────────────────────────────────
// 定义此宏后：跳过 WiFi / micro-ROS，直接跑 PID + 正弦目标转速
// 调参完毕后注释掉即可恢复正常联网模式
//#define PID_TEST

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

// Clears accumulated integral error. Called whenever the setpoint changes
// (new command, or a forced stop) so a stale windup from an unreachable
// target can't keep driving the motors after the setpoint has moved on.
void resetIntegrators() {
    eintegralA = 0;
    eintegralB = 0;
}

void callback_robot_control(const void* msgin) {
    const geometry_msgs__msg__Twist* twist =
        (const geometry_msgs__msg__Twist*) msgin;
    drive(twist->linear.x, twist->angular.z);
    resetIntegrators();
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
    if (RCL_RET_OK != rclc_publisher_init_default(&wheel_vel_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/" ROBOT_NAME "/wheel_vel")) return false;
    wheel_vel_msg.data.data     = wheel_vel_data;
    wheel_vel_msg.data.size     = 2;
    wheel_vel_msg.data.capacity = 2;
    if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 1, &allocator)) return false;
    if (RCL_RET_OK != rclc_executor_add_subscription(&executor, &subscriber, &msg,
        &callback_robot_control, ON_NEW_DATA)) return false;
    return true;
}

// Destroy micro-ROS entities (called when agent disconnects)
void destroyMicroROS() {
    rcl_subscription_fini(&subscriber, &node);
    rcl_publisher_fini(&wheel_vel_publisher, &node);
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
#ifndef PID_TEST
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
#else
    // PID_TEST 模式：跳过 WiFi，直接使能电机
    digitalWrite(MOTOR_STBY, HIGH);
    fill_solid(leds, NUM_LEDS, CRGB::Blue);  // 蓝色 = PID 测试模式
    FastLED.show();
    Serial.println("[3] PID_TEST mode — WiFi skipped, motors enabled");
#endif
}

void loop() {
    unsigned long currentTime = micros();

    // ── PID 测试模式：设置正弦目标，跳过状态机 ────────────────────
#ifdef PID_TEST
    {
        // ±30 RPM 正弦波，周期 2π 秒（约 6.28 s）
        float targetRPM = 20.0f * sinf(currentTime / 1000000.0f);
        targetRPMA = targetRPM;
        targetRPMB = targetRPM;
    }
#else
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
        resetIntegrators();
        return;
    }

    // agentState == AGENT_CONNECTED
    // Ping every 500 ms; tear down only after 3 consecutive failures (tolerates WiFi jitter).
    // rmw_uros_ping_agent(200, 3): 200 ms timeout per attempt × 3 attempts = up to 600 ms/cycle.
    if (currentTime - lastPingTime > 500000) {
        bool pingOk = (RMW_RET_OK == rmw_uros_ping_agent(200, 3));
        if (pingOk) {
            consecutivePingFailures = 0;
            agentConnected = true;
        } else {
            consecutivePingFailures++;
            Serial.print("Ping failed (");
            Serial.print(consecutivePingFailures);
            Serial.println("/3)");
            if (consecutivePingFailures >= 3) {
                Serial.println("Agent lost — resetting micro-ROS");
                agentConnected = false;
                consecutivePingFailures = 0;
                destroyMicroROS();
                agentState = WAITING_FOR_AGENT;
                drive(0, 0);
                resetIntegrators();
                lastPingTime = currentTime;
                return;
            }
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
        resetIntegrators();
        cmdActive = false;
    }
#endif  // !PID_TEST

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

#ifndef PID_TEST
    if (agentState == AGENT_CONNECTED) {
        wheel_vel_data[0] = vaFilt;
        wheel_vel_data[1] = vbFilt;
        rcl_publish(&wheel_vel_publisher, &wheel_vel_msg, NULL);
    }
#endif

    Serial.print(targetRPMA);
    Serial.print(",");
    Serial.print(vaFilt);
    Serial.print(",");
    Serial.print(targetRPMB);
    Serial.print(",");
    Serial.print(vbFilt);
    Serial.print(",");
    Serial.print(snapA);   // raw encoder A count — should change when motor A spins
    Serial.print(",");
    Serial.print(snapB);   // raw encoder B count — should change when motor B spins
    Serial.println();
} 
