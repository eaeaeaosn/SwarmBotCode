# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SwarmBot is an ESP32-based differential drive robot running **micro-ROS over WiFi**. It subscribes to `cmd_vel` (geometry_msgs/Twist) from a ROS 2 agent and drives two independently PID-controlled DC motors with quadrature encoders.

## Build & Flash Commands

This project uses **PlatformIO** (not Arduino IDE). All commands run from the project root.

```bash
# Build
pio run

# Build + upload to connected ESP32
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor

# Build + upload + monitor in one step
pio run --target upload && pio device monitor

# Clean build artifacts
pio run --target clean
```

The target board is `esp32dev` with Arduino framework and WiFi micro-ROS transport (`board_microros_transport = wifi`).

## Architecture

### Control Flow

```
ROS 2 agent (PC)
    └── WiFi → micro-ROS subscriber (cmd_vel)
                    └── callback_robot_control()
                            └── drive(v, ω)  →  targetRPMA / targetRPMB
                                                        ↓
                                               PID loop (~100Hz)
                                                        ↓
                                               setMotor() → H-bridge PWM
```

### Key Subsystems (`src/main.cpp`)

**`drive(v, omega)`** — Converts robot-level linear velocity (m/s) and angular velocity (rad/s) into per-wheel target RPMs. Scales down both wheels proportionally if either exceeds `MaxRPM`.

**PID loop** — Runs every 10 ms. Reads encoder counts atomically (interrupt-safe), computes velocity with a 25 Hz Butterworth low-pass filter, then runs independent PID controllers for motors A and B. Outputs `Serial.print` CSV: `targetRPMA, vaFilt, targetRPMB, vbFilt` for live tuning.

**Safety mechanisms:**
- `CMD_TIMEOUT_US` (5 s) — stops robot if no new `cmd_vel` arrives
- Agent ping every 500 ms — stops robot if micro-ROS agent disconnects

### Hardware Pin Mapping

| Signal | GPIO |
|--------|------|
| Motor A IN1/IN2 | 14 / 4 |
| Motor A PWM | 23 |
| Motor B IN1/IN2 | 5 / 18 |
| Motor B PWM | 19 |
| Motor STBY | 13 |
| Encoder A (CH1/CH2) | 32 / 33 |
| Encoder B (CH1/CH2) | 34 / 35 |

Interrupts fire on rising edge of CH1 (ENCA1/ENCB1); CH2 pin is read inside the ISR to determine direction. The reversal flags `isMotorAReversed`, `isMotorAEncoderReversed`, etc. correct for physical wiring polarity.

### Mechanical Constants (tune in `main.cpp`)

- `PPR = 7` — encoder pulses per motor shaft revolution
- `GearRatio = 1/380` — output shaft turns per motor revolution
- `MaxRPM = 33` — max output shaft RPM at 5 V
- `WheelBase = 0.106 m`, `WheelDiameter = 0.040 m`

### Network Config (update before flashing)

WiFi SSID/password and agent IP are hardcoded in `setup()`:
```cpp
set_microros_wifi_transports("ASUS_50", "autumn_3269", agent_ip, 8888);
```
Agent IP: `192.168.50.181`, port `8888`. The ROS 2 node name is `robot1`, topic is `/cmd_vel`.

### IMU (currently disabled)

`src/imu.cpp` contains a fully commented-out ICM-20948 implementation using DMP Quat6 (fast, ~225 Hz) fused with Quat9 (magnetometer, ~25 Hz) for yaw drift correction. Re-enable by uncommenting and adding to `platformio.ini`.

### IMU Visualizer

`imu_visualize/sketch_260203a.pde` is a **Processing** sketch that reads serial CSV (`roll,pitch,yaw`) from the ESP32 and renders a 3D board. Hardcoded to `COM6` at 115200 baud.

## Dependencies

| Library | Source |
|---------|--------|
| micro_ros_platformio | `https://gitee.com/ohhuo/micro_ros_platformio.git` |
| SparkFun ICM-20948 | `sparkfun/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library@^1.3.2` |
