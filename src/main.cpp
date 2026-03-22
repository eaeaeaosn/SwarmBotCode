#include <Arduino.h>
#include "BluetoothSerial.h"

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

BluetoothSerial SerialBT;

// Mechanical config
int isMotorAReversed = 1; //1 for normal, -1 for reversed
int isMotorBReversed = 1;
int isMotorAEncoderReversed = 1; //1 for normal, -1 for reversed
int isMotorBEncoderReversed = -1;
float PPR = 7; // Pulses per revolution for the encoder
float GearRatio = 1.0/385.0; // Gear ratio 
float MaxRPM = 33; // Maximum RPM of the motor under 5V supply 
float WheelBase = 0.106; // Distance between the midpoints of two wheels in meters
float WheelDiameter = 0.040; // Diameter of the wheel in meters

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

// PID values
float pa = 50.0;
float ia = 25.0;
float da = 0.8;
float pb = 50.0;
float ib = 25.0;
float db = 0.8;

float integralLimit = 255.0 / ia;
float eintegralA = 0;
float eintegralB = 0;
float eprevA = 0;
float eprevB = 0;

// Bluetooth PID tuning
String input = "";

void processCommand(String cmd) {
  if (cmd.startsWith("PA=")) {
    pa = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("IA=")) {
    ia = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("DA=")) {
    da = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("PB=")) {
    pb = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("IB=")) {
    ib = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("DB=")) {
    db = cmd.substring(3).toFloat();
  }

  SerialBT.printf("Updated PID: PA=%.2f IA=%.2f DA=%.2f  PB=%.2f IB=%.2f DB=%.2f\n", pa, ia, da, pb, ib, db);
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


void setup() {
    Serial.begin(115200);

    SerialBT.begin("ESP32_PID"); // Bluetooth name
    Serial.println("Ready for PID tuning...");

    // Init encoder pins
    pinMode(ENCB1, INPUT);
    pinMode(ENCB2, INPUT);
    pinMode(ENCA1, INPUT);
    pinMode(ENCA2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoderB, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoderA, RISING);

    // Init motor pins
    pinMode(MOTORBIN1, OUTPUT);
    pinMode(MOTORBIN2, OUTPUT);
    pinMode(MOTORBPWM, OUTPUT);
    pinMode(MOTORAIN1, OUTPUT);
    pinMode(MOTORAIN2, OUTPUT);
    pinMode(MOTORAPWM, OUTPUT);

    // Ensure motors are off at startup
    digitalWrite(MOTORBIN1, LOW);
    digitalWrite(MOTORBIN2, LOW);
    digitalWrite(MOTORBPWM, LOW);
    digitalWrite(MOTORAIN1, LOW);
    digitalWrite(MOTORAIN2, LOW);
    digitalWrite(MOTORAPWM, LOW);
}

void loop() {
    while (SerialBT.available()) {
        char c = SerialBT.read();
        if (c == '\n') {
        processCommand(input);
        input = "";
        } else {
        input += c;
        }
    }

    long currentTime = micros();

    //sine wave to tune pid
    // float targetRPM = 30.0 * sin(currentTime / 1000000.0); 
    // targetRPMA = targetRPM;
    // targetRPMB = targetRPM;

    //testing speed limiting
    // if ((currentTime / 5000000) % 2 == 0) { 
    //     drive(0.05, 0.5);
    // } else {
    //     drive(0.1, 1.0);
    // }

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