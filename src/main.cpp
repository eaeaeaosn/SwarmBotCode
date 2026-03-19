#include <Arduino.h>

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

int isMotorAInverted = 1; //1 for normal, -1 for inverted
int isMotorBInverted = 1;
int isMotorAEncoderReversed = 1; //1 for normal, -1 for reversed
int isMotorBEncoderReversed = -1;
float PPR = 7; // Pulses per revolution for the encoder
float GearRatio = 1; // Gear ratio TODO: correct ratio

long prevTime = 0;
int prevPosA = 0;
int prevPosB = 0;
volatile int posA = 0;
volatile int posB = 0;

float vaFilt = 0;
float vaPrev = 0;
float vbFilt = 0;
float vbPrev = 0;

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


void setup() {
    Serial.begin(115200);

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
    long currentTime = micros();
    if (currentTime - prevTime < 10000) return;

    int pwr = constrain((int)(255.0 / 3.0 * 3000000 / 1000000.0), 0, 255);
    
    setMotor(1 * isMotorBInverted, pwr, MOTORBPWM, MOTORBIN1, MOTORBIN2);
    setMotor(1 * isMotorAInverted, pwr, MOTORAPWM, MOTORAIN1, MOTORAIN2);

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

    vaFilt = 0.854*vaFilt + 0.0728*velocityA + 0.0728*vaPrev;
    vaPrev = velocityA;
    vbFilt = 0.854*vbFilt + 0.0728*velocityB + 0.0728*vbPrev;
    vbPrev = velocityB;

    Serial.print(velocityA);
    Serial.print(",");
    Serial.print(vaFilt);
    Serial.print(",");
    Serial.print(velocityB);
    Serial.print(",");
    Serial.print(vbFilt);
    Serial.println();
} 