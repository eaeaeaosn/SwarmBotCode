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

int posA = 0;
int posB = 0;
int isMotorAInverted = 1; //1 for normal, -1 for inverted
int isMotorBInverted = 1;
int isMotorAEncoderReversed = 1; //1 for normal, -1 for reversed
int isMotorBEncoderReversed = -1;


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
    setMotor(1 * isMotorBInverted, 25, MOTORBPWM, MOTORBIN1, MOTORBIN2);
    setMotor(1 * isMotorAInverted, 25, MOTORAPWM, MOTORAIN1, MOTORAIN2);
    Serial.print("Encoder A: ");
    Serial.print(posA);
    Serial.print(" | Encoder B: ");
    Serial.println(posB);
}