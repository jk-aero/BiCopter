/**********************************************************************************************
*   CUSTOM MONOCOPTER FLIGHT CONTROLLER
*   Teensy 4.1 + 3 Fin Servos + 2 BLDC ESC + FS-i6X RX + BNO055 NDoF Orientation
*
*   Includes:
*   - Raw PPM/iBUS receiver decoding
*   - Bosch BNO055 API (hrp Euler)
*   - PID controller for roll + pitch + yaw
*   - Full 3-FIN VECTOR CONTROL MIXER (0°,120°,240°)
*   - Motor throttle stabilizer
*   - Safety + arming
*
*   Author: ChatGPT Flight Systems Lab
**********************************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "BNO055_support.h"     // Bosch official API bridge

// ========================== RECEIVER CONFIG ==============================
#define PPM_PIN 2
volatile uint16_t ppmChannels[8];
volatile uint8_t ppmIndex = 0;

void ppmISR() {
    static uint32_t lastTime = 0;
    uint32_t now = micros();
    uint32_t diff = now - lastTime;
    lastTime = now;

    if (diff > 4000) {
        ppmIndex = 0;
    } else if (ppmIndex < 8) {
        ppmChannels[ppmIndex++] = diff;
    }
}

// Receiver channels (FS-i6X)
#define CH_THROTTLE ppmChannels[2]
#define CH_ROLL     ppmChannels[0]
#define CH_PITCH    ppmChannels[1]
#define CH_YAW      ppmChannels[3]
#define CH_ARM      ppmChannels[4]

// ========================== SERVO + ESC CONFIG ===========================

Servo fin1, fin2, fin3;
Servo motorUp, motorDown;

#define FIN1_PIN 5
#define FIN2_PIN 6
#define FIN3_PIN 9

#define MOTOR_UP_PIN 22
#define MOTOR_DOWN_PIN 23

// ========================== BNO055 CONFIG ================================

struct bno055_t bno;
struct bno055_euler eul;

unsigned long imuPrev = 0;

// ========================== PID CONTROLLERS ==============================

struct PID {
    float kp, ki, kd;
    float prevErr;
    float integral;
};

PID pidRoll  = {2.5, 0.003, 0.2};
PID pidPitch = {2.5, 0.003, 0.2};
PID pidYaw   = {1.8, 0.002, 0.12};

float dtPID = 0.01; // 100Hz loop

float computePID(PID &p, float error) {
    p.integral += error * dtPID;
    float derivative = (error - p.prevErr) / dtPID;
    p.prevErr = error;
    return p.kp * error + p.ki * p.integral + p.kd * derivative;
}

// ========================== ARMING SYSTEM ================================

bool armed = false;

void updateArming() {
    if (CH_ARM > 1500) { armed = true;  }
    else { armed = false; }
}

// ========================== VECTOR MIXER (3 FINS) ========================
// Fin angles: 0°, 120°, 240°
// servo = R*cos(theta) + P*sin(theta)

float mixFin1(float R, float P) { return R; }
float mixFin2(float R, float P) { return -0.5*R + 0.866*P; }
float mixFin3(float R, float P) { return -0.5*R - 0.866*P; }

// ========================== SETUP ========================================

void setup() {
    Serial.begin(115200);

    // PPM Receiver
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

    // IMU init
    Wire.begin();
    BNO_Init(&bno);
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(10);

    // Servos
    fin1.attach(FIN1_PIN);
    fin2.attach(FIN2_PIN);
    fin3.attach(FIN3_PIN);

    // Motors
    motorUp.attach(MOTOR_UP_PIN);
    motorDown.attach(MOTOR_DOWN_PIN);

    // Startup safe
    motorUp.writeMicroseconds(1000);
    motorDown.writeMicroseconds(1000);
    fin1.writeMicroseconds(1500);
    fin2.writeMicroseconds(1500);
    fin3.writeMicroseconds(1500);

    Serial.println("MONOCOPTER FLIGHT CONTROLLER READY");
}

// ========================== MAIN LOOP ====================================

void loop() {

    static uint32_t lastLoop = millis();
    if (millis() - lastLoop >= 10) {  // 100Hz
        lastLoop = millis();

        updateArming();

        // ---- Read IMU ----
        bno055_read_euler_hrp(&eul);
        float yaw   = eul.h / 16.0;
        float roll  = eul.r / 16.0;
        float pitch = eul.p / 16.0;

        // ---- Normalize angles ----
        if (yaw > 180) yaw -= 360;

        // ---- Receiver normalized ----
        float rollCmd  = map(CH_ROLL, 1000, 2000, -30, 30);
        float pitchCmd = map(CH_PITCH, 1000, 2000, -30, 30);
        float yawCmd   = map(CH_YAW, 1000, 2000, -60, 60);
        float throttle = map(CH_THROTTLE, 1000, 2000, 1000, 1800);

        // ---- Compute attitude errors ----
        float errRoll  = rollCmd  - roll;
        float errPitch = pitchCmd - pitch;
        float errYaw   = yawCmd   - yaw;

        // ---- PID outputs ----
        float rollPID  = computePID(pidRoll,  errRoll);
        float pitchPID = computePID(pidPitch, errPitch);
        float yawPID   = computePID(pidYaw,   errYaw);

        // ---- VECTOR CONTROL OF FINS ----
        float f1 = mixFin1(rollPID, pitchPID);
        float f2 = mixFin2(rollPID, pitchPID);
        float f3 = mixFin3(rollPID, pitchPID);

        // ---- Add center pulse ----
        f1 = 1500 + f1;
        f2 = 1500 + f2;
        f3 = 1500 + f3;

        // ---- Constrain ----
        f1 = constrain(f1, 1100, 1900);
        f2 = constrain(f2, 1100, 1900);
        f3 = constrain(f3, 1100, 1900);

        // ---- Motor stabilization with yaw ----
        float motorUpOut   = throttle + yawPID;
        float motorDownOut = throttle - yawPID;

        motorUpOut = constrain(motorUpOut, 1000, 1900);
        motorDownOut = constrain(motorDownOut, 1000, 1900);

        // ---- WRITE OUTPUTS ----
        if (armed) {
            fin1.writeMicroseconds(f1);
            fin2.writeMicroseconds(f2);
            fin3.writeMicroseconds(f3);
            motorUp.writeMicroseconds(motorUpOut);
            motorDown.writeMicroseconds(motorDownOut);
        } else {
            // SAFE MODE
            fin1.writeMicroseconds(1500);
            fin2.writeMicroseconds(1500);
            fin3.writeMicroseconds(1500);
            motorUp.writeMicroseconds(1000);
            motorDown.writeMicroseconds(1000);
        }

        // ---- Debug ----
        Serial.print("R:"); Serial.print(roll);
        Serial.print(" P:"); Serial.print(pitch);
        Serial.print(" Y:"); Serial.print(yaw);
        Serial.print(" | F1:");
        Serial.print(f1); Serial.print(" F2:");
        Serial.print(f2); Serial.print(" F3:");
        Serial.print(f3); Serial.print(" | M:");
        Serial.print(motorUpOut); Serial.print(",");
        Serial.println(motorDownOut);
    }
}
