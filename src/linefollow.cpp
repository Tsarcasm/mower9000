#include <Arduino.h>

#include "motordriver.h"

#define L_EN1 22
#define L_IN1 27
#define L_IN2 26
#define R_EN1 19
#define R_IN1 20
#define R_IN2 21

L298N motor(L_EN1, L_IN1, L_IN2, R_EN1, R_IN1, R_IN2);

struct PIDConstants {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
};
PIDConstants pidConstants = {0.15, 0.0, 0};

float computeP(float error) {
    return pidConstants.Kp * error;
}

float computeI(float error, float deltaTime) {
    static float integral = 0;
    integral += error * deltaTime;
    return pidConstants.Ki * integral;
}

float computeD(float error, float deltaTime) {
    static float previousError = 0;
    float derivative = (error - previousError) / deltaTime;
    previousError = error;
    return pidConstants.Kd * derivative;
}

float computePID(float error, float deltaTime) {
    float p = computeP(error);
    float i = computeI(error, deltaTime);
    float d = computeD(error, deltaTime);
    Serial.print("P: ");
    Serial.print(p);
    Serial.print(" I: ");
    Serial.print(i);
    Serial.print(" D: ");
    Serial.print(d);
    Serial.print(" Total: ");
    Serial.println(p + i + d);
    return p + i + d;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    delay(1000);

    // Clear serial buffer
    while (Serial.available()) {
        Serial.read();
    }
}
uint32_t last_time = 0;

void loop() {
    float x = 0;
    float y = 0;
    uint64_t timestamp = 0;
    uint64_t jitter = 0;

    if (millis() - last_time >= 500) {
        motor.setBothSpeeds(0);
    }

    // Continually read from serial port
    if (Serial.available()) {
        digitalWrite(LED_BUILTIN, HIGH);
        auto str = Serial.readStringUntil('\n');
        // String format:
        // 1.0247609093785286,34.68833666825958,1692705059400,99
        // x,y,timestamp,jitter
        sscanf(str.c_str(), "%f,%f,%llu,%llu", &x, &y, &timestamp, &jitter);
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.print(timestamp);
        Serial.print(",");
        Serial.println(jitter);
    } else {
        return;
    }

    // Compute PID
    // Feed Y value into PID
    float error = y;
    uint32_t dt = (last_time == 0) ? 0 : millis() - last_time;
    last_time = millis();
    float pid = computePID(error, dt);

    // Constrain PID to [-1, 1]
    float topSpeed = 1;
    pid = constrain(pid, -topSpeed, topSpeed);
    Serial.print("PID: ");
    Serial.println(pid);

    // Set motor speeds
    motor.setBothDirections(L298N::FORWARD);
    float innerSpeed = topSpeed - abs(pid/2);
    // Print speeds
    Serial.print("Inner speed: ");
    Serial.println(innerSpeed);
    if (pid < 0) {
        motor.setSpeedA(topSpeed);
        motor.setSpeedB(innerSpeed);
    } else {
        motor.setSpeedA(innerSpeed);
        motor.setSpeedB(topSpeed);
    }
}