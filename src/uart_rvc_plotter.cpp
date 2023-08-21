#include <Adafruit_BNO08x.h>

#include "motordriver.h"
#define RAD_TO_DEG 57.295779513082320876798154814105
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#define L_EN1 22
#define L_IN1 27
#define L_IN2 26
#define R_EN1 19
#define R_IN1 20
#define R_IN2 21

L298N motor(L_EN1, L_IN1, L_IN2, R_EN1, R_IN1, R_IN2);

void error(const char* msg) {
    digitalWrite(LED_BUILTIN, LOW);
    while (true) {
        Serial.println("Error");
        Serial.println(msg);
        delay(500);
    }
}

uint32_t start_time = 0;
void setup(void) {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial1.setRX(17);
    Serial1.setTX(16);
    Serial1.setFIFOSize(512);
    if (!bno08x.begin_UART(&Serial1)) {
        error("Failed to find BNO08x chip");
    }

    Serial.println("BNO08x Found!");

    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        error("Could not enable game vector");
    }

    delay(100);
    start_time = millis();
}

struct PIDConstants {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
};

PIDConstants pidConstants = {0.03, 0.02, 0.005};  // These values are placeholders. You should tune them accordingly.

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

// Desired yaw value
float setPoint = 0;

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};
euler_t quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false) {
    euler_t ypr;
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
        ypr.yaw *= RAD_TO_DEG;
        ypr.pitch *= RAD_TO_DEG;
        ypr.roll *= RAD_TO_DEG;
    }
    return ypr;
}

double modified_sigmoid(double x, double k) {
    return x / (x + (1.0 - x) * exp(-k * x));
}

uint32_t last_report = 0;

void loop() {
    if (millis() - start_time < 10000) {
        motor.setBothSpeeds(0);
        return;
    }

    if (bno08x.wasReset()) {
        Serial.println("Sensor was reset ");
        if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
            error("Could not enable game vector");
        }
    }
    if (!bno08x.getSensorEvent(&sensorValue)) {
        if (millis() - last_report > 1000) {
            Serial.println("No sensor event");
            last_report = millis();
        }
        return;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    auto rotvec = sensorValue.un.rotationVector;

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        uint32_t interval = millis() - last_report;
        float deltaTime = interval / 1000.0;  // Convert to seconds

        last_report = millis();
        auto ypr = quaternionToEuler(rotvec.real, rotvec.i, rotvec.j, rotvec.k, true);
        const float yaw = ypr.yaw;
        const float error = yaw;  // abs(yaw);
        // bool isClockwise = yaw > 0;

        float controlSpeed = computePID(error, deltaTime);

        Serial.print("Yaw: ");
        Serial.print(yaw);
        Serial.print(" Error: ");
        Serial.print(error);
        Serial.print(" Control Output: ");
        Serial.print(controlSpeed);
        Serial.print(" Direction: ");
        bool isClockwise = controlSpeed > 0;
        Serial.println(isClockwise ? "Clockwise" : "Counter-clockwise");

        controlSpeed = constrain(abs(controlSpeed), 0, 1);
        controlSpeed = modified_sigmoid(controlSpeed, 5);
        Serial.println(controlSpeed);
        if (controlSpeed < 0.1) {
            controlSpeed = 0;
        }

        motor.setBothDirections(L298N::Direction::FORWARD);
        // Apply the control
        if (isClockwise) {
            motor.setSpeedA(1 - controlSpeed);
            // motor.setDirectionA(L298N::Direction::BACKWARD);
            motor.setSpeedB(1);
        } else {
            motor.setSpeedA(1);
            motor.setSpeedB(1 - controlSpeed);
            // motor.setDirectionB(L298N::Direction::BACKWARD);
        }
        // motor.setBothSpeeds(controlSpeed);
    }
}
