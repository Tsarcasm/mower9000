#include <Adafruit_BNO08x.h>

#include "maths.h"
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

void error(const char *msg) {
    digitalWrite(LED_BUILTIN, LOW);
    while (true) {
        Serial.println("Error");
        Serial.println(msg);
        delay(500);
    }
}

uint32_t start_time = 0;
void setup(void) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial1.setRX(17);
    Serial1.setTX(16);
    Serial1.setFIFOSize(512);
    if (!bno08x.begin_UART(&Serial1)) {
        error("Failed to find BNO08x chip");
    }

    // Serial.println("BNO08x Found!");

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

struct PIDState {
    float integral;
    float previousError;
};

float computeP(float error, PIDConstants C) {
    return C.Kp * error;
}

float computeI(float error, float deltaTime, PIDConstants C, PIDState &state) {
    state.integral += error * deltaTime;
    return C.Ki * state.integral;
}

float computeD(float error, float deltaTime, PIDConstants C, PIDState &state) {
    float derivative = (error - state.previousError) / deltaTime;
    state.previousError = error;
    return C.Kd * derivative;
}

float computePID(float error, float deltaTime, PIDConstants c, PIDState &state) {
    float p = computeP(error, c);
    float i = computeI(error, deltaTime, c, state);
    float d = computeD(error, deltaTime, c, state);
    // Serial.print("P: ");
    // Serial.print(p);
    // Serial.print(" I: ");
    // Serial.print(i);
    // Serial.print(" D: ");
    // Serial.print(d);
    // Serial.print(" Total: ");
    // Serial.println(p + i + d);
    return p + i + d;
}

// Desired yaw value
float setPoint = 0;

double modified_sigmoid(double x, double k) {
    return x / (x + (1.0 - x) * exp(-k * x));
}

float fix_yaw(float yaw) {
    // Get back in range [-180, 180]
    if (yaw > 180) {
        yaw -= 360;
    } else if (yaw < -180) {
        yaw += 360;
    }
    return yaw;
}

uint32_t last_report = 0;

enum State {
    INIT,
    TURN,
    STRAIGHT,
    CALIBRATE,
    CAL_TURN,
    CAL_STRAIGHT
};
volatile State state = INIT;

PIDConstants driveConstants = {0.06, 0.02, 0.005};
PIDState driveState = {0, 0};
PIDConstants turnConstants = {0.06, 0.02, 0.005};
PIDState turnState = {0, 0};

uint32_t last_time = 0;
uint32_t drive_start_ms = 0;
float start_x = 0, start_y = 0;
float end_x, end_y;
float calibrated_yaw_offset = 0;

void setup1() {
    Serial.begin(115200);
}

volatile float gps_x = 0;
volatile float gps_y = 0;
volatile uint32_t last_gps = 0;

void loop1() {
    if (Serial.available()) {
        while (Serial.available()) {
            auto str = Serial.readStringUntil('\n');
            // String format:
            // 1.0247609093785286,34.68833666825958,1692705059400,99
            // x,y,timestamp,jitter
            float x = 0;
            float y = 0;
            uint64_t timestamp = 0;
            uint64_t jitter = 0;
            sscanf(str.c_str(), "%f,%f,%llu,%llu", &x, &y, &timestamp, &jitter);
            Serial.print(x);
            Serial.print(",");
            Serial.print(y);
            Serial.print(",");
            Serial.print(timestamp);
            Serial.print(",");
            Serial.print(jitter);
            Serial.println((int)state);
            gps_x = x;
            gps_y = y;
            last_gps = millis();
        }
    }
}

void turn_to_face(float target, float yaw, uint32_t deltaTime, PIDConstants &pid_constants, PIDState &pid_state) {
    // Turn to reduce yaw to zero
    float error = yaw - target;
    float controlSpeed = computePID(error, deltaTime / 1000.0f, turnConstants, turnState);
    bool isClockwise = controlSpeed > 0;
    controlSpeed = modified_sigmoid(abs(controlSpeed), 5);
    motor.setBothSpeeds(controlSpeed);
    // Apply the control
    if (isClockwise) {
        motor.setDirectionA(L298N::Direction::BACKWARD);
        motor.setDirectionB(L298N::Direction::FORWARD);
    } else {
        motor.setDirectionB(L298N::Direction::BACKWARD);
        motor.setDirectionA(L298N::Direction::FORWARD);
    }
}

void go_straight(float target, float yaw, uint32_t deltaTime, PIDConstants &pid_constants, PIDState &pid_state) {
    // Drive straight
    float error = fix_yaw(yaw - target);
    float controlSpeed = computePID(error, deltaTime / 1000.0f, driveConstants, driveState);
    bool isClockwise = controlSpeed > 0;
    controlSpeed = modified_sigmoid(abs(controlSpeed), 5);
    // Serial.println(controlSpeed);

    motor.setBothDirections(L298N::Direction::FORWARD);
    // Apply the control
    if (isClockwise) {
        motor.setSpeedA(1 - controlSpeed);
        motor.setSpeedB(1);
    } else {
        motor.setSpeedA(1);
        motor.setSpeedB(1 - controlSpeed);
    }
}

void loop() {
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

    auto rotvec = sensorValue.un.rotationVector;
    float yaw = 0;
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        auto ypr = quaternionToEuler(rotvec.real, rotvec.i, rotvec.j, rotvec.k, true);
        yaw = ypr.yaw;
    } else {
        return;
    }

    // Apply the calibrated yaw offset
    yaw -= calibrated_yaw_offset;
    yaw = fix_yaw(yaw);



    uint32_t deltaTime = millis() - last_time;
    last_time = millis();

    switch (state) {
        case INIT:
            if (millis() > 2000 && gps_x != 0) state = TURN;
            break;
        case TURN:
            // Turn to reduce yaw to zero
            if (abs(yaw) < 3) {
                state = STRAIGHT;
                drive_start_ms = millis();
                break;
            }
            turn_to_face(0, yaw, deltaTime, turnConstants, turnState);
            break;

        case STRAIGHT:
            if (millis() - drive_start_ms > 3000 && start_x == 0 && start_y == 0) {
                start_x = gps_x;
                start_y = gps_y;
            }
            if (millis() - drive_start_ms > 7500) {
                state = CALIBRATE;
                end_x = gps_x;
                end_y = gps_y;
            }
            go_straight(0, yaw, deltaTime, driveConstants, driveState);
            break;

        case CALIBRATE: {
            // Calculate our heading based on the GPS
            float dx = end_x - start_x;
            float dy = end_y - start_y;
            float heading = atan2(dy, dx) * RAD_TO_DEG;
            calibrated_yaw_offset = heading;
            Serial.print("Heading: ");
            Serial.println(heading);

            motor.setBothSpeeds(0);
            motor.setBothDirections(L298N::Direction::STOP);
            // Serial.println(yaw);
            state = CAL_TURN;
        } break;

        case CAL_TURN: {
            if (abs(yaw) < 5) {
                state = CAL_STRAIGHT;
                drive_start_ms = millis();
            }
            const float y = gps_y + 1;
            const float target = constrain(y / 4, -1, 1) * 75;
            turn_to_face(target, yaw, deltaTime, turnConstants, turnState);
            break;
        }

        case CAL_STRAIGHT: {
            // We want to drive on the y=0 line
            // Calculate the error
            const float y = gps_y + 1;
            const float target = constrain(y / 4, -1, 1) * 75;
            // const float target = -90;
            Serial.print("Target: ");
            Serial.println(target);
            Serial.print("Yaw: ");
            Serial.println(yaw);
            if (millis() - drive_start_ms > 60000 || millis() - last_gps > 1500) {
                motor.setBothSpeeds(0);
                motor.setBothDirections(L298N::Direction::STOP);
                break;
            }
            // go_straight(calculated_yaw, yaw, deltaTime, driveConstants, driveState);
            go_straight(target, yaw, deltaTime, driveConstants, driveState);
            break;
        }
    }
}
