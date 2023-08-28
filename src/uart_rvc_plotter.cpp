#include <Adafruit_BNO08x.h>

#include "maths.h"
#include "motordriver.h"
#include "pid.h"

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

void setup(void) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial1.setRX(17);
    Serial1.setTX(16);
    Serial1.setFIFOSize(512);
    if (!bno08x.begin_UART(&Serial1)) {
        error("Failed to find BNO08x chip");
    }

    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        error("Could not enable game vector");
    }

    delay(100);
}

double hyp_tangent(double x, double scale) {
    return (tanh(x * scale));
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

PID drivePid(0.06, 0.02, 0.005);
PID turnPid(0.06, 0.02, 0.005);

void turn_to_face(float target, float yaw, uint32_t deltaTime, PID &pid) {
    // Turn to reduce yaw to zero
    float error = bearing_diff(yaw, target);
    float controlSpeed = pid.computePID(error, deltaTime / 1000.0f);
    bool isClockwise = controlSpeed > 0;
    controlSpeed = hyp_tangent(abs(controlSpeed), 0.2);
    motor.setBothSpeeds(controlSpeed);
    // Apply the control
    if (!isClockwise) {
        motor.setDirectionA(L298N::Direction::BACKWARD);
        motor.setDirectionB(L298N::Direction::FORWARD);
    } else {
        motor.setDirectionB(L298N::Direction::BACKWARD);
        motor.setDirectionA(L298N::Direction::FORWARD);
    }
}

void go_straight(float target, float yaw, uint32_t deltaTime, PID &pid) {
    // Drive straight
    float error = bearing_diff(yaw, target);
    float controlSpeed = pid.computePID(error, deltaTime / 1000.0f);
    bool isClockwise = controlSpeed > 0;
    controlSpeed = hyp_tangent(abs(controlSpeed), 0.2);
    // Serial.println(controlSpeed);

    motor.setBothDirections(L298N::Direction::FORWARD);
    // Apply the control
    if (!isClockwise) {
        motor.setSpeedA(1 - controlSpeed);
        motor.setSpeedB(1);
    } else {
        motor.setSpeedA(1);
        motor.setSpeedB(1 - controlSpeed);
    }
}

enum State {
    INIT,
    CAL_ZERO_YAW,
    CAL_STRAIGHT,
    CALIBRATE,
    TURN,
    STRAIGHT,
    DONE
};
volatile State state = INIT;

/*
    State machine variables
*/
uint32_t last_imu_reading_ms = 0;
uint32_t calibrate_drive_start_ms = 0;
float start_x = 0, start_y = 0;
float end_x, end_y;
float calibrated_yaw_offset = 0;

point start_point = {0, 0};
point path[] = {
    {3.7, -4.5},
    {-6.6, -6.6},
    {-7, 7},
    {0, 10},
};
#define NUM_POINTS 4

int path_index = 0;
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
            // Serial.printf("GPS: (%f,%f) timestamp: %llu, jitter: %llu, state: %d\r\n", x, y, timestamp, jitter, (int)state);
            gps_x = x;
            gps_y = y;
            last_gps = millis();
        }
    }
}

/*
    The main loop will update the state machine (and probably the PID controllers)
    every time a reading is available from the IMU
    This should happen every 10ms.
*/
void loop() {
    // If the IMU is reset then try and re-enable the game vector report
    if (bno08x.wasReset()) {
        Serial.println("Sensor was reset ");
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
            error("Could not enable game vector");
        }
    }
    // Don't update if we haven't got a sensor event
    if (!bno08x.getSensorEvent(&sensorValue) || sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
        if (millis() - last_imu_reading_ms > 1000) {
            Serial.printf("No game vector event in %dms\r\n", millis() - last_imu_reading_ms);
        }
        return;
    }

    const auto rotvec = sensorValue.un.gameRotationVector;
    const euler_t ypr = quaternionToEuler(rotvec.real, rotvec.i, rotvec.j, rotvec.k, true);
    const float yaw = fix_yaw(fix_yaw(calibrated_yaw_offset) - ypr.yaw);
    const float raw_yaw_ = ypr.yaw;

    const uint32_t deltaTime = millis() - last_imu_reading_ms;
    last_imu_reading_ms = millis();

    Serial.printf("X=%f, Y=%f, Yaw=%f, State=%d\r\n", gps_x, gps_y, yaw, (int)state);
    switch (state) {
        // Wait for the GPS to get a lock
        case INIT:
            if (millis() > 2000 && gps_x != 0) state = TURN;
            break;

        // Turn to face what the IMU says is north (yaw = 0)
        case CAL_ZERO_YAW:
            if (abs(yaw) < 2) {
                state = STRAIGHT;
                calibrate_drive_start_ms = millis();
                break;
            }
            turn_to_face(0, yaw, deltaTime, turnPid);
            break;

        // Drive straight for 7.5 seconds (facing north)
        // Take a start gps position at 3 seconds, and an end gps position at 7.5 seconds
        case CAL_STRAIGHT:
            if (millis() - calibrate_drive_start_ms > 3000 && start_x == 0 && start_y == 0) {
                start_x = gps_x;
                start_y = gps_y;
            }
            if (millis() - calibrate_drive_start_ms > 7500) {
                state = CALIBRATE;
                end_x = gps_x;
                end_y = gps_y;
            }
            go_straight(0, yaw, deltaTime, drivePid);
            break;

        // Reconcile the GPS position with the magnetometer heading
        // Calculate a yaw offset to apply to future magnetometer readings
        case CALIBRATE: {
            calibrated_yaw_offset = calc_bearing({{start_x, start_y}, {end_x, end_y}});
            start_point = {gps_x, gps_y};

            motor.setBothSpeeds(0);
            motor.setBothDirections(L298N::Direction::STOP);
            state = TURN;
            turnPid.reset();
        } break;

        // Turn to face the next point in the path
        case TURN: {
            const float target = calc_bearing({start_point, path[path_index]});

            if (abs(bearing_diff(yaw, target)) < 1) {
                state = CAL_STRAIGHT;
                drivePid.reset();
            }
            turn_to_face(target, yaw, deltaTime, turnPid);
            break;
        }

        // Drive along the line to the next point in the path
        case STRAIGHT: {
            line AB = {path[(path_index - 1) % NUM_POINTS], path[path_index]};
            const point target_point = path[path_index];
            const point pos = {gps_x, gps_y};
            const float distance = perpendicular_distance(AB, pos);
            const float approach_angle = constrain(distance / 2, 0, 1) * 27;
            const float target = approach_angle_calc(AB, pos, approach_angle);
            Serial.printf("Now (%f,%f), Start (%f,%f) Target (%f,%f), Distance: %f, Approach angle: %f, Target: %f\r\n",
                          pos.x, pos.y, start_point.x, start_point.y, target_point.x, target_point.y,
                          distance, approach_angle, target);
            if (has_passed_B(AB, pos)) {
                path_index = (path_index + 1) % NUM_POINTS;
                start_point = {gps_x, gps_y};
                state = TURN;
                turnPid.reset();
                break;
            }
            if (millis() - last_gps > 1500) {
                motor.setBothSpeeds(0);
                break;
            }
            go_straight(target, yaw, deltaTime, drivePid);
            break;
        }
        case DONE:
            motor.setBothSpeeds(0);
            motor.setBothDirections(L298N::Direction::STOP);
            Serial.printf("Heading: %f, Raw Yaw: %f, Yaw Offset: %f\r\n", yaw, raw_yaw_, calibrated_yaw_offset);
            delay(1000);
            break;
    }
}
