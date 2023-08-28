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


uint32_t last_report = 0;

void loop() {
    if (millis() - start_time > 20000) {
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
        const float yaw = -ypr.yaw;
        Serial.print("Yaw: ");
        Serial.println(yaw);
    }
}
