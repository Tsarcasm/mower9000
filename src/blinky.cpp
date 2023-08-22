#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // Continually read from serial port
    if (Serial.available()) {
        digitalWrite(LED_BUILTIN, HIGH);
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
            Serial.println(jitter);
        }
        delay(25);
        digitalWrite(LED_BUILTIN, LOW);
    }
}