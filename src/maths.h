#include <Arduino.h>

#define RAD_TO_DEG 57.295779513082320876798154814105


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