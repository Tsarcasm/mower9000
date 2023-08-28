#include <Arduino.h>

class PID {
   private:
    // Constants
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain

    // State
    float integral;
    float previousError;

    // Compute the Proportional term
    float computeP(float error) {
        return Kp * error;
    }

    // Compute the Integral term
    float computeI(float error, float deltaTime) {
        integral += error * deltaTime;
        return Ki * integral;
    }

    // Compute the Derivative term
    float computeD(float error, float deltaTime) {
        float derivative = (error - previousError) / deltaTime;
        previousError = error;
        return Kd * derivative;
    }

   public:
    struct pid_components {
        float p, i, d;
    };
    // Constructor
    PID(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd), integral(0), previousError(0) {
    }

    // Set the PID constants
    void setConstants(float newKp, float newKi, float newKd) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
    }

    // Reset the internal state
    void reset() {
        integral = 0;
        previousError = 0;
    }

    // Compute the PID control value
    // Optionally write the individual components to the components pointer
    float computePID(float error, float deltaTime, pid_components* components = nullptr) {
        float p = computeP(error);
        float i = computeI(error, deltaTime);
        float d = computeD(error, deltaTime);
        if (components != nullptr) {
            components->p = p;
            components->i = i;
            components->d = d;
        }
        return p + i + d;
    }
};
