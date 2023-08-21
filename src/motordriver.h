#pragma once
#include <Arduino.h>
#define PWM_FREQ 500
class L298N {
   public:
    enum Direction {
        FORWARD,
        BACKWARD,
        STOP
    };

   private:
    uint enA;
    uint in1;
    uint in2;
    uint enB;
    uint in3;
    uint in4;


    inline void set_direction(Direction dir, uint p1, uint p2) {
        switch (dir) {
            case FORWARD:
                digitalWriteFast(p1, 1);
                digitalWriteFast(p2, 0);
                break;
            case BACKWARD:
                digitalWriteFast(p1, 0);
                digitalWriteFast(p2, 1);
                break;
            case STOP:
            default:
                digitalWriteFast(p1, 0);
                digitalWriteFast(p2, 0);
                break;
        }
    }

   public:
    L298N(uint enA, uint in1, uint in2, uint enB, uint in3, uint in4) {
        this->enA = enA;
        this->in1 = in1;
        this->in2 = in2;
        this->enB = enB;
        this->in3 = in3;
        this->in4 = in4;

        pinMode(enA, OUTPUT);
        pinMode(enB, OUTPUT);
        
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(in3, OUTPUT);
        pinMode(in4, OUTPUT);

        analogWriteFreq(PWM_FREQ);
        analogWriteRange(255);
        analogWrite(enA, 0);
        analogWrite(enB, 0);
        digitalWriteFast(in1, 0);
        digitalWriteFast(in2, 0);
        digitalWriteFast(in3, 0);
        digitalWriteFast(in4, 0);

    }

    void setDirectionA(Direction direction) {
        set_direction(direction, in1, in2);
    }

    void setDirectionB(Direction direction) {
        set_direction(direction, in3, in4);
    }

    void setSpeedA(float speed) {
        analogWrite(enA, speed * 255);
    }

    void setSpeedB(float speed) {
        analogWrite(enB, speed * 255);
    }

    void setBothSpeeds(float speed) {
        analogWrite(enA, speed * 255);
        analogWrite(enB, speed * 255);
    }

    void setBothDirections(Direction direction) {
        set_direction(direction, in1, in2);
        set_direction(direction, in3, in4);
    }

};
