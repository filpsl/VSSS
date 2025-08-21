#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

class MotorEncoder {
private:
    uint8_t encoderPinA;
    uint8_t encoderPinB;
    volatile int pulseCount;
    volatile int direction;
    unsigned long lastUpdateTime;
    float rpm;
    static constexpr int pulsesPerRevolution = 48;

    // Novas funções de interrupção
    static void IRAM_ATTR handleInterruptA_Rising(void *arg);
    static void IRAM_ATTR handleInterruptA_Falling(void *arg);
    static void IRAM_ATTR handleInterruptB_Rising(void *arg);
    static void IRAM_ATTR handleInterruptB_Falling(void *arg);

public:
    MotorEncoder(uint8_t pinA, uint8_t pinB);
    void begin();
    void updateSpeed();
    float getRPM();
    int getDirection();
};

#endif
