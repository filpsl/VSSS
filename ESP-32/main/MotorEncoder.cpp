#include "MotorEncoder.h"

// Construtor
MotorEncoder::MotorEncoder(uint8_t pinA, uint8_t pinB)
    : encoderPinA(pinA), encoderPinB(pinB), pulseCount(0), direction(0), lastUpdateTime(0), rpm(0.0) {}

// Interrupção para borda de subida no pino A
void IRAM_ATTR MotorEncoder::handleInterruptA_Rising(void* arg) {
    MotorEncoder* instance = static_cast<MotorEncoder*>(arg);
    if(digitalRead(instance->encoderPinA)){
      int stateB = digitalRead(instance->encoderPinB);
      instance->direction = (stateB == HIGH) ? 1 : -1; 
      instance->pulseCount++;
    }
}

// Interrupção para borda de descida no pino A
void IRAM_ATTR MotorEncoder::handleInterruptA_Falling(void* arg) {
    MotorEncoder* instance = static_cast<MotorEncoder*>(arg);
    if(!digitalRead(instance->encoderPinA)){
      int stateB = digitalRead(instance->encoderPinB);
      instance->direction = (stateB == LOW) ? 1 : -1;  
      instance->pulseCount++;
    }
}

// Interrupção para borda de subida no pino B
void IRAM_ATTR MotorEncoder::handleInterruptB_Rising(void* arg) {
    MotorEncoder* instance = static_cast<MotorEncoder*>(arg);
    if(digitalRead(instance->encoderPinB)){
      // int stateA = digitalRead(instance->encoderPinA);
      // instance->direction = (stateA == LOW) ? 1 : -1;  
      instance->pulseCount++;
    }
}

// Interrupção para borda de descida no pino B
void IRAM_ATTR MotorEncoder::handleInterruptB_Falling(void* arg) {
    MotorEncoder* instance = static_cast<MotorEncoder*>(arg);
    if(!digitalRead(instance->encoderPinB)){
      // int stateA = digitalRead(instance->encoderPinA);
      // instance->direction = (stateA == HIGH) ? 1 : -1;  
      instance->pulseCount++;
    }
    
}

// Inicializa o encoder e configura as interrupções
void MotorEncoder::begin() {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

    attachInterruptArg(digitalPinToInterrupt(encoderPinA), handleInterruptA_Rising, this, RISING);
    attachInterruptArg(digitalPinToInterrupt(encoderPinA), handleInterruptA_Falling, this, FALLING);
    attachInterruptArg(digitalPinToInterrupt(encoderPinB), handleInterruptB_Rising, this, RISING);
    attachInterruptArg(digitalPinToInterrupt(encoderPinB), handleInterruptB_Falling, this, FALLING);

    lastUpdateTime = millis();
}

// Atualiza a velocidade do motor
void MotorEncoder::updateSpeed() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;
    // static int lastCount = 0;

    if (elapsedTime >= 700) { // Atualiza a cada 500 ms
        noInterrupts();
        int count = pulseCount; // - lastCount;
        pulseCount = 0;
        interrupts();

        rpm = (2 * count / (float)pulsesPerRevolution) * (60000.0 / elapsedTime);

        if (count == 0) {
            direction = 0;
        }

        lastUpdateTime = currentTime;
        // lastCount = count;
    }
}

// Retorna a velocidade atual em RPM
float MotorEncoder::getRPM() {
    return rpm;
}

// Retorna a direção de rotação
int MotorEncoder::getDirection() {
    return direction;
}
