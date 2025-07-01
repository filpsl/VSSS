#include "Robot.h"
#include "communication.h"

// Variáveis de controle
volatile int setPointRight = 0, setPointLeft = 0, directionRight = 0, directionLeft = 0;

// Buffer para média móvel (armazenando as 3 últimas leituras)
float speedRightBuffer[5] = {0, 0, 0, 0, 0};
float speedLeftBuffer[5] = {0, 0, 0, 0, 0};

// Criação do objeto do robô
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L,
              LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

// Comunicação Wi-Fi
Communication messenger(NETWORK, PASSWORD, 80); 

void setup() {
    corobeu.initializeRobot();
    Serial.begin(19200);
    messenger.begin();
}

void loop() {

        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) {
            Serial.println("Recebidados");

            // Extrai velocidade e direção de cada motor
            int speedRight = (receivedValue & 0x00FF0000) >> 16;
            directionRight = (receivedValue & 0x000000FF);

            int speedLeft = (receivedValue & 0xFF000000) >> 24;
            directionLeft = (receivedValue & 0x0000FF00) >> 8;

            setPointRight = speedRight;
            setPointLeft = speedLeft;

            Serial.printf("Setpoints -> Direita: %d (Dir: %d), Esquerda: %d (Dir: %d)\n", 
                          setPointRight, directionRight, setPointLeft, directionLeft);
            corobeu.setMotorLeft(speedLeft, directionLeft);
            corobeu.setMotorRight(speedRight, directionRight); 
            
        }
    }
