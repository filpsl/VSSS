// --- CÓDIGO COMPLETO COM LOG DE DEPURAÇÃO ---

#include "behavior.h"
#include "led_rgb.h"
#include "motor.h"
#include <Ps3Controller.h>

Motor motor1(MOTOR1_PIN_R, MOTOR1_PIN_L, PWM_MOTOR1_CHANNEL_R, PWM_MOTOR1_CHANNEL_L);
Motor motor2(MOTOR2_PIN_R, MOTOR2_PIN_L, PWM_MOTOR2_CHANNEL_R, PWM_MOTOR2_CHANNEL_L);

// --- FUNÇÃO AUXILIAR PARA CONTROLAR OS MOTORES ---
void setMotorSpeed(Motor& motor, int speed, int motor_id) {
    // =================================================================
    // PONTO DE AJUSTE 1: DIREÇÃO DOS MOTORES
    // Se o robô anda para trás quando você comanda para frente,
    // inverta o sinal desta variável para o motor correspondente.
    // Exemplo: se o motor 1 está invertido, mude (motor_id == 1) ? -1 : 1; para (motor_id == 1) ? 1 : 1;
    // =================================================================
    int dir_frente = (motor_id == 1) ? -1 : 1;

    if (speed > 0) {
        motor.moveForward(speed, dir_frente);
    } else if (speed < 0) {
        motor.moveForward(abs(speed), -dir_frente);
    } else {
        motor.stop();
    }
}

// --- LÓGICA DE CONTROLE PRINCIPAL ---
// --- CÓDIGO COMPLETO COM FATOR DE GIRO AJUSTÁVEL ---

// ... (includes e declarações dos motores permanecem os mesmos) ...
// ... (a função setMotorSpeed permanece a mesma) ...

void notify() {
    // --- Etapa 1 e 2: Leitura e Zona Morta ---
    int joyY = Ps3.data.analog.stick.ly;
    int joyX = Ps3.data.analog.stick.lx;
    int deadZone = 15;
    int processedJoyY = (abs(joyY) < deadZone) ? 0 : joyY;
    int processedJoyX = (abs(joyX) < deadZone) ? 0 : joyX;

// =================================================================
    // PASSO 3 e 4 AVANÇADOS: Mixagem com Curva de Sensibilidade
    // =================================================================
    // Mapeamento linear para FRENTE/TRÁS
    int moveSpeed = map(processedJoyY, -128, 127, 255, -255);

    // Mapeamento NÃO-LINEAR para ROTAÇÃO
    // 1. Normalizamos o valor do joystick para um intervalo de -1.0 a 1.0
    float normalizedX = processedJoyX / 127.0;
    
    // 2. Aplicamos uma função de potência (cúbica é ótima).
    //    Isso cria a curva: movimentos pequenos têm pouco efeito, movimentos grandes têm muito efeito.
    float curveX = pow(normalizedX, 5);
    
    // 3. Mapeamos o resultado da curva de volta para a velocidade de rotação.
    //    Podemos também definir uma velocidade máxima de giro aqui (ex: 200 em vez de 255).
    int maxTurnSpeed = 100;
    int turnSpeed = curveX * maxTurnSpeed;

    // A mixagem continua a mesma
    int speedMotor1 = moveSpeed - turnSpeed;
    int speedMotor2 = moveSpeed + turnSpeed;
    // =================================================================

    // --- Etapa 5: Limitar os valores ---
    speedMotor1 = constrain(speedMotor1, -255, 255);
    speedMotor2 = constrain(speedMotor2, -255, 255);

    // --- Etapa 6: Enviar comandos ---
    setMotorSpeed(motor1, speedMotor1, 1);
    setMotorSpeed(motor2, speedMotor2, 2);

    // --- LOG DETALHADO PARA O MONITOR SERIAL ---
    Serial.print("Move: "); Serial.print(moveSpeed);
    Serial.print("\t Turn: "); Serial.print(turnSpeed);
    Serial.print("  ||  ");
    Serial.print("==> M1: "); Serial.print(speedMotor1);
    Serial.print("\t M2: "); Serial.println(speedMotor2);
}


void onConnect() {
    Serial.println("Conectado.");
}

void setup() {
    Serial.begin(115200); // Aumentei a velocidade para um log mais rápido
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("F4:65:0B:46:C6:A2");
    Serial.println("Pronto. Realize os testes de calibração.");
}

void loop() {}