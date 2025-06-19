#include "Robot.h"
#include "communication.h"

// Configurações do controle PID
const float kpl = 0.8394, kil = 0.1528, kdl = 0.0;
const float alfal = 1, umalfal = 0;

const float kpr = 0.8302, kir = 0.1508, kdr = 0.0;
const float alfar = 1, umalfar = 0;

const float INTEGRAL_LIMIT = 2500.0;

// Variáveis de controle
volatile int setPointRight = 0, setPointLeft = 0, directionRight = 0, directionLeft = 0;
float controlRight = 0, controlLeft = 0;
float integralRight = 0, integralLeft = 0;
float fr = 0, fanteriorr = 0, fl = 0, fanteriorl = 0;
float currentPWMRight = 0.0, currentPWMLeft = 0.0;

// Buffer para média móvel (armazenando as 3 últimas leituras)
float speedRightBuffer[5] = {0, 0, 0, 0, 0};
float speedLeftBuffer[5] = {0, 0, 0, 0, 0};

// Criação do objeto do robô
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L,
              LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

// Comunicação Wi-Fi
Communication messenger(NETWORK, PASSWORD, 80);

// Variáveis de controle de tempo
unsigned long lastMotorUpdate = 0;
unsigned long lastCommUpdate = 0;
const unsigned long motorUpdateInterval = 700; // Atualiza os motores a cada 100ms
const unsigned long commUpdateInterval = 10;   // Verifica comunicação a cada 10ms

// Função para aplicar o filtro de média móvel
float movingAverageFilter(float newReading, float *buffer) {
    buffer[4] = buffer[3];
    buffer[3] = buffer[2];
    buffer[2] = buffer[1];
    buffer[1] = buffer[0];
    buffer[0] = newReading;
    
    return ((buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4]) / 5.0);
}

void setup() {
    corobeu.initializeRobot();
    Serial.begin(19200);
    messenger.begin();
}

void loop() {
    // unsigned long currentTime = millis();

    // // Comunicação Wi-Fi
    // if (currentTime - lastCommUpdate >= commUpdateInterval) {
    //     lastCommUpdate = currentTime;
        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) {
            Serial.println("Recebidados");

            // Extrai velocidade e direção de cada motor
            int speedRight = (receivedValue & 0x00FF0000) >> 16;
            directionRight = (receivedValue & 0x000000FF);
            // "192.168.0.103"
            int speedLeft = (receivedValue & 0xFF000000) >> 24;
            directionLeft = (receivedValue & 0x0000FF00) >> 8;

            setPointRight = speedRight;
            setPointLeft = speedLeft;

            Serial.printf("Setpoints -> Direita: %d (Dir: %d), Esquerda: %d (Dir: %d)\n", 
                          setPointRight, directionRight, setPointLeft, directionLeft);
            corobeu.setMotorLeft(speedLeft, directionLeft); // comentar
            corobeu.setMotorRight(speedRight, directionRight); // comentar
        }
    }

    // // Controle dos motores
    // if (currentTime - lastMotorUpdate >= motorUpdateInterval) {
    //     lastMotorUpdate = currentTime;
        
    //     // Atualizar velocidades dos encoders
    //     corobeu.encoderLeft.updateSpeed();
    //     corobeu.encoderRight.updateSpeed();
    //     float currentSpeedRight = corobeu.encoderRight.getRPM();
    //     float currentSpeedLeft = corobeu.encoderLeft.getRPM();
    //     // Leitura dos encoders
    //     // float rawSpeedRight = corobeu.encoderRight.getRPM();
    //     // float rawSpeedLeft = corobeu.encoderLeft.getRPM();
        
    //     // // Aplicação do filtro de média móvel"192.168.0.103"
    //     // float currentSpeedRight = movingAverageFilter(rawSpeedRight, speedRightBuffer);
    //     // float currentSpeedLeft = movingAverageFilter(rawSpeedLeft, speedLeftBuffer);Due to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating thDue to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating them in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.em in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.

    //     Serial.printf("RPM R: %.2f | RPM L: %.2f\n", currentSpeedRight, currentSpeedLeft);
    //     // Controle PID para motor direito
    //     if (setPointRight == 0 || abs(setPointRight) < 70) {
    //         controlRight = integralRight = currentPWMRight = 0;
    //         // Serial.print("Deu zero");
    //         corobeu.setMotorRight(0, 0);
    //     } else {
    //         float errorRight = setPointRight - currentSpeedRight;
    //         integralRight += errorRight * 0.7;Due to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating them in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.
    //         // Serial.println(integralRight);
    //         integralRight = constrain(integralRight, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    //         fr = umalfar * fanteriorr + alfar * errorRight;
    //         float derivativeRight = 1.42857 * (fr - fanteriorr);
    //         fanteriorr = fr;

    //         float pidOutputRight = (kpr * errorRight) + (kir * integralRight) + (kdr * derivativeRight);
    //         pidOutputRight = constrain(pidOutputRight, 0, 165);

    //         corobeu.setMotorRight(((int)pidOutputRight + 70), (int)directionRight);Due to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating them in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.
    //        // corobeu.setMotorRight((int)setPointRight, (int)directionRight);
    //     }

    //     // //Controle PID para motor esquerdo
    //     if (setPointLeft == 0 || abs(setPointLeft) < 70) {
    //         controlLeft = integralLeft = currentPWMLeft = 0;
    //         corobeu.setMotorLeft(0, 0);
    //     } else {
    //         float errorLeft = setPointLeft - currentSpeedLeft;
    //         integralLeft += 0.7 * errorLeft;
    //         integralLeft = constrain(integralLeft, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    //         fl = umalfal * fanteriorl + alfal * errorLeft;
    //         float derivativeLeft = 1.42857 * (fl - fanteriorl);
    //         fanteriorl = fl;

    //         float pidOutputLeft = (kpl * errorLeft) + (kil * integralLeft) + (kdl * derivativeLeft);
    //         pidOutputLeft = constrain(pidOutputLeft, 0, 165);

    //         corobeu.setMotorLeft(((int)pidOutputLeft + 70), (int)directionLeft);
    //        // corobeu.setMotorLeft((int)setPointLeft, (int)directionLeft);

    //        // Serial.print("SetR:"); Serial.print(setPointRight);

        // }Due to the high dynamism of IEEE Very Small Size Soccer (VSSS) matches, the main differences between the teams are focused on their decision-making system, path planning methods, and control and positioning algorithms. A path planning algorithm that fits well in the context of IEEE VSSS differential robots is the univector field navigation. For the positioning of the robots the combination of Delaunay Triangulation and Gouraud Shading proves to be quite efficient. Regarding decision-making, behavior trees are a good approach, due to their modularity and easiness in changing player's actions. In addition, a system for changing player's roles is also important, in order to make a dynamic match possible. The techniques shown here are not novel in the context of robot soccer. In fact, this paper contribution lies in integrating them in a full working strategy for 5×5 VSSS, which ended up winning Latin American Robotics Competition (LARC) 2019.

    //     //Prints abaixos são para o Serial Plotter 
    //     // Serial.print("\tRPM_R:"); Serial.print(currentSpeedRight);
    //     // Serial.print("\tPID_R:"); Serial.print(controlRight); // ou pidOutputRight

    //     // Serial.print("\tSetL:"); Serial.print(setPointLeft);
    //     // Serial.print("\tRPM_L:"); Serial.print(currentSpeedLeft);
    //     // Serial.print("\tPID_L:"); Serial.println(controlLeft); // ou pidOutputLeft
    // }
// }

