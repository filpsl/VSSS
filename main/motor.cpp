/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.cpp                                    Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "motor.h"

/**
 * @brief Constructor for the Motor class.
 * 
 * Initializes the motor pins and PWM channels.
 * 
 * @param pin_R Pin number for the right motor.
 * @param pin_L Pin number for the left motor.
 * @param channelR PWM channel for the right motor.
 * @param channelL PWM channel for the left motor.
 */


Motor::Motor(uint8_t pin_R, uint8_t pin_L, uint8_t channelR, uint8_t channelL) {
    this->pin_R_ = pin_R;
    this->pin_L_ = pin_L;
    this->channelR_ = channelR;
    this->channelL_ = channelL;

    // NÃO FUNCIONANDO DIREITO 
    ledcAttachChannel(pin_R_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION, channelR_);
    ledcAttachChannel(pin_L_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION, channelL_);

    ledcAttach(pin_R_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    ledcAttach(pin_L_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
}

/**
 * @brief Move the motor forward with specified speed and direction.
 * 
 * Sets the PWM values to control the speed and direction of the motors.
 * 
 * @param speed PWM value for motor speed (0 to 255).
 * @param direction Direction of movement (1 for forward, 0 for backward).
 */
void Motor::moveForward(int speed, int direction) {
    if (direction == 1) {
        ledcWrite(pin_R_, LOW);
        ledcWrite(pin_L_, speed);        
    } else if (direction == 0) {
        ledcWrite(pin_R_, speed);
        ledcWrite(pin_L_, LOW);              
    }
}

/**
 * @brief Stop the motor by setting PWM values to zero.
 * 
 * Disables both motors by setting PWM values to 0.
 */

void Motor::stop() {
    ledcWrite(pin_R_, 0x00);
    ledcWrite(pin_L_, 0x00);
}
