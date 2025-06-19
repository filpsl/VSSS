/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        Robot.cpp                                    Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "Robot.h"

/**
 * @brief Constructor for the Robot class.
 * 
 * Initializes the motors with specified pins and PWM channels.
 * 
 * @param pin_R1 GPIO pin for controlling the right motor direction 1.
 * @param pin_L1 GPIO pin for controlling the right motor direction 2.
 * @param pin_R2 GPIO pin for controlling the left motor direction 1.
 * @param pin_L2 GPIO pin for controlling the left motor direction 2.
 * @param encoderAL GPIO pin for receiving the A channel for the left encoder
 * @param encoderBL GPIO pin for receiving the B channel for the left encoder
 * @param encoderAR GPIO pin for receiving the A channel for the right encoder
 * @param encoderBR GPIO pin for receiving the B channel for the right encoder
 */
Robot::Robot(uint8_t pin_R1, uint8_t pin_L1, uint8_t pin_R2, uint8_t pin_L2, uint8_t encoderAL, uint8_t encoderBL, uint8_t encoderAR, uint8_t encoderBR)
    : motorRight(pin_R1, pin_L1, ROBOT_CHANEL_1R, ROBOT_CHANEL_1L),
      motorLeft(pin_R2, pin_L2, ROBOT_CHANEL_2R, ROBOT_CHANEL_2L),
      encoderLeft(encoderAL, encoderBL),
      encoderRight(encoderAR, encoderBR)  {
    this->pin_R1_ = pin_R1;
    this->pin_L1_ = pin_L1; 
    this->channelR1_ = ROBOT_CHANEL_1R;
    this->channelL1_ = ROBOT_CHANEL_1L;
    this->pin_R2_ = pin_R2;
    this->pin_L2_ = pin_L2;
    this->channelR2_ = ROBOT_CHANEL_2R;
    this->channelL2_ = ROBOT_CHANEL_2L;
    this->encoderAL = encoderAL;
    this->encoderBL = encoderBL;
    this->encoderAR = encoderAR;
    this->encoderBR = encoderBR;
    this->state_ = 0;
}

/**
 * @brief Initialize the robot.
 * 
 * Stop the motors and initialize the encoders
 */
void Robot::initializeRobot() {
    motorRight.stop();
    motorLeft.stop();
    encoderLeft.begin();
    encoderRight.begin();
}

/**
 * @brief Set the speed and direction for the right motor.
 * 
 * Controls the right motor with the given speed and direction.
 * 
 * @param speedR PWM value for the right motor speed (0 to 255).
 * @param direction Direction of the right motor (1 for forward, 0 for backward).
 */
void Robot::setMotorRight(int speedR, int direction) {
    this->motorRight.moveForward(speedR, direction);
}

/**
 * @brief Set the speed and direction for the left motor.
 * 
 * Controls the left motor with the given speed and direction.
 * 
 * @param speedL PWM value for the left motor speed (0 to 255).
 * @param direction Direction of the left motor (1 for forward, 0 for backward).
 */
void Robot::setMotorLeft(int speedL, int direction) {
    this->motorLeft.moveForward(speedL, direction);
}

/**
 * @brief Stop the specified motors.
 * 
 * Stops the motors based on the motorState. The state is a bitmask where:
 * - Bit 0 indicates the left motor.
 * - Bit 1 indicates the right motor.
 * 
 * @param Motor1 State for the left motor (0 to stop, 1 to keep running).
 * @param Motor2 State for the right motor (0 to stop, 1 to keep running).
 */
void Robot::Stop(int Motor1, int Motor2) {
    int motorState = (Motor1 << 1) | Motor2; 

    switch (motorState) {
        case 0b01:  // 0 and 1: Stop only the left motor
            motorLeft.stop();
            break;
        case 0b10:  // 1 and 0: Stop only the right motor
            motorRight.stop();
            break;
        case 0b11:  // 1 and 1: Stop both motors
            motorRight.stop();
            motorLeft.stop();
            break;
        default:    // No motors stopped if both are 0
            break;
    }
}

void Robot::updateEncoder(){
    encoderLeft.updateSpeed();
    encoderRight.updateSpeed();
}
