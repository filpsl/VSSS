/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        Roboy.h                                      Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#ifndef ROBO_H
#define ROBO_H

#include "motor.h"
#include "MotorEncoder.h"


/**
 * @brief Class for controlling a robot with two motors.
 * 
 * This class allows for the control of two motors, including setting their 
 * speed and direction, as well as stopping them individually or together.
 */
class Robot {
public:
    /**
     * @brief Constructor for the Robot class.
     * 
     * Initializes the Robot object with the specified pins and channels for 
     * controlling two motors.
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
    Robot(uint8_t pin_R1, uint8_t pin_L1, uint8_t pin_R2, uint8_t pin_L2, uint8_t encoderAL, uint8_t encoderBL, uint8_t encoderAR, uint8_t encoderBR);

    /**
     * @brief Initializes the robot's motors and settings.
     * 
     * Sets up the motor pins and PWM channels, preparing the robot for operation.
     */
    void initializeRobot();

    /**
     * @brief Sets the speed and direction of the right motor.
     * 
     * Configures the speed and direction of the right motor using PWM signals.
     * 
     * @param speed Speed of the motor, a value between 0 and 255.
     * @param direction Direction of the motor, where 1 is forward and 0 is reverse.
     */
    void setMotorRight(int speed, int direction);

    /**
     * @brief Sets the speed and direction of the left motor.
     * 
     * Configures the speed and direction of the left motor using PWM signals.
     * 
     * @param speed Speed of the motor, a value between 0 and 255.
     * @param direction Direction of the motor, where 1 is forward and 0 is reverse.
     */
    void setMotorLeft(int speed, int direction);

    /**
     * @brief Stops the motors based on the specified parameters.
     * 
     * Stops the motors based on the values of Motor1 and Motor2. If Motor1 is 1 
     * and Motor2 is 0, only the right motor stops. If Motor1 is 0 and Motor2 is 
     * 1, only the left motor stops. If both are 1, both motors stop.
     * 
     * @param Motor1 Control flag for the right motor (1 to stop, 0 to keep running).
     * @param Motor2 Control flag for the left motor (1 to stop, 0 to keep running).
     */
    void Stop(int Motor1, int Motor2);

    void updateEncoder();

    Motor motorRight; /**< Motor object controlling the right motor. */
    Motor motorLeft;  /**< Motor object controlling the left motor. */
    // QuadratureEncoder encoderRight;
    // QuadratureEncoder encoderLeft;
    MotorEncoder encoderRight;
    MotorEncoder encoderLeft;

private:
    uint8_t pin_R1_;  /**< GPIO pin for the right motor direction 1. */
    uint8_t pin_L1_;  /**< GPIO pin for the left motor direction 1. */
    uint8_t channelR1_; /**< PWM channel for the right motor direction 1. */
    uint8_t channelL1_; /**< PWM channel for the left motor direction 1. */
    uint8_t pin_R2_;  /**< GPIO pin for the right motor direction 2. */
    uint8_t pin_L2_;  /**< GPIO pin for the left motor direction 2. */
    uint8_t channelR2_; /**< PWM channel for the right motor direction 2. */
    uint8_t channelL2_; /**< PWM channel for the left motor direction 2. */
    uint8_t state_;   /**< State of the robot, used for tracking status or mode. */
    uint8_t encoderAL; /**< GPIO pin for receiving the A channel of the left encoder */
    uint8_t encoderBL; /**< GPIO pin for receiving the B channel of the left encoder */
    uint8_t encoderAR; /**< GPIO pin for receiving the A channel of the right encoder */
    uint8_t encoderBR; /**< GPIO pin for receiving the B channel of the right encoder */
};



#endif // ROBO_H
