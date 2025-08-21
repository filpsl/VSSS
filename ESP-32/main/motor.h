/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.h                                      Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Class to control motor operations using ESP32 and L298N H-bridge.
 * 
 * This class provides methods to control the motor's speed and direction 
 * using PWM signals, allowing the motor to move forward or reverse.
 */
class Motor {
  /*
   * This class was created to manage motor objects and control them.
   */
private:
    /**
     * @brief Pin connected to the motor for controlling rotation in the forward direction (R).
     */
    int pin_R_;                

    /**
     * @brief Pin connected to the motor for controlling rotation in the reverse direction (L).
     */
    int pin_L_;                       

    /**
     * @brief PWM channel used to control the motor in the forward direction (R).
     */
    int channelR_;

    /**
     * @brief PWM channel used to control the motor in the reverse direction (L).
     */
    int channelL_;
    
public:
    /**
     * @brief Constructor for the Motor class.
     * 
     * Initializes the motor control pins and PWM channels for forward and reverse motion.
     * 
     * @param pin_R GPIO pin for controlling the forward (R) direction.
     * @param pin_L GPIO pin for controlling the reverse (L) direction.
     * @param channelR PWM channel for forward (R) control.
     * @param channelL PWM channel for reverse (L) control.
     */
    Motor(uint8_t pin_R, uint8_t pin_L, uint8_t channelR, uint8_t channelL);

    /**
     * @brief Controls the motor rotation using ESP32 PWM and L298N H-bridge.
     * 
     * This function controls the motor's speed and direction using the L298N H-bridge.
     * The speed ranges from 0 to 255 (8-bit integer), and the direction can be 
     * set to 1 (forward) or 0 (reverse).
     * 
     * @param speed Motor speed, a value between 0 and 255.
     * @param direction Motor direction, accepts 1 (forward) or 0 (reverse) as parameters.
     */
    void moveForward(int speed, int direction);

    /**
     * @brief Stops the motor.
     * 
     * This function sets zero on pwm pins of motor
     */
    void stop();
};



#endif