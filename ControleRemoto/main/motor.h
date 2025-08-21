/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.h                                      Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2023/03/31          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "behavior.h"

class Motor {
  /*
    This class was been create to makes Motors objects and control them
  */
private:
    int pin_R;                
    int pin_L;                 
    bool stateMotor;
    int channelR;
    int channelL;
public:
    Motor(int pin_R, int pin_L, int channelR, int channelL);
    void moveForward(int speed, int direction);
    void stop();
};

#endif