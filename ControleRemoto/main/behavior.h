/**********************************************************************************************/
/*                                                                                            */
/*  File: behavior.h                                                                          */
/*  Author: Luiz Felipe                                                                       */
/*  Description:                                                                              */
/*      This file contains configuration settings for controlling RGB LEDs and motors, as     */
/*      well as defining constants related to PWM frequencies, resolutions, and pin mappings. */
/*                                                                                            */
/*  Created: 2023/02/07 by Luiz F.                                                            */
/*  Updated: 2023/03/31 by Luiz F.                                                            */
/*  Contact: luizcn123@hotmail.com                                                            */
/*  Location: DF, BRAZIL                                                                      */
/*  All rights reserved                                                                       */
/**********************************************************************************************/
#ifndef behavior_h
#define behavior_h

// RGB LED CONFIG FOR PINS COLORS
#define RED_PIN                 18         // GPIO 18 
#define GREEN_PIN               5          // GPIO 5    
#define BLUE_PIN                17         // GPIO 17

// SETTINGS PWM LED RGB
#define PWM_FREQUENCY_LED       500        // Frequency 500 hz
#define PWM_RESOLUTION_LED      8          // 2^(8-1) = 255
#define PWM_CHANNEL_LED_RED     0          // Channel 0
#define PWM_CHANNEL_LED_GREEN   1          // Channel 1
#define PWM_CHANNEL_LED_BLUE    2          // Channel 2

// SETTINGS MOTOR M1 PINs
#define MOTOR1_PIN_R            33          // GPIO 19    
#define MOTOR1_PIN_L            25          // GPIO 21

// SETTINGS MOTOR M2 PINs
#define MOTOR2_PIN_R            26          // GPIO 12    
#define MOTOR2_PIN_L            27          // GPIO 13

// SETTINS MOTORs PWM properties
#define PWM_MOTOR_FREQUENCY     5000        // Frequency 5 khz
#define PWM_MOTOR_RESOLUTION    8           // 2^(8-1) = 255
#define PWM_MOTOR1_CHANNEL_R    3           // Channel 3
#define PWM_MOTOR1_CHANNEL_L    4           // Channel 4
#define PWM_MOTOR2_CHANNEL_R    5           // Channel 5
#define PWM_MOTOR2_CHANNEL_L    6           // Channel 6


// SETTINGS PS3
// Aqui tu tem que mudar o endereço mac do controle pra funcionar
#define CONTROLS_MAC_ADDRESS   "74:85:76:73:65:71"
// 08:a9:5a:a7:97:47
#endif