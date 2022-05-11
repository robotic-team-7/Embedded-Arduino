#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <MeEncoderMotor.h>
#include <MeEncoderOnBoard.h>
#include <Arduino.h>
//#include <MeAuriga.h>


typedef enum {
  S_AUTO,
  S_MANUAL,
  S_TEST
} s_modes;

typedef enum {
  M_FORWARD,
  M_BACKWARDS,
  M_LEFT,
  M_RIGHT,
  M_NONE
} m_direction;

typedef struct encoder_data{
    long left_motor;
    long right_motor;
}Encoder_data;

/*
* Should be called once in the set-up code. 
* Sets-up motors and states.
* Returns nothing.
*/
void drive_control_init();

/*
*Called to set motor speed and direction. 
*Takes parameters direction and speed:
* Direction should be a value between 1-4:
*   1 = forward
*   2 = backwards
*   3 = left
*   4 = right
*Speed is a value between 0-255 where 255 is full speed.
*Returns nothing.
*/
void move(int direction, int speed);

/*
* Returns current drive mode as a s_mode enum
*/
s_modes get_drive_mode();

/*
*Sets motor speed on each motor. 
*encoder_1 is left motor
*encoder_2 is right motor
*Both takes a value between 0-255 where 255 is max speed.
*Returns nothing.
*/
void set_encoders_tar_pwm(int encoder_1, int encoder_2);

/*
*Should be called continuously to keep motors running
*Returns noting.
*/
void encoders_loop();

/*
*Interrupt function for encoder 1 (left motor) 
*Returns nothing.
*/
void isr_process_encoder1(void);

/*
*Interrupt function for encoder 2 (right motor) 
*Returns nothing.
*/
void isr_process_encoder2(void);

/*
*Sets operation mode.
*In parameter should be of type s_modes.
* S_AUTO for autonomous operation.
* S_MANUAL for manual operation.
*Returns nothing.
*/
void set_drive_mode(s_modes new_mode);

/*
*Returns the current manual direction as a m_direction.
*/
m_direction get_manual_direction();

/*
*Sets a new manual direction. 
*Takes a m_direction.
* M_FORWARD for driving forward
* M_BACKWARDS for driving backwards
* M_LEFT for driving left
* M_RIGHT for driving right
* M_NONE for standing still
*Returns nothing.
*/
void set_manual_direction(m_direction new_manual_direction);

/*
* Returns a struct containing information about registered pulses on each motor
* Suggestion to call reset_encoders() after calling this function in order to start a new messurement
* with a new starting position. Otherwise it continues to count from the same start position.
*/
Encoder_data get_encoder_pos_of_motor();

/*
*Resets the encoder motor pulse counters 
*Returns nothing
*/
void reset_encoders();

#endif
