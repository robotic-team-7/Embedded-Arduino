#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <MeEncoderMotor.h>
#include <MeEncoderOnBoard.h>
#include <Arduino.h>
//#include <MeAuriga.h>


typedef enum {
  S_AUTO,
  S_MANUAL
} s_modes;

typedef enum {
  M_FORWARD,
  M_BACKWARDS,
  M_LEFT,
  M_RIGHT,
  M_NONE
} m_direction;

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
*/
void move(int direction, int speed);

/*
* Returns current drive mode as a s_mode enum
*/
s_modes get_drive_mode();

void set_encoders_tar_pwm(int encoder_1, int encoder_2);

void encoders_loop();

void isr_process_encoder1(void);

void isr_process_encoder2(void);

void set_drive_mode(s_modes new_mode);

bool is_auto_mode_started();

void set_auto_mode_started(bool is_started);

m_direction get_manual_direction();

void set_manual_direction(m_direction new_manual_direction);

#endif
