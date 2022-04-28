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

void drive_control_init();

void move(int direction, int speed);

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