#include "drive_control.h"

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
m_direction manual_direction;
s_modes mode;

void drive_control_init(){
    manual_direction = M_NONE;
    mode = S_AUTO;
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
}

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

Encoder_data get_encoder_pos_of_motor(){
  Encoder_data encoder_return_data;
  encoder_return_data.left_motor = Encoder_1.getPulsePos();
  encoder_return_data.right_motor = Encoder_2.getPulsePos();
  
  return encoder_return_data;
}

void reset_encoders(){
  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);
}

void move(int direction, int speed)
{
  int left_speed = 0;
  int right_speed = 0;
  if (direction == 1) { //Move forward
    left_speed = -speed;
    right_speed = speed;
  } else if (direction == 2) { //Move backwards
    left_speed = speed;
    right_speed = -speed;
  } else if (direction == 3) { //Turn left
    left_speed = -speed;
    right_speed = -speed;
  } else if (direction == 4) { //Turn right
    left_speed = speed;
    right_speed = speed;
  }
  //Update motors speed
  Encoder_1.setTarPWM(left_speed);
  Encoder_2.setTarPWM(right_speed);
}

s_modes get_drive_mode(){
    return mode;
}

void set_encoders_tar_pwm(int encoder_1_pwm, int encoder_2_pwm){
    Encoder_1.setTarPWM(encoder_1_pwm);
    Encoder_2.setTarPWM(encoder_2_pwm);
}

void encoders_loop(){
    Encoder_1.loop();
    Encoder_2.loop();
}

void set_drive_mode(s_modes new_mode){
    mode = new_mode;
}

m_direction get_manual_direction(){
    return manual_direction;
}

void set_manual_direction(m_direction new_manual_direction){
    manual_direction = new_manual_direction;
}
