#include "drive_control.h"

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
m_direction manual_direction;
s_modes mode;
bool autoModeStarted;

void drive_control_init(){
    manual_direction = M_NONE;
    mode = S_AUTO;
    autoModeStarted = false;
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

Encoder_data getEncoderPosOfMotor(){
  Encoder_data encoder_return_data;
  encoder_return_data.left_motor = Encoder_1.getPulsePos();
  encoder_return_data.right_motor = Encoder_2.getPulsePos();
  
  return encoder_return_data;
}

void reset_encoders(){
  //Encoder_1.reset(SLOT_1);
  //Encoder_2.reset(SLOT_2);
  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);
}

void print_encoder_data(){
  Serial.print("Motor 1: ");
  Serial.println(Encoder_1.getPulsePos());
  
  Serial.print("Motor 2: ");
  Serial.println(Encoder_2.getPulsePos());
}

void move(int direction, int speed)
{
  static int offset = 10;
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == 1) { //Move forward
    leftSpeed = -speed;
    rightSpeed = speed+offset;
  } else if (direction == 2) { //Move backwards
    leftSpeed = speed;
    rightSpeed = -speed-offset;
  } else if (direction == 3) { //Turn left
    leftSpeed = -speed;
    rightSpeed = -speed-offset;
  } else if (direction == 4) { //Turn right
    leftSpeed = speed;
    rightSpeed = speed+offset;
  }
  //Update motors speed
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);
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

bool is_auto_mode_started(){
    return autoModeStarted;
}

void set_auto_mode_started(bool is_started){
    autoModeStarted = is_started;
}

m_direction get_manual_direction(){
    return manual_direction;
}

void set_manual_direction(m_direction new_manual_direction){
    manual_direction = new_manual_direction;
}
