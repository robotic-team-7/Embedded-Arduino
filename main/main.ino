#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <stdlib.h>
#include <time.h>
#include "positioning.h"
#include "led.h"
#include "drive_control.h"

MeLightSensor lightsensor_12(12);
MeLineFollower linefollower_9(9);
MeGyro gyro(0, 0x69);

unsigned long timestamp_last_sample = 0;
int speed_manual = 50;

typedef enum {
  LTS_NOT_TRIGGERED,
  LTS_STOPPING_ROBOT,
  LTS_WAITING_ON_PIC_TAKEN,
  LTS_TURNING_AWAY_FROM_OBSTACLE
} s_lidar_triggered;

s_lidar_triggered lidar_triggered_states = LTS_NOT_TRIGGERED;

void _delay(float seconds) {
  if (seconds < 0.0) {
    seconds = 0.0;
  }
  long end_time = millis() + seconds * 1000;
  while (millis() < end_time) _loop();
}

void check_serial_input() {
  if (Serial.available() > 1) {
    int available_serial = Serial.available();
    char buff[available_serial];
    
    for(int i = 0; i < available_serial + 1; i++){
      buff[i] = Serial.read();
    }

    if (buff[0] == 'A' && buff[1] == 'M' && get_drive_mode() != S_AUTO) {
      lidar_triggered_states = LTS_NOT_TRIGGERED;
      set_drive_mode(S_AUTO);
    }
    else if (buff[0] == 'M' && buff[1] == 'M' && get_drive_mode() != S_MANUAL) {
      set_drive_mode(S_MANUAL);
    }
    else if (get_drive_mode() == S_MANUAL){
      if(buff[0] == 'M' && buff[1] == 'F'){
        set_manual_direction(M_FORWARD);
      }
      else if(buff[0] == 'M' && buff[1] == 'B'){
        set_manual_direction(M_BACKWARDS);
      }
      else if(buff[0] == 'M' && buff[1] == 'L'){
        set_manual_direction(M_LEFT);
      }
      else if(buff[0] == 'M' && buff[1] == 'R'){
        set_manual_direction(M_RIGHT);
      }
      else if(buff[0] == 'M' && buff[1] == 'S'){
        set_manual_direction(M_NONE);
      }
      else if(buff[0] == 'G' && buff[1] == 'C'){      //DEBUG PURPOSES
        send_coordinates();
      }
      else if(buff[1] == 'I' && buff[0] == 'S')
      {
        if(speed_manual != 100)
          speed_manual += 25;
      }
      else if(buff[1] == 'D' && buff[0] == 'S')
      {
        if(speed_manual != 25)
          speed_manual -= 25;
      }
    }
    else if(buff[0] == 'L' && buff[1] == 'T'){ //When in Auto mode, LIDAR triggered command from Raspberry
      lidar_triggered_states = LTS_STOPPING_ROBOT;
    }
    else if(lidar_triggered_states == LTS_WAITING_ON_PIC_TAKEN && buff[0] == 'P' && buff[1] == 'T'){
      lidar_triggered_states = LTS_TURNING_AWAY_FROM_OBSTACLE;
    }
    else{
      Serial.print("UC");
    }
  }
}

void setup() {
  Serial.begin(115200);

  gyro.begin();

  randomSeed((unsigned long)(lightsensor_12.read() * 123456));
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  positioning_init(&gyro);
  led_init();
  drive_control_init();
  reset_encoders();

  while (1) {
    check_serial_input();

    if (get_drive_mode() == S_AUTO) {
      auto_mode();
    }
    else if (get_drive_mode() == S_MANUAL) {
      manual_mode();
    }

    _loop();
  }
}

void auto_mode() {
  if (linefollower_9.readSensors() == 0.000000) {
    line_follower_triggered();
  }
  else if(lidar_triggered_states != LTS_NOT_TRIGGERED){
     lidar_triggered();
  }
  else {
    auto_drive_forward();
  }
}

void manual_mode() {
  set_leds_yellow();
  calculate_new_coordinates_interval();
  send_latest_coordinates_interval();
  
  switch(get_manual_direction()){
    case M_NONE:
      move(1,0);
      break;
    case M_FORWARD:
      move(1, speed_manual/ 100.0 * 255);
      break;
    case M_BACKWARDS:
      move(2, speed_manual / 100.0 * 255);
      break;
    case M_LEFT:
      move(3, 50 / 100.0 * 255);
      reset_encoders(); //Turning is done in place
      break;
    case M_RIGHT:
      move(4, 50 / 100.0 * 255);
      reset_encoders();
      break;
  }
}

void line_follower_triggered() {
  //Set motor speed to 0 in 0.5 seconds
  set_encoders_tar_pwm(0, 0);

  _delay(1);  //Stop motors completely
  calculate_new_coordinates();
  send_latest_coordinates_interval();

  set_leds_red();

  //Go backwards in 1 second, 50% of maximum speed
  move(2, 50 / 100.0 * 255);
  _delay(1);
  move(2, 0);
  _delay(1);  //Let motors come to a complete stop
  calculate_new_coordinates();
  send_latest_coordinates_interval();

  //Choose left or right randomly and turn in  2 second 50% of speed
  int random_direction = rand() % 2 + 3;
  move(random_direction, 50 / 100.0 * 255);
  _delay(1);
  move(random_direction, 0);
  reset_encoders(); //Rotation is stationary and therefore not counted as moving
}

void lidar_triggered() {
  if(lidar_triggered_states == LTS_STOPPING_ROBOT){
    //Set motor speed to 0 in 0.5 seconds
    set_encoders_tar_pwm(0, 0);
    _delay(1);  //Let wheels halt completely
    calculate_new_coordinates();
    send_latest_coordinates_interval();
  
    //Show red color on LED-ring
    set_leds_blue();
  
    //Tell Raspberry Pi that we have stoped due to lidar triggered
    Serial.print("LOK");
    lidar_triggered_states = LTS_WAITING_ON_PIC_TAKEN;
  }
  else if(lidar_triggered_states == LTS_WAITING_ON_PIC_TAKEN){
    return;
  }
  else if(lidar_triggered_states == LTS_TURNING_AWAY_FROM_OBSTACLE){
    //Go backwards in 0.5 seconds, 50% of maximum speed
    move(2, 50 / 100.0 * 255);
    _delay(0.5);
    move(2, 0);
    _delay(1);  //To let wheels come to a complete halt
    calculate_new_coordinates();
    send_latest_coordinates_interval();
  
    //Choose left or right randomly and turn in  1 second 50% of speed
    int random_direction = rand() % 2 + 3;
    move(random_direction, 50 / 100.0 * 255);
    _delay(1);
    move(random_direction, 0);
    lidar_triggered_states = LTS_NOT_TRIGGERED;
    reset_encoders(); //Rotating is not changing the position
  }
}

void auto_drive_forward() {
  //Show green color on LED-ring
  set_leds_green();

  calculate_new_coordinates_interval();
  send_latest_coordinates_interval();

  //Go forward, 50% of maximum speed
  move(1, 50 / 100.0 * 255);
}

void _loop() {
  encoders_loop();
  gyro.update();
}

void loop() {
  srand(time(NULL));
  _loop();
}
