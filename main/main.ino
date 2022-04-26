#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <stdlib.h>
#include <time.h>
#include "positioning.h"
MeLightSensor lightsensor_12(12);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed rgbled_0(0, 12);
MeLineFollower linefollower_9(9);
MeGyro gyro(0, 0x69);

//Adjust brightness of LED-ring. 0.0 = minimum brightness, 1.0 = maximum brightness
float led_brightness = 0.1;

//Define standard colors and adjust brightness
int ledRed[3] = {(int)(255 * led_brightness), 0, 0};
int ledGreen[3] = {0, (int)(255 * led_brightness), 0};
int ledBlue[3] = {0, 0, (int)(255 * led_brightness)};
int ledYellow[3] = {(int)(255 * led_brightness), (int)(255 * led_brightness), 0};

unsigned long timestamp_last_sample = 0;

typedef enum {
  S_AUTO,
  S_MANUAL
} s_modes;

s_modes mode = S_MANUAL;

typedef enum {
  M_FORWARD,
  M_BACKWARDS,
  M_LEFT,
  M_RIGHT,
  M_NONE
} m_direction;

m_direction manual_direction = M_NONE;

bool autoModeStarted = false;

typedef enum {
  LTS_NOT_TRIGGERED,
  LTS_STOPPING_ROBOT,
  LTS_WAITING_ON_PIC_TAKEN,
  LTS_TURNING_AWAY_FROM_OBSTACLE
} s_lidar_triggered;

s_lidar_triggered lidar_triggered_states = LTS_NOT_TRIGGERED;

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
void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == 1) { //Move forward
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if (direction == 2) { //Move backwards
    leftSpeed = speed;
    rightSpeed = -speed;
  } else if (direction == 3) { //Turn left
    leftSpeed = -speed;
    rightSpeed = -speed;
  } else if (direction == 4) { //Turn right
    leftSpeed = speed;
    rightSpeed = speed;
  }
  //Update motors speed
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);
}

void _delay(float seconds) {
  if (seconds < 0.0) {
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void checkSerialInput() {
  if (Serial.available() > 1) {
    int availableSerial = Serial.available();
    char buff[availableSerial];
    
    for(int i = 0; i < availableSerial + 1; i++){
      buff[i] = Serial.read();
    }

    if (buff[0] == 'A' && buff[1] == 'M' && mode != S_AUTO) {
      lidar_triggered_states = LTS_NOT_TRIGGERED;
      autoModeStarted = false;
      mode = S_AUTO;
    }
    else if (buff[0] == 'M' && buff[1] == 'M' && mode != S_MANUAL) {
      mode = S_MANUAL;
    }
    else if (mode == S_MANUAL){
      if(buff[0] == 'M' && buff[1] == 'F'){
        manual_direction = M_FORWARD;
      }
      else if(buff[0] == 'M' && buff[1] == 'B'){
        manual_direction = M_BACKWARDS;
      }
      else if(buff[0] == 'M' && buff[1] == 'L'){
        manual_direction = M_LEFT;
      }
      else if(buff[0] == 'M' && buff[1] == 'R'){
        manual_direction = M_RIGHT;
      }
      else if(buff[0] == 'M' && buff[1] == 'S'){
        manual_direction = M_NONE;
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

void debugTrackingPrint(unsigned long timestamp, float distance) {
  Serial.print("Timestamp: ");
  Serial.print(get_time_passed(timestamp) / 1000);
  Serial.print("\n");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("\n");
  Serial.print(" RPM: ");
  Serial.print(Encoder_1.getCurrentSpeed());
  Serial.print("\n");
  Serial.print("GYRO Z: ");
  Serial.print(gyro.getAngle(3));
  Serial.print("\n");
  Serial.print( "X: ");
  Serial.print(getCoordinateX());
  Serial.print( "Y: ");
  Serial.print(getCoordinateY());
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
}

void setup() {
  Serial.begin(115200);

  gyro.begin();

  init(&Encoder_1, &gyro);

  randomSeed((unsigned long)(lightsensor_12.read() * 123456));
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  rgbled_0.setpin(44);
  rgbled_0.fillPixelsBak(0, 2, 1);

  while (1) {
    checkSerialInput();

    if (mode == S_AUTO) {
      autoMode();
    }
    else if (mode == S_MANUAL) {
      manualMode();
    }

    _loop();
  }
}

void autoMode() {
  //Needed in order to take an accurate timestamp when starting autoMode
  if(autoModeStarted == false){
    setTimestamp();
    autoModeStarted = true;
  }
  
  if (linefollower_9.readSensors() == 0.000000) {
    lineFollowerTriggered();
  }
  else if(lidar_triggered_states != LTS_NOT_TRIGGERED){
     lidarTriggered();
  }
  else {
    autoDriveForward();
  }
}

void manualMode() {
  rgbled_0.setColor(0, ledYellow[0], ledYellow[1], ledYellow[2]);
  rgbled_0.show();
  
  switch(manual_direction){
    case M_NONE:
      move(1,0);
      break;
    case M_FORWARD:
      move(1, 50 / 100.0 * 255);
      break;
    case M_BACKWARDS:
      move(2, 50 / 100.0 * 255);
      break;
    case M_LEFT:
      move(3, 50 / 100.0 * 255);
      break;
    case M_RIGHT:
      move(4, 50 / 100.0 * 255);
      break;
  }
}

void lineFollowerTriggered() {
  //Set motor speed to 0 in 0.5 seconds
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  registerPositionChange(20.0);
  _delay(0.5);

  //Show red color on LED-ring
  rgbled_0.setColor(0, ledRed[0], ledRed[1], ledRed[2]);
  rgbled_0.show();

  //Go backwards in 0.5 seconds, 50% of maximum speed
  setTimestamp();
  move(2, 50 / 100.0 * 255);
  _delay(0.5);
  registerPositionChange(-1*20.0);
  move(2, 0);

  //Choose left or right randomly and turn in  2 second 50% of speed
  int randomDirection = rand() % 2 + 3;
  move(randomDirection, 50 / 100.0 * 255);
  _delay(2);
  move(randomDirection, 0);
  setTimestamp();
}

void lidarTriggered() {
  if(lidar_triggered_states == LTS_STOPPING_ROBOT){
    //Set motor speed to 0 in 0.5 seconds
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    registerPositionChange(20.0);
    _delay(0.5);
  
    //Show red color on LED-ring
    rgbled_0.setColor(0, ledBlue[0], ledBlue[1], ledBlue[2]);
    rgbled_0.show();
  
    //Tell Raspberry Pi that we have stoped due to lidar triggered
    Serial.print("LOK");
    lidar_triggered_states = LTS_WAITING_ON_PIC_TAKEN;
  }
  else if(lidar_triggered_states == LTS_WAITING_ON_PIC_TAKEN){
    return;
  }
  else if(lidar_triggered_states == LTS_TURNING_AWAY_FROM_OBSTACLE){
    //Go backwards in 0.5 seconds, 50% of maximum speed
    setTimestamp();
    move(2, 50 / 100.0 * 255);
    _delay(0.5);
    registerPositionChange(-1*20.0);
    move(2, 0);
  
    //Choose left or right randomly and turn in  1 second 50% of speed
    int randomDirection = rand() % 2 + 3;
    move(randomDirection, 50 / 100.0 * 255);
    _delay(1);
    move(randomDirection, 0);
    setTimestamp();
    lidar_triggered_states = LTS_NOT_TRIGGERED;
  }
}

void autoDriveForward() {
  //Show green color on LED-ring
  rgbled_0.setColor(0, ledGreen[0], ledGreen[1], ledGreen[2]);
  rgbled_0.show();

  //Sample coordinates and send every second
  if(millis() > timestamp_last_sample + 250){
    registerPositionChange(20.0);
    timestamp_last_sample = millis();
  }

  //Go forward, 50% of maximum speed
  move(1, 50 / 100.0 * 255);
}

//For debug purpose
void print_gyro_values() {
  Serial.print("X: ");
  Serial.print(gyro.getAngle(1));
  Serial.print("Y: ");
  Serial.print(gyro.getAngle(2));
  Serial.print("Z: ");
  Serial.print(gyro.getAngle(3));
  Serial.print("\n");
  Serial.print("X GYRO: ");
  Serial.print(gyro.getGyroX());
  Serial.print("Y GYRO: ");
  Serial.print(gyro.getGyroY());
  Serial.print("\n");
  Serial.print("\n");

}

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
  gyro.update();
  //print_gyro_values(); For debug purpose
}

void loop() {
  srand(time(NULL));
  _loop();
}
