#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <stdlib.h>
#include <time.h>
MeLightSensor lightsensor_12(12);
MeUltrasonicSensor ultrasonic_10(10);
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

typedef enum{
  S_AUTO,
  S_MANUAL
} s_modes;

s_modes mode = S_AUTO;

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }else{
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus();
  }else{
    Encoder_2.pulsePosPlus();
  }
}
void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if(direction == 1){ //Move forward
    leftSpeed = -speed;
    rightSpeed = speed;
  }else if(direction == 2){ //Move backwards
    leftSpeed = speed;
    rightSpeed = -speed;
  }else if(direction == 3){ //Turn left
    leftSpeed = -speed;
    rightSpeed = -speed;
  }else if(direction == 4){ //Turn right
    leftSpeed = speed;
    rightSpeed = speed;
  }
  //Update motors speed
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);
}

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void serialCheckState(){
  if(Serial.available() > 0){
    int availableSerial = Serial.available();
    char buff[availableSerial];
    Serial.readString().toCharArray(buff, availableSerial + 1);
  
    if(strcmp(buff, "auto") == 0){
      mode = S_AUTO;
    }
    else if(strcmp(buff, "manual") == 0){
      mode = S_MANUAL;
    }
    
  }
}

void setup() {
  Serial.begin(9600);

  gyro.begin();
  
  randomSeed((unsigned long)(lightsensor_12.read() * 123456));
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  rgbled_0.setpin(44);
  rgbled_0.fillPixelsBak(0, 2, 1);
  
  while(1) {
    serialCheckState();
    if(mode == S_AUTO){
      autoMode();
    }
    else if(mode == S_MANUAL){
      manualMode();
    }
    _loop();
  }
}

void autoMode(){
  if(ultrasonic_10.distanceCm() < 20){
    ultrasonicTriggered();
  }
  else if(linefollower_9.readSensors() == 0.000000){
    lineFollowerTriggered();

  }else{
    autoDriveForward();   
  }
}

void manualMode(){
  
}

void ultrasonicTriggered(){
  //Set motor speed to 0 in 0.5 seconds
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  _delay(0.5);
  
  //Show blue color on LED-ring
  rgbled_0.setColor(0,ledBlue[0],ledBlue[1],ledBlue[2]);
  rgbled_0.show();
  
  //Move backwards in 0.5 seconds, 50% of maximum speed
  move(2, 50 / 100.0 * 255);
  _delay(0.5);
  move(2, 0);
  
  //Notify Raspberry Pi that Ultrasonic sensor has observed an obsticle 
  Serial.write("hit");
  _delay(1); //Might not needed
  //Await response from Raspberry Pi that picture is captured
  while(1){
    if(Serial.available() == 4){
      int availableSerial = Serial.available();
      char buff[availableSerial];
      Serial.readString().toCharArray(buff, availableSerial + 1);
  
      if(strcmp(buff, "done") == 0){
        break;
      }
    }
  }
  
  //Choose left or right randomly and turn in  1 second 50% of speed
  int randomDirection = rand() % 2 + 3;
  move(randomDirection, 50 / 100.0 * 255);
  _delay(1);
  move(randomDirection, 0);
}

void lineFollowerTriggered(){
  //Set motor speed to 0 in 0.5 seconds
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  _delay(0.5);
  
  //Show red color on LED-ring
  rgbled_0.setColor(0,ledRed[0],ledRed[1],ledRed[2]);
  rgbled_0.show();
  
  //Go backwards in 0.5 seconds, 50% of maximum speed
  move(2, 50 / 100.0 * 255);
  _delay(0.5);
  move(2, 0);
  
  //Choose left or right randomly and turn in  1 second 50% of speed
  int randomDirection = rand() % 2 + 3;
  move(randomDirection, 50 / 100.0 * 255);
  _delay(1);
  move(randomDirection, 0);
}

void autoDriveForward(){
  //Show green color on LED-ring
  rgbled_0.setColor(0,ledGreen[0],ledGreen[1],ledGreen[2]);
  rgbled_0.show();
  
  //Go forward, 50% of maximum speed
  move(1, 50 / 100.0 * 255);
}

//For debug purpose
void print_gyro_values(){
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
