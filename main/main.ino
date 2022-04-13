#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>

#include "positioning.h"



MeLightSensor lightsensor_12(12);
MeUltrasonicSensor ultrasonic_10(10);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed rgbled_0(0, 12);
MeLineFollower linefollower_9(9);

char* readSerialBuffer(int bufferAvailableSize);

//Adjust brightness of LED-ring. 0.0 = minimum brightness, 1.0 = maximum brightness
float led_brightness = 0.1;

//Define standard colors and adjust brightness
int ledRed[3] = {(int)(255 * led_brightness), 0, 0};
int ledGreen[3] = {0, (int)(255 * led_brightness), 0};
int ledBlue[3] = {0, 0, (int)(255 * led_brightness)};

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

void clearSerialBuffer(){
  while(Serial.available() > 0){
    char t = Serial.read();
    //Serial.print(t); 
  }
}

void setup() {
  Serial.begin(9600);
  
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
<<<<<<< Updated upstream
      if(ultrasonic_10.distanceCm() < 20){
        //Set motor speed to 0 in 0.5 seconds
        Encoder_1.setTarPWM(0);
        Encoder_2.setTarPWM(0);
        _delay(0.5);
        
        //Show blue color on LED-ring
        rgbled_0.setColor(0,ledBlue[0],ledBlue[1],ledBlue[2]);
        rgbled_0.show();

       //Notify Raspberry Pi that Ultrasonic sensor has observed an obsticle 
       Serial.write("hit");
       _delay(2); //Might not needed
       //Await response from Raspberry Pi that picture is captured
        while(1){
          if(Serial.available() >= 4){
            int availableSerial = Serial.available();
            char buff[availableSerial];
            Serial.readString().toCharArray(buff, availableSerial + 1);
            if(strcmp(buff, "done") == 0){
              break;
            }
          }
          delay(50);
        }
        
        //Move backwards in 0.5 seconds, 50% of maximum speed
        move(2, 50 / 100.0 * 255);
        _delay(0.5);
        move(2, 0);

        //Turn right in 1 second, 50% of maximum speed
        move(4, 50 / 100.0 * 255);
        _delay(1);
        move(4, 0);

=======
    
    //serialCheckState();
  //  if(mode == S_AUTO){ 
      autoMode();
   // }
//    }
//    else if(mode == S_MANUAL){
//      manualMode();
//    }


    
    unsigned long timestamp = millis();
    move(1, 50 / 100.0 * 255);
    _delay(1);
    float cmPerSecond = 20.0;
    _delay(1);
    move(2, 0);
    float distance = get_distance_traveled(cmPerSecond, get_time_passed(timestamp) / 1000);
    update_coordinates(distance, degrees_to_radians(gyro.getAngle(3)));
    debugTrackingPrint(timestamp, distance);
    _delay(1);


    timestamp = millis();
    move(4, 50 / 100.0 * 255);
    _delay(1);
    move(1, 0);
    distance = get_distance_traveled(cmPerSecond, get_time_passed(timestamp) / 1000);
    update_coordinates(distance, degrees_to_radians(gyro.getAngle(3)));
    debugTrackingPrint(timestamp, distance);
    _delay(1);



    timestamp = millis();
    move(1, 50/100.0 * 255);
    _delay(1);
    move(1, 0);
    distance = get_distance_traveled(cmPerSecond, get_time_passed(timestamp) / 1000);
    update_coordinates(distance, degrees_to_radians(gyro.getAngle(3)));
    debugTrackingPrint(timestamp, distance);
    _delay(1);
    
    _loop();
  }
}

void autoMode(){
    while(1){
      if(Serial.available() >= 3)
      {
        _delay(0.2);
        //Wait for Raspberry Pi to notify that Lidar has observed an obstacle
        int availableSerial = Serial.available();
        char buff[availableSerial];
        Serial.readString().toCharArray(buff, availableSerial +1);
        //Serial.write(buff);
        if(strcmp(buff, "lidarHit") == 0)
        {
          //Show blue color on LED-ring
          rgbled_0.setColor(0,ledBlue[0],ledBlue[1],ledBlue[2]);
          rgbled_0.show();
          //Set motor speed to 0 in 0.5 seconds
          Encoder_1.setTarPWM(0);
          Encoder_2.setTarPWM(0);
          Serial.write("stopped");
          _delay(0.2);
          while(1){
            int availableSerial2 = Serial.available();
            char buff2[availableSerial2];
            Serial.readString().toCharArray(buff2, availableSerial2 +1);
            if(strcmp(buff2, "done") == 0){
              //Show green color on LED-ring
              rgbled_0.setColor(0,ledGreen[0],ledGreen[1],ledGreen[2]);
              rgbled_0.show();
              break;
            }
          }
          //Move backwards in 0.5 seconds, 50% of maximum speed
          move(2, 50 / 100.0 * 255);
          _delay(0.5);
          move(2, 0);
    
          //Choose left or right randomly and turn in  1 second 50% of speed
          int randomDirection = rand() % 2 + 3;
          move(randomDirection, 50 / 100.0 * 255);
          _delay(1);
          move(randomDirection, 0);
        }
      }
        else if(linefollower_9.readSensors() == 0.000000)
        {
          lineFollowerTriggered();
        }
        else
        {
          autoDriveForward();   
        }
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
>>>>>>> Stashed changes
      }
      else if(linefollower_9.readSensors() == 0.000000){
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

        //Turn right in right in 1 second, 50% of maximum speed
        move(4, 50 / 100.0 * 255);
        _delay(1);
        move(4, 0);

      }else{
        //Show green color on LED-ring
        rgbled_0.setColor(0,ledGreen[0],ledGreen[1],ledGreen[2]);
        rgbled_0.show();

        //Go forward, 50% of maximum speed
        move(1, 50 / 100.0 * 255);

      }

      _loop();
  }

}

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

void loop() {
  _loop();
}
