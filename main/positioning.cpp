#include "positioning.h"

#define WHEEL_CIRCUMFERENCE_IN_CM 16
#define SECONDS_IN_MINUTE 60

Coordinate current_position;
MeEncoderOnBoard* encoder_motor; //Might delete, not used right now
MeGyro* gyroscope;

int amount_of_samples = 0;

Coordinate coordinates[32];

unsigned long timestamp = 0;

void init(MeEncoderOnBoard* encoder, MeGyro* gyro0){
    encoder_motor = encoder;
    gyroscope = gyro0;
    current_position = {0, 0};

    for(int i = 0; i < 32; i++){
      coordinates[i].x = 0;
      coordinates[i].y = 0;
    }
}

//Not used right now, might delete
float get_current_speed_cm_per_second(){
  return WHEEL_CIRCUMFERENCE_IN_CM * encoder_motor->getCurrentSpeed() / SECONDS_IN_MINUTE;
}

double degrees_to_radians(float angle_in_degrees){
  return (angle_in_degrees * M_PI) / 180;
}

float get_distance_traveled(float speed_cm_per_sec, float time_in_seconds){
  return speed_cm_per_sec * time_in_seconds;
}

float get_time_passed(unsigned long timestamp_in_ms){
  return millis() - timestamp_in_ms;
}

void update_coordinates(float distance_traveled_cm, double angle_in_radians){
    current_position.x = current_position.x + distance_traveled_cm * cos(angle_in_radians);
    current_position.y = current_position.y + distance_traveled_cm * sin(angle_in_radians);
}

float getCoordinateX(){
  return current_position.x;
}

float getCoordinateY(){
  return current_position.y;
}

void setTimestamp(){
  timestamp = millis();
}

void registerPositionChange(float speed_cm_per_sec){
  gyroscope->update();
  float time_passed = get_time_passed(timestamp) / 1000;
  float distance = get_distance_traveled(speed_cm_per_sec, time_passed);
  update_coordinates(distance, degrees_to_radians(gyroscope->getAngle(3)));

  coordinates[amount_of_samples].x = current_position.x;
  coordinates[amount_of_samples].y = current_position.y;
  amount_of_samples++;
  printCoordinates();
}

void printCoordinates(){
  Serial.print("(");
  Serial.print(current_position.x);
  Serial.print(",");
  Serial.print(current_position.y);
  Serial.print(")");
  Serial.print("\n");

  /*
  for(int i = 0; i < amount_of_samples; i++){
    Serial.print("X: ");
    Serial.print((double)coordinates[i].x);
    Serial.print("Y: ");
    Serial.print((double)coordinates[i].y);
    Serial.print("\n");
  }
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  */
}
