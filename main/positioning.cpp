#include "positioning.h"

#define RADIUS 16.0
#define SECONDS_IN_MINUTE 60.0

Coordinate current_position;
MeGyro* gyroscope;

unsigned long timestamp = 0;

void positioning_init(MeGyro* gyro0){
    gyroscope = gyro0;
    current_position = {0, 0};
}

float get_current_speed(MeEncoderOnBoard* encoder){
  float rotations_per_second = encoder->getCurrentSpeed() / SECONDS_IN_MINUTE;
  float cm_per_second = 2 * M_PI * RADIUS * rotations_per_second;
  return cm_per_second; 
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

float get_coordinate_x(){
  return current_position.x;
}

float get_coordinate_y(){
  return current_position.y;
}

void setTimestamp(){
  timestamp = millis();
}

void register_position_change(float distance){
  gyroscope->update();
  update_coordinates(distance, degrees_to_radians(gyroscope->getAngle(3)));
  reset_encoders();
}

float driven_distance(long pulses_left_motor, int pulses_right_motor){
  float pulses_per_cm = 27.71;

  float distance_left_motor = (pulses_left_motor * -1) / pulses_per_cm;
  float distance_right_motor = pulses_right_motor / pulses_per_cm;

  return (distance_left_motor + distance_right_motor) /2;
}

void send_coordinates(){
  Serial.print("(");
  Serial.print(current_position.x);
  Serial.print(",");
  Serial.print(current_position.y);
  Serial.print(")");
  Serial.print("\n");
}
