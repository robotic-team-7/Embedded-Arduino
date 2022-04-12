#include "positioning.h"

#define WHEEL_CIRCUMFERENCE_IN_CM 16
#define SECONDS_IN_MINUTE 60

Coordinate current_position;
MeEncoderOnBoard* encoder_motor;

void init(MeEncoderOnBoard* encoder){
    encoder_motor = encoder;
    current_position = {0, 0};
}

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
