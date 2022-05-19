#include "positioning.h"

Coordinate current_position;
MeGyro* gyroscope;

unsigned long timestamp = 0;

void positioning_init(MeGyro* gyro0){
    gyroscope = gyro0;
    current_position = {0, 0};
}

double degrees_to_radians(float angle_in_degrees){
  return (angle_in_degrees * M_PI) / 180;
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

void register_position_change(float distance){
  gyroscope->update();
  update_coordinates(distance, degrees_to_radians(-gyroscope->getAngle(3)) + M_PI/2);
  
  reset_encoders();
}

float driven_distance(long pulses_left_motor, int pulses_right_motor){
  float pulses_per_cm = 27.71;

  float distance_left_motor = (pulses_left_motor * -1) / pulses_per_cm;
  float distance_right_motor = pulses_right_motor / pulses_per_cm;

  return (distance_left_motor + distance_right_motor) /2;
}

void calculate_new_coordinates(){
  Encoder_data encoder_data = get_encoder_pos_of_motor();
  float distance = driven_distance(encoder_data.left_motor, encoder_data.right_motor);
  register_position_change(distance);
}

void calculate_new_coordinates_interval(){
  static long calculate_interval_timer = 0;
  if(millis() > calculate_interval_timer + 100){
    Encoder_data encoder_data = get_encoder_pos_of_motor();
    float distance = driven_distance(encoder_data.left_motor, encoder_data.right_motor);
    register_position_change(distance);
    
    calculate_interval_timer = millis();
  }
}

void send_latest_coordinates_interval(){
  static long send_interval_timer = 0;
  if(millis() > send_interval_timer + 625){
    send_coordinates();
    send_interval_timer = millis();
  }
}

void send_coordinates(){
  //Serial.print("(");
  Serial.print(current_position.x);
  Serial.print(",");
  Serial.print(current_position.y);
  //Serial.print(")");
  Serial.print("\n");
}
