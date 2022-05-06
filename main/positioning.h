#ifndef POSITIONING
#define POSITIONING

#include <MeEncoderOnBoard.h>
#include <MeGyro.h>
#include <math.h>
#include "drive_control.h"



typedef struct coordinate{
    float x;
    float y;
}Coordinate;

float get_current_speed(MeEncoderOnBoard* encoder);

/*
*Used to initiate positioning module
*Takes a MeEncoderOnBoard reference
*Returns nothing
*/
void positioning_init(MeGyro* gyro0);

/*
*Used to get time passed since last timestamp
*Suggestuin is to call this function when:
*   Making a call to get_distance_traveled()
*Returns number of milliseconds passed since given timestamp
*/
float get_time_passed(unsigned long timestamp_in_ms);

/*
* Updates current coordinate
* Suggestion is to call this function when:
*   Asked for coordinate
*   Mower stop/change direction
*   Mower have reversed after obstacle
*Returns nothing
*/
void update_coordinates(float distance_traveled_cm, double angle_in_radians);

/*
*Used to convert degrees to radians
*Suggestion is to call this function when:
*   Making a call to update_coordinates which takes an angle in radians
*Returns equivalent angle in radians
*/
double degrees_to_radians(float angle_in_degrees);

/*
*Used to get a distance moved during some given duration
*Suggestion is to call this function when:
*   Making a call to update_coordinates() which takes a distance in cm
*Returns distance traveled during given time duration in unit cm/sec
*/
float get_distance_traveled(float speed_cm_per_sec, float time_in_seconds);

/*
*Used to get the last calculated x-coordinate.
*Returns a float value representing traveled distance on the x-axis in cm.
*/
float get_coordinate_x();

/*
*Used to get the last calculated y-coordinate.
*Returns a float value representing traveled distance on the y-axis in cm.
*/
float get_coordinate_y();

/*
*Used to set timestamp which is used to calculate distance
*Suggestion is to call this function every time robot starts
*travel a new path or during a travel after registerPositionChange has been called.
*Returns nothing.
*/
void set_timestamp();

/*
*Used to update coordinates. 
*Suggestion is to call this function periodically or at a path change.
*SetTimestamp should be called afterwards, when a new travel starts.
*Returns nothing.
*/
void register_position_change(float encoder_data);

/*
*Used to calculate distance traveled through encoder pulses. 
*Returns a float representing distance traveled in cm.
*/
float driven_distance(long pulses_left_motor, int pulses_right_motor);

/*
*Sends the latest sampled coordinates.
*Returns nothing.
*/
void send_coordinates();

void calculate_new_coordinates();

void calculate_new_coordinates_interval();

void send_latest_coordinates_interval();

#endif
