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

/*
*Used to initiate positioning module
*Takes a MeEncoderOnBoard reference
*Returns nothing
*/
void positioning_init(MeGyro* gyro0);

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

/*
*Immediatly calculates the new coordinates
*/
void calculate_new_coordinates();

/*
*Calculates the new coordinates is enough time has passed
*/
void calculate_new_coordinates_interval();

/*
*Sends last calculated coordinates if enough time has passed
*/
void send_latest_coordinates_interval();

#endif
