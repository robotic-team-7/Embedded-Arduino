#ifndef POSITIONING
#define POSITIONING

#include <MeEncoderOnBoard.h>
#include <MeGyro.h>
#include <math.h>



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

float getCoordinateX();

float getCoordinateY();

void setTimestamp();

void registerPositionChange(float speed_cm_per_sec);

void printCoordinates();


#endif
