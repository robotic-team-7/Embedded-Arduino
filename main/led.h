#ifndef LED_H
#define LED_H

#include <MeRGBLed.h>

/*
* Should be called once in the set-up code. 
* Sets up the led.
* Returns noithing
*/
void led_init();

/*
*Called to set led to yellow color. 
*Returns nothing.
*/
void set_leds_yellow();

/*
*Called to set led to red color.
*Returns nothing.
*/
void set_leds_red();

/*
*Called to set led to blue color.
*Returns nothing.
*/
void set_leds_blue();

/*
*Called to set led to green color. 
*Returns nothing.
*/
void set_leds_green();

#endif
