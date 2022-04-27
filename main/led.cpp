#include "led.h"

MeRGBLed rgbled_0(0, 12);

//Adjust brightness of LED-ring. 0.0 = minimum brightness, 1.0 = maximum brightness
float led_brightness = 0.1;

//Define standard colors and adjust brightness
int ledRed[3] = {(int)(255 * led_brightness), 0, 0};
int ledGreen[3] = {0, (int)(255 * led_brightness), 0};
int ledBlue[3] = {0, 0, (int)(255 * led_brightness)};
int ledYellow[3] = {(int)(255 * led_brightness), (int)(255 * led_brightness), 0};

void led_init(){
    rgbled_0.setpin(44);
    rgbled_0.fillPixelsBak(0, 2, 1);
}

void set_leds_yellow(){
    rgbled_0.setColor(0, ledYellow[0], ledYellow[1], ledYellow[2]);
    rgbled_0.show();
}

void set_leds_red(){
    rgbled_0.setColor(0, ledRed[0], ledRed[1], ledRed[2]);
    rgbled_0.show();
}

void set_leds_blue(){
    rgbled_0.setColor(0, ledBlue[0], ledBlue[1], ledBlue[2]);
    rgbled_0.show();
}

void set_leds_green(){
    rgbled_0.setColor(0, ledGreen[0], ledGreen[1], ledGreen[2]);
    rgbled_0.show();
}