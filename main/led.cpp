#include "led.h"

MeRGBLed rgb_led_0(0, 12);

//Adjust brightness of LED-ring. 0.0 = minimum brightness, 1.0 = maximum brightness
float led_brightness = 0.1;

//Define standard colors and adjust brightness
int led_red[3] = {(int)(255 * led_brightness), 0, 0};
int led_green[3] = {0, (int)(255 * led_brightness), 0};
int led_blue[3] = {0, 0, (int)(255 * led_brightness)};
int led_yellow[3] = {(int)(255 * led_brightness), (int)(255 * led_brightness), 0};

void led_init(){
    rgb_led_0.setpin(44);
    rgb_led_0.fillPixelsBak(0, 2, 1);
}

void set_leds_yellow(){
    rgb_led_0.setColor(0, led_yellow[0], led_yellow[1], led_yellow[2]);
    rgb_led_0.show();
}

void set_leds_red(){
    rgb_led_0.setColor(0, led_red[0], led_red[1], led_red[2]);
    rgb_led_0.show();
}

void set_leds_blue(){
    rgb_led_0.setColor(0, led_blue[0], led_blue[1], led_blue[2]);
    rgb_led_0.show();
}

void set_leds_green(){
    rgb_led_0.setColor(0, led_green[0], led_green[1], led_green[2]);
    rgb_led_0.show();
}
