#ifndef __PINS_H__
#define __PINS_H__

#include <stdio.h>
#include <driver/gpio.h>

typedef gpio_num_t PIN;

#define PIN_BUTTON_VOLUME       GPIO_NUM_13
#define PIN_BUTTON_INPUT        GPIO_NUM_48
#define PIN_BUTTON_POWER        GPIO_NUM_17

#define PIN_LED_FRONT_POWER     GPIO_NUM_16
#define PIN_LED_FRONT_STANDBY   GPIO_NUM_28

#define PIN_RELAY               GPIO_NUM_16
#define PIN_BACKLIGHT           GPIO_NUM_14

#define PIN_DECODER_RESET       GPIO_NUM_3
#define PIN_DECODER_IRQ         GPIO_NUM_3
#define PIN_LED_ACTIVE          GPIO_NUM_3
#define PIN_CS8416_RESET        GPIO_NUM_3

#endif