#ifndef INS_BOARD_H
#define INS_BOARD_H

#include <libopencm3/stm32/gpio.h>

#define STATUS_LED_ON() {gpio_set(GPIOB, GPIO14);}
#define STATUS_LED_OFF() {gpio_clear(GPIOB, GPIO14);}
#define STATUS_LED_TOGGLE() {gpio_toggle(GPIOB, GPIO14);}

#define ERROR_LED_ON() {gpio_set(GPIOA, GPIO10);}
#define ERROR_LED_OFF() {gpio_clear(GPIOA, GPIO10);}
#define ERROR_LED_TOGGLE() {gpio_toggle(GPIOA, GPIO10);}

#define HEARTBEAT_LED_ON() {gpio_set(GPIOA, GPIO8);}
#define HEARTBEAT_LED_OFF() {gpio_clear(GPIOA, GPIO8);}
#define HEARTBEAT_LED_TOGGLE() {gpio_toggle(GPIOA, GPIO8);}

#define SD_LED_ON() {gpio_set(GPIOB, GPIO15);}
#define SD_LED_OFF() {gpio_clear(GPIOB, GPIO15);}
#define SD_LED_TOGGLE() {gpio_toggle(GPIOB, GPIO15);}


#endif // INS_BOARD_H
