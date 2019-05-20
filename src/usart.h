#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "diag/Trace.h"
#include "queue.h"

#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <stddef.h>

#define CMD_BACKLIGHT_ON 0x20
#define CMD_BACKLIGHT_OFF 0x10


uint8_t USART_Temp_Data;

struct Queue* queue;

void init_usart1();

void send_byte(uint8_t b);

uint8_t read_char();

int data_available();

uint8_t get_data();

void broadcast_reading(uint16_t adc);
#endif
