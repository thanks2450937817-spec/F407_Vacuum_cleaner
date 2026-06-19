

#ifndef F407_VACUUM_CLEANER_ESP8266_H
#define F407_VACUUM_CLEANER_ESP8266_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "gpio.h"

void ESP8266_SendString(char *str);
uint8_t ESP8266_SendCommand(char *cmd, char *expected, uint32_t timeout_ms);
void ESP_8266_Init(void);

extern uint8_t rx_buf[];
extern volatile uint8_t rx_ready;
extern uint8_t rx_len;
void Signal_decoding();





#endif //F407_VACUUM_CLEANER_ESP8266_H