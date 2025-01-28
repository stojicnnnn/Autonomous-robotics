/*
 * uart.h
 *
 *  Created on: 17. mar 2024.
 *      Author: HP
 */

#ifndef INC_BIBLIOTEKA_UART_H_
#define INC_BIBLIOTEKA_UART_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

void
uart_init ();
void uart_send_byte (uint8_t podatak);

#endif /* INC_BIBLIOTEKA_UART_H_ */
