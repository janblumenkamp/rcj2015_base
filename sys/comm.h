#ifndef COMM_H
#define COMM_H

#include <avr/io.h>
#include <avr/pgmspace.h> 	// Program memory (=Flash ROM) access routines.
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/wdt.h> //watchdog

#define UART_COMM_BAUD_RATE    115200

extern void comm_init(void);

extern uint8_t comm_listen(void);

/** @brief  Initialize USART1 (only available on selected ATmegas) @see uart_init */
extern void uart1_init(unsigned int baudrate);
/** @brief  Get received byte of USART1 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern void uart1_putc(unsigned char data);

#endif // COMM_H
