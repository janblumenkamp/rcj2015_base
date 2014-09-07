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

#define COMM_REGSIZE 53 //In bytes
#define COMM_BUFSIZE 128 //In Bytes

#define COMM_BATCH_WRITE	0x80
#define COMM_BATCH			0x7F

typedef struct {
	uint8_t reg;
	uint8_t batch:7;
	uint8_t batch_write:1;
	uint8_t *data; //Pointer to buffer (in case of write access)
	uint16_t checksum;
} comm_msg_t;

extern void comm_init(void);

extern void comm_handler(void);

extern void comm_sendPackage(comm_msg_t *msg);

/** @brief  Initialize USART1 (only available on selected ATmegas) @see uart_init */
extern void uart1_init(unsigned int baudrate);
/** @brief  Get received byte of USART1 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern void uart1_putc(unsigned char data);

#endif // COMM_H
