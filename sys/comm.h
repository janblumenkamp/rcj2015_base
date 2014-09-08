/////////////////////////////////////////////////////////////////////////////////
/// Communication with master - definitions
/////////////////////////////////////////////////////////////////////////////////

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

#define COMM_REGSIZE 53 //In bytes. Size of the register.
#define COMM_BUFSIZE 128 //In Bytes

#define COMM_BATCH_WRITE	0x80 //Batch write bit
#define COMM_BATCH			0x7F //Batch length bits

//Message/Package struct
typedef struct {
	uint8_t reg; //Desired register
	uint8_t batch:7; //Batchlength
	uint8_t batch_write:1; //Write access?
	uint8_t *data; //Pointer to buffer
	uint16_t checksum;
} comm_msg_t;

typedef enum {
	WAITFORPACKAGE,
	GET_REGISTER, GET_BATCH, GET_DATA, GET_CHK_LSB, GET_CHK_MSB,
	BUSY
} COMM_SM;

//Initialisation of the comm interface
extern void comm_init(void);

//Handles all queries. To call as often as possible!
extern void comm_handler(void);

//Sends given package
extern void comm_sendPackage(comm_msg_t *msg);

/** @brief  Initialize USART1 (only available on selected ATmegas) @see uart_init */
extern void uart1_init(unsigned int baudrate);
/** @brief  Get received byte of USART1 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern void uart1_putc(unsigned char data);

#endif // COMM_H
