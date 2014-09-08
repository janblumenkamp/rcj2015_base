/////////////////////////////////////////////////////////////////////////////////
/// Communication with master - definitions
///	Register/Command:
/// 0			Status (succeed/error)
/// 1			DIST_BACK_RIGHT LSB
/// 2			DIST_BACK_RIGHT MSB
/// 3			DIST_RIGHT_BACK LSB
/// 4			DIST_RIGHT_BACK MSB
/// 5			DIST_LEFT_BACK  LSB
/// 6			DIST_LEFT_BACK  MSB
/// 7			DIST_BACK_LEFT  LSB
/// 8			DIST_BACK_LEFT  MSB
/// 9			DIST_FRONT_FRONT LSB
/// 10			DIST_FRONT_FRONT MSB
/// 11			DIST_BACK_BACK   LSB
/// 12			DIST_BACK_BACK   MSB
/// 13			ADC 6 LSB
/// 14			ADC 6 MSB
/// 15			DIST_DOWN LSB
/// 16			DIST_DOWN MSB
/// 17			ADC 8 LSB
/// 18			ADC 8 MSB
/// 19			ADC_BATTERY LSB
/// 20			ADC_BATTERY MSB
/// 21			DIST_FRONT_LEFT LSB
/// 22			DIST_FRONT_LEFT MSB
/// 23			DIST_LEFT_FRONT LSB
/// 24			DIST_LEFT_FRONT MSB
/// 25			SENS_IMPASSE_1 LSB
/// 26			SENS_IMPASSE_1 MSB
/// 27			SENS_IMPASSE_1 LSB
/// 28			SENS_IMPASSE_1 MSB
/// 29			DIST_RIGHT_FRONT LSB
/// 30			DIST_RIGHT_FRONT MSB
/// 31			DIST_FRONT_RIGHT LSB
/// 32			DIST_FRONT_RIGHT MSB
/// 33			Batterie in mV LSB
/// 34			Batterie in mV MSB
/// 35			Batterie in %
/// 36			Motor Encoder L LSB
/// 37			Motor Encoder L
/// 38			Motor Encoder L
/// 39			Motor Encoder L MSB
/// 40			Motor Encoder R LSB
/// 41			Motor Encoder R
/// 42			Motor Encoder R
/// 43			Motor Encoder R MSB
/// 44			Motor Geschwindigkeit L ist
/// 45			Motor Geschwindigkeit R ist
/// 46			Motor Geschwindigkeit L soll
/// 47			Motor Geschwindigkeit R soll
/// 48			Motortreiber aktiv
/// 49			LED Modus
/// 50			LED Hue
/// 51			LED Saturation
/// 52			LED Value

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

#define UART_COMM_BAUD_RATE    1000000

#define COMM_BUFSIZE 128 //In Bytes. Size of the receive buffer.

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

typedef enum {
	COMM_SYSTEMSTATUS, //Can be: 0 (ok), 1 (error)
	COMM_DIST_BACK_RIGHT_LSB, COMM_DIST_BACK_RIGHT_MSB,
	COMM_DIST_RIGHT_BACK_LSB, COMM_DIST_RIGHT_BACK_MSB,
	COMM_DIST_LEFT_BACK_LSB, COMM_DIST_LEFT_BACK_MSB,
	COMM_DIST_BACK_LEFT_LSB, COMM_DIST_BACK_LEFT_MSB,
	COMM_DIST_FRONT_FRONT_LSB, COMM_DIST_FRONT_FRONT_MSB,
	COMM_DIST_BACK_BACK_LSB, COMM_DIST_BACK_BACK_MSB,
	COMM_ADC6_LSB, COMM_ADC6_MSB,
	COMM_DIST_DOWN_LSB, COMM_DIST_DOWN_MSB,
	COMM_ADC8_LSB, COMM_ADC8_MSB,
	COMM_ADC_BATTERY_RAW_LSB, COMM_ADC_BATTERY_RAW_MSB,
	COMM_DIST_FRONT_LEFT_LSB, COMM_DIST_FRONT_LEFT_MSB,
	COMM_DIST_LEFT_FRONT_LSB, COMM_DIST_LEFT_FRONT_MSB,
	COMM_SENS_IMPASSE_1_LSB, COMM_SENS_IMPASSE_1_MSB,
	COMM_SENS_IMPASSE_2_LSB, COMM_SENS_IMPASSE_2_MSB,
	COMM_DIST_RIGHT_FRONT_LSB, COMM_DIST_RIGHT_FRONT_MSB,
	COMM_DIST_FRONT_RIGHT_LSB, COMM_DIST_FRONT_RIGHT_MSB,
	COMM_BATTERY_MV_LSB, COMM_BATTERY_MV_MSB,
	COMM_BATTERY_PERCENT,
	COMM_MOT_ENC_L_LSB_0, COMM_MOT_ENC_L_1, COMM_MOT_ENC_L_2, COMM_MOT_ENC_L_MSB_3,
	COMM_MOT_ENC_R_LSB_0, COMM_MOT_ENC_R_1, COMM_MOT_ENC_R_2, COMM_MOT_ENC_R_MSB_3,
	COMM_MOT_SPEED_L_IS,
	COMM_MOT_SPEED_R_IS,
	COMM_MOT_SPEED_L_TO,
	COMM_MOT_SPEED_R_TO,
	COMM_MOT_DRIVER_STANDBY,
	COMM_LED_MODE, //Can be: 0 (control by sub), 1 (control by main -> Hue, saturation and value)
	COMM_LED_HUE,
	COMM_LED_SAT,
	COMM_LED_VAL,
	/////// DO NOT ADD ANYTHING BELOW THIS LINE!!!//////////////
	COMM_REGISTERS_CNT
} REGISTERS;

//Initialisation of the comm interface
extern void comm_init(void);

//Handles all queries. To call as often as possible!
extern void comm_handler(void);

//gateway between register and variables of the system
extern void comm_reg_gateway(void);

//Sends given package
extern void comm_sendPackage(comm_msg_t *msg);

/** @brief  Initialize USART1 (only available on selected ATmegas) @see uart_init */
extern void uart1_init(unsigned int baudrate);
/** @brief  Get received byte of USART1 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern void uart1_putc(unsigned char data);

#endif // COMM_H
