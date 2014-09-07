////////////////////////////////////////////////////////////////////////////////
////////////////////////////////JuFo 2015///////////////////////////////////////
/////////////////////////////////comm.c/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///	Kommunikation mit Master
///
/// Protokoll:
///		<Startbyte 0xAB><Register><Batchlength n>(n*<Data>)<Checksumme1><Checksumme2> -> 5 + n Byte pro Paket
///		Slave antwortet genau so.
///		Batch: Bit 0..6: Batchlänge, Bit 7: Write access
///
///	Register:
/// 0			Systemstatus
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
///
////////////////////////////////////////////////////////////////////////////////

#include "comm.h"
#include "uart.h"

#define COMM_BAUD_SELECT(baudRate,xtalCpu)  (((xtalCpu) + 8UL * (baudRate)) / (16UL * (baudRate)) -1UL)

/** Size of the circular transmit buffer, must be power of 2 */
#define UART_TX_BUFFER_SIZE 32

#define COMM_REGSIZE 53 //In bytes
#define COMM_BUFSIZE 128 //In Bytes

#define COMM_BATCH_WRITE	0x80
#define COMM_BATCH			0x7F

static volatile unsigned char UART1_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;

static volatile uint8_t comm_reg[COMM_REGSIZE];
static volatile uint8_t comm_buf[COMM_BUFSIZE];
static volatile uint8_t comm_lastReg = 0;
static volatile uint8_t comm_batch = 0;


void comm_init(void)
{
	uart1_init(COMM_BAUD_SELECT(UART_COMM_BAUD_RATE,F_CPU)); //IMU
}

uint8_t comm_listen(void)
{

	return 0;
}

static volatile uint8_t comm_sm;

ISR(USART1_RX_vect)
/*************************************************************************
Function: UART1 Receive Complete interrupt
Purpose:  called when the UART1 has received a character
**************************************************************************/
{
	unsigned char data;
	static int8_t comm_batch_i = 0;

	/* read UART status register and UART data register */
	data = UDR1;


	switch (comm_sm) {
	case 0:
		if(data == 0xAB)
			comm_sm = 1;
		break;
	case 1:
		comm_lastReg = data;
		comm_sm = 2;
		break;
	case 2:
		comm_batch = data;

		if(comm_batch & COMM_BATCH_WRITE)
			comm_sm = 3; //Write access
		else
			comm_sm = 4; //Checksum
		break;
	case 3: //Master wants to write in registers
		if(comm_batch_i < (comm_batch & COMM_BATCH))
			comm_buf[comm_batch_i] = data;
		else
			comm_sm = 4; //checksum
		break;
	case 4: //Checksum LSB

	default:
		break;
	}
}


ISR(USART1_UDRE_vect)
/*************************************************************************
Function: UART1 Data Register Empty interrupt
Purpose:  called when the UART1 is ready to transmit the next byte
**************************************************************************/
{
	unsigned char tmptail;


	if ( UART1_TxHead != UART1_TxTail) {
		/* calculate and store new buffer index */
		tmptail = (UART1_TxTail + 1) & (UART_TX_BUFFER_SIZE - 1);
		UART1_TxTail = tmptail;
		/* get one byte from buffer and write it to UART */
		UDR1 = UART1_TxBuf[tmptail];  /* start transmission */
	}else{
		/* tx buffer empty, disable UDRE interrupt */
		UCSR1B &= ~_BV(UDRIE1);
	}
}


/*************************************************************************
Function: uart1_init()
Purpose:  initialize UART1 and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart1_init(unsigned int baudrate)
{
	UART1_TxHead = 0;
	UART1_TxTail = 0;


	/* Set baud rate */
	if ( baudrate & 0x8000 )
	{
		UCSR1A = (1<<U2X1);  //Enable 2x speed
	  baudrate &= ~0x8000;
	}
	UBRR1H = (unsigned char)(baudrate>>8);
	UBRR1L = (unsigned char) baudrate;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	UCSR1B = _BV(RXCIE1)|(1<<RXEN1)|(1<<TXEN1);

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
	#ifdef URSEL1
	UCSR1C = (1<<URSEL1)|(3<<UCSZ10);
	#else
	UCSR1C = (3<<UCSZ10);
	#endif
}/* uart_init */


/*************************************************************************
Function: uart1_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart1_putc(unsigned char data)
{
	unsigned char tmphead;


	tmphead  = (UART1_TxHead + 1) & (UART_TX_BUFFER_SIZE - 1);

	while ( tmphead == UART1_TxTail ){
		;/* wait for free space in buffer */
	}

	UART1_TxBuf[tmphead] = data;
	UART1_TxHead = tmphead;

	/* enable UDRE interrupt */
	UCSR1B    |= _BV(UDRIE1);

}/* uart1_putc */
