////////////////////////////////////////////////////////////////////////////////
////////////////////////////////JuFo 2015///////////////////////////////////////
/////////////////////////////////comm.c/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///	Kommunikation mit Master
///
/// Protokoll:
///		<Startbyte 0xAB><Register/Command><Batchlength n>(n*<Data>)<Checksumme1><Checksumme2> -> 5 + n Byte pro Paket
///		Slave antwortet genau so.
///		Batch: Bit 0..6: Batchlänge, Bit 7: Write access
///
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
///
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "comm.h"
#include "uart.h"
#include "bluetooth.h"

#define COMM_BAUD_SELECT(baudRate,xtalCpu)  (((xtalCpu) + 8UL * (baudRate)) / (16UL * (baudRate)) -1UL)

/** Size of the circular transmit buffer, must be power of 2 */
#define UART_TX_BUFFER_SIZE 32

static volatile unsigned char UART1_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;

static volatile uint8_t comm_reg[COMM_REGSIZE];

comm_msg_t receivedMessage;
uint8_t messageBuffer[COMM_BUFSIZE];
comm_msg_t sendMessage;

////////////////////////////////////////////////////////////////////////
/// \brief comm_init
///		inits comm (usart and other stuff, variables etc...)

void comm_init(void)
{
	uart1_init(COMM_BAUD_SELECT(UART_COMM_BAUD_RATE,F_CPU)); //IMU
	receivedMessage.data = messageBuffer;
}

//////////////////////////////////////////////////////////////////
/// \brief ISR(USART1_RX_vect)
///		listens to the uart and puts a message into a message struct
///		if receives one

static volatile uint8_t comm_sm = 0; //statemachine

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

	bt_putCh(data);
	bt_putCh(' ');

	switch (comm_sm) {
	case 0:
		if(data == 0xAB)
			comm_sm = 1;
		break;
	case 1:
		receivedMessage.reg = data;
		comm_sm = 2;
		break;
	case 2:
		receivedMessage.batch = (data & COMM_BATCH);
		receivedMessage.batch_write = (data & COMM_BATCH_WRITE) >> 7;

		comm_batch_i = 0;

		if(receivedMessage.batch_write)
			comm_sm = 3; //Write access! Write into registers.
		else
			comm_sm = 4; //The master wants ro read. Continue with checksum.

		break;
	case 3: //Master wants to write in registers
		if(comm_batch_i < receivedMessage.batch)
		{
			receivedMessage.data[comm_batch_i] = data;
			comm_batch_i ++;
		}
		else
			comm_sm = 4; //checksum
		break;
	case 4: //Checksum LSB
		receivedMessage.checksum = data;
		comm_sm = 5;
		break;
	case 5:
		receivedMessage.checksum |= (data << 8);
		//Received the package. Let the slave process and respond to it!
		comm_sm = 6;
		break;
	case 6: break; //IDLE, wait for processing of query. Only listen for new packages if processed and answered.
	default: comm_sm = 0;
		break;
	}
}

/////////////////////////////////////////////////////////////////////////////////
/// \brief comm_sendPackage
///		sends the message/the package *msg to the master and also calculates checksum etc.
/// \param msg
///		message to send

void comm_sendPackage(comm_msg_t *msg)
{
	msg->checksum = 0xAB + msg->reg + (msg->batch_write | msg->batch);

	uart1_putc(0xAB);
	uart1_putc(msg->reg);
	uart1_putc(msg->batch_write | msg->batch);
	for(uint8_t i = 0; i < msg->batch; i++)
	{
		uart1_putc(msg->data[i]);
		msg->checksum += msg->data[i];
	}
	uart1_putc(msg->checksum & 0xff);
	uart1_putc((msg->checksum & 0xff00) >> 8);
}

//////////////////////////////////////////////////////////////////////
/// \brief comm_handler
///		handles the received messages and answers to them (to call as often
///		as possible)

void comm_handler(void)
{
	if(comm_sm == 6) //new package arrived!
	{
		TOGGLE_MAIN_LED(); //Toggle LED on the RNmega Board

		//Process...
		int16_t checksum_calc = 0xAB;
		checksum_calc += receivedMessage.reg;
		checksum_calc += (receivedMessage.batch_write | receivedMessage.batch);
		if(receivedMessage.batch & COMM_BATCH_WRITE)
		{
			for(uint8_t i = 0; i < receivedMessage.batch; i++)
				checksum_calc += receivedMessage.data[i];
		}

		if(checksum_calc == receivedMessage.checksum) //Checksum matches, if write access write registers. Sens answer.
		{
			sendMessage.reg = 255;
			if(receivedMessage.batch_write) //Master wants to write into slave
			{
				for(uint8_t i = 0; i < receivedMessage.batch; i++)
				{
					comm_reg[receivedMessage.reg + i] = receivedMessage.data[i];
				}
				sendMessage.batch_write = 0;
				sendMessage.batch = 0;
			}
			else //Master wants to read -> Slave writes to master
			{
				sendMessage.batch_write = 1;
				sendMessage.batch = receivedMessage.batch;
				sendMessage.data = (uint8_t *) &comm_reg[receivedMessage.reg];
			}
		}
		else //Send error message.
		{
			sendMessage.reg = 254;
			sendMessage.batch_write = 0;
			sendMessage.batch = 0;
		}

		comm_sendPackage(&sendMessage);

		comm_sm = 0;
	}
}

//////////////////////////////////////////////////////////////////////////
////////////////////USART intern functions////////////////////////////////
//////////////////////////////////////////////////////////////////////////

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
