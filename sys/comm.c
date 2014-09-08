////////////////////////////////////////////////////////////////////////////////
////////////////////////////////JuFo 2015///////////////////////////////////////
/////////////////////////////////comm.c/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///	Kommunikation mit Master
///
/// Protokoll:
///		<Startbyte 0xAB><Register/Command><Batchlength n>(n*<Data>)<Checksumme1><Checksumme2> -> 5 + n Byte pro Paket
///		Slave antwortet genau so. Bei fehlerhaftem Paket antwortet der Slave mit batch = 0 und Register = 255.
///		Batch: Bit 0..6: Batchlänge, Bit 7: Write access
///


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

volatile uint8_t comm_sm = WAITFORPACKAGE; //statemachine

uint8_t chk_lsb = 0;
uint8_t chk_msb = 0;

ISR(USART1_RX_vect)
/*************************************************************************
Function: UART1 Receive Complete interrupt
Purpose:  called when the UART1 has received a character
**************************************************************************/
{
	timer_comm_timeout = TIMER_COMM_TIMEOUT;
	static int8_t comm_batch_i = 0;

	/* read UART status register and UART data register */
	unsigned char data = UDR1;

	switch (comm_sm) {
	case WAITFORPACKAGE:
		if(data == 0xAB)
			comm_sm = GET_REGISTER;
		break;
	case GET_REGISTER:
		receivedMessage.reg = data;
		comm_sm = GET_BATCH;
		break;
	case GET_BATCH:
		receivedMessage.batch = (data & COMM_BATCH);
		receivedMessage.batch_write = (data & COMM_BATCH_WRITE) >> 7;
		comm_batch_i = 0;

		if(receivedMessage.batch_write)
			comm_sm = GET_DATA; //Write access! Write into registers.
		else
			comm_sm = GET_CHK_LSB; //The master wants ro read. Continue with checksum.

		break;
	case GET_DATA: //Master wants to write in registers
		receivedMessage.data[comm_batch_i] = data;
		comm_batch_i ++;

		if(comm_batch_i <= receivedMessage.batch)
			break;

	case GET_CHK_LSB: //Checksum LSB
		receivedMessage.checksum = data << 8;
		chk_lsb = data;
		comm_sm = GET_CHK_MSB;
		break;
	case GET_CHK_MSB:
		receivedMessage.checksum |= data;
		chk_msb = data;
		//Received the package. Let the slave process and respond to it!
		comm_sm = BUSY;
		break;
	case BUSY: break; //BUSY, wait for processing of query. Only listen for new packages if processed and answered.
	default: comm_sm = WAITFORPACKAGE;
		break;
	}
}

//////////////////////////////////////////////////////////////////////////////
/// \brief comm_calcChecksum
///		calculates checksum of the given message and returns it (does not save
///		it into the msg.checksum element!!!)
/// \param msg
/// \return checksum

uint16_t comm_calcChecksum(comm_msg_t *msg)
{
	uint16_t checksum = 0xAB + msg->reg + ((msg->batch_write << 7) | msg->batch);
	if(msg->batch_write)
	{
		for(uint8_t i = 0; i < msg->batch; i++)
		{
			checksum += msg->data[i];
		}
	}
	return checksum;
}

/////////////////////////////////////////////////////////////////////////////////
/// \brief comm_sendPackage
///		sends the message/the package *msg to the master and also calculates checksum etc.
/// \param msg
///		message to send

void comm_sendPackage(comm_msg_t *msg)
{
	uart1_putc(0xAB);
	uart1_putc(msg->reg);
	uart1_putc((msg->batch_write << 7) | msg->batch);
	if(msg->batch_write)
	{
		for(uint8_t i = 0; i < msg->batch; i++)
		{
			uart1_putc(msg->data[i]);
		}
	}
	msg->checksum = comm_calcChecksum(msg);
	uart1_putc(msg->checksum >> 8);
	uart1_putc(msg->checksum & 0xff);
}

//////////////////////////////////////////////////////////////////////
/// \brief comm_handler
///		handles the received messages and answers to them (to call as often
///		as possible)

void comm_handler(void)
{
	if(comm_sm == BUSY) //new package arrived!
	{
		bt_putStr("batchlength: "); bt_putLong(receivedMessage.batch);bt_putStr(", write: ");bt_putLong(receivedMessage.batch);bt_putStr("\n");
		bt_putStr("lsb: "); bt_putLong(chk_lsb);bt_putStr("\n");
		bt_putStr("msb: "); bt_putLong(chk_msb);bt_putStr("\n");


		bt_putStr("Received Package... Process!\n");

		TOGGLE_MAIN_LED(); //Toggle LED on the RNmega Board

		//Process...
		if(comm_calcChecksum(&receivedMessage) == receivedMessage.checksum) //Checksum matches, if write access write registers. Sens answer.
		{
			bt_putStr("Checksum matches!\n");

			sendMessage.reg = receivedMessage.reg;
			if(receivedMessage.batch_write) //Master wants to write into slave
			{
				bt_putStr("Write access! Write ");
				bt_putLong(receivedMessage.batch);
				bt_putStr(" bytes beginning from register ");
				bt_putLong(receivedMessage.reg);
				bt_putStr("!\n ");
				for(uint8_t i = 0; i < receivedMessage.batch; i++)
				{
					comm_reg[receivedMessage.reg + i] = receivedMessage.data[i];
				}
				sendMessage.batch_write = 0;
				sendMessage.batch = 0;
			}
			else //Master wants to read -> Slave writes to master
			{
				bt_putStr("Read access! Read ");
				bt_putLong(receivedMessage.batch);
				bt_putStr(" bytes beginning from register ");
				bt_putLong(receivedMessage.reg);
				bt_putStr("!\n ");

				sendMessage.batch_write = 1;
				sendMessage.batch = receivedMessage.batch;
				sendMessage.data = (uint8_t *) &comm_reg[receivedMessage.reg];
			}
		}
		else //Send error message.
		{
			bt_putStr("Checksum does not match! Checksum calc: ");
			bt_putLong(comm_calcChecksum(&receivedMessage));
			bt_putCh('\n');
			bt_putStr("Received checksum: ");
			bt_putLong(receivedMessage.checksum);
			bt_putCh('\n');
			sendMessage.reg = 255; //Register 0xff: Error
			sendMessage.batch_write = 0;
			sendMessage.batch = 0;
		}

		comm_sendPackage(&sendMessage);

		comm_sm = WAITFORPACKAGE;
	}
	else if(comm_sm != WAITFORPACKAGE)
	{
		if(timer_comm_timeout == 0)
		{
			timer_comm_timeout = -1;
			bt_putStr("Timeout!\n");
			comm_sm = WAITFORPACKAGE;
		}
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
