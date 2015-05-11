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
#include "funktionen.h"
#include "system.h"

#define COMM_BAUD_SELECT(baudRate,xtalCpu)  (((xtalCpu) + 8UL * (baudRate)) / (16UL * (baudRate)) -1UL)

/** Size of the circular transmit buffer, must be power of 2 */
#define UART_TX_BUFFER_SIZE 32

static volatile unsigned char UART1_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;

volatile uint8_t comm_reg[COMM_REGISTERS_CNT];

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
		{
			MAIN_LED_ON(); //Toggle LED on the RNmega Board (After sending answer powered off again)
			comm_sm = GET_REGISTER;
		}
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
		comm_sm = GET_CHK_MSB;
		break;
	case GET_CHK_MSB:
		receivedMessage.checksum |= data;
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
		timer_nocomm = TIMER_NOCOMM;

		if(debug)	bt_putStr_P(PSTR("Received Package... Process!\n"));

		//Process...
		if(comm_calcChecksum(&receivedMessage) == receivedMessage.checksum) //Checksum matches, if write access write registers. Sens answer.
		{
			if(debug) bt_putStr_P(PSTR("Checksum matches!\n"));

			sendMessage.reg = receivedMessage.reg;
			if(receivedMessage.batch_write) //Master wants to write into slave
			{
				if(debug)
				{
					bt_putStr_P(PSTR("Write access! Write "));
					bt_putLong(receivedMessage.batch);
					bt_putStr_P(PSTR(" bytes beginning from register "));
					bt_putLong(receivedMessage.reg);
					bt_putStr_P(PSTR("!\n "));
				}

				for(uint8_t i = 0; i < receivedMessage.batch; i++)
				{
					comm_reg[receivedMessage.reg + i] = receivedMessage.data[i];
				}
				sendMessage.batch_write = 0;
				sendMessage.batch = 0;

				if((receivedMessage.reg >= COMM_MOT_SPEED_L_TO) && (receivedMessage.reg <= COMM_MOT_DRIVER_STANDBY))
				{
					timer_comm_mot_to = TIMER_COMM_MOT_TO; //Set timeout
				}
			}
			else //Master wants to read -> Slave writes to master
			{
				if(debug)
				{
					bt_putStr_P(PSTR("Read access! Read "));
					bt_putLong(receivedMessage.batch);
					bt_putStr_P(PSTR(" bytes beginning from register "));
					bt_putLong(receivedMessage.reg);
					bt_putStr_P(PSTR("!\n "));
				}

				sendMessage.batch_write = 1;
				sendMessage.batch = receivedMessage.batch;
				sendMessage.data = (uint8_t *) &comm_reg[receivedMessage.reg];
			}
		}
		else //Send error message.
		{
			if(debug)
			{
				bt_putStr_P(PSTR("Checksum does not match! Checksum calc: "));
				bt_putLong(comm_calcChecksum(&receivedMessage));
				bt_putStr_P(PSTR("\n"));
				bt_putStr_P(PSTR("Received checksum: "));
				bt_putLong(receivedMessage.checksum);
				bt_putStr_P(PSTR("\n"));
			}
			sendMessage.reg = 255; //Register 0xff: Error
			sendMessage.batch_write = 0;
			sendMessage.batch = 0;
		}

		comm_sendPackage(&sendMessage);

		comm_sm = WAITFORPACKAGE;

		MAIN_LED_OFF();
	}
	else if(comm_sm != WAITFORPACKAGE)
	{
		if(timer_comm_timeout == 0)
		{
			timer_comm_timeout = -1;
			if(debug) bt_putStr_P(PSTR("Timeout!\n"));
			comm_sm = WAITFORPACKAGE;
		}
	}

	if(timer_comm_mot_to == 0) //No new speed/motor activity commands for TIMER_COMM_MOT_TO ms. Stop motors.
	{
		timer_comm_mot_to = -1;
		comm_reg[COMM_MOT_SPEED_L_TO] = 0;
		comm_reg[COMM_MOT_SPEED_R_TO] = 0;
		mot.d[LEFT].speed.to = 0;
		mot.d[RIGHT].speed.to = 0;
	}
}

//////////////////////////////////////////////////////////////////////////////////////
/// \brief comm_reg_gateway
///		interface between the comm_reg registers and the content of the register.
///		Manages to match >8bit variables into the register, reads out variables etc...
///		Has to be called as often as possible!

void comm_reg_gateway(void)
{
	comm_reg[COMM_SYSTEMSTATUS] = system_status;
	comm_reg[COMM_DIST_BACK_RIGHT_LSB] = dist[LIN][BACK][RIGHT] & 0xff;		comm_reg[COMM_DIST_BACK_RIGHT_MSB] = (dist[LIN][BACK][RIGHT] & 0xff00) >> 8;
	comm_reg[COMM_DIST_RIGHT_BACK_LSB] = dist[LIN][RIGHT][BACK] & 0xff;		comm_reg[COMM_DIST_RIGHT_BACK_MSB] = (dist[LIN][RIGHT][BACK] & 0xff00) >> 8;
	comm_reg[COMM_DIST_LEFT_BACK_LSB] = dist[LIN][LEFT][BACK] & 0xff;		comm_reg[COMM_DIST_LEFT_BACK_MSB] = (dist[LIN][LEFT][BACK] & 0xff00) >> 8;
	comm_reg[COMM_DIST_BACK_LEFT_LSB] = dist[LIN][BACK][LEFT] & 0xff;		comm_reg[COMM_DIST_BACK_LEFT_MSB] = (dist[LIN][BACK][LEFT] & 0xff00) >> 8;
	comm_reg[COMM_DIST_FRONT_FRONT_LSB] = dist[LIN][FRONT][FRONT] & 0xff;	comm_reg[COMM_DIST_FRONT_FRONT_MSB] = (dist[LIN][FRONT][FRONT] & 0xff00) >> 8;
	comm_reg[COMM_DIST_BACK_BACK_LSB] = dist[LIN][BACK][BACK] & 0xff;		comm_reg[COMM_DIST_BACK_BACK_MSB] = (dist[LIN][BACK][BACK] & 0xff00) >> 8;
	//comm_reg[COMM_ADC6_LSB] //Not used
	comm_reg[COMM_DIST_DOWN_LSB] = dist_down & 0xff;						comm_reg[COMM_DIST_DOWN_MSB] = (dist_down & 0xff00) >> 8;
	//comm_reg[COMM_ADC8_LSB] //Not used
	comm_reg[COMM_ADC_BATTERY_RAW_LSB] = batt_raw & 0xff;					comm_reg[COMM_ADC_BATTERY_RAW_MSB] = (batt_raw & 0xff00) >> 8;
	comm_reg[COMM_DIST_FRONT_LEFT_LSB] = dist[LIN][FRONT][LEFT] & 0xff;		comm_reg[COMM_DIST_FRONT_LEFT_MSB] = (dist[LIN][FRONT][LEFT] & 0xff00) >> 8;
	comm_reg[COMM_DIST_LEFT_FRONT_LSB] = dist[LIN][LEFT][FRONT] & 0xff;		comm_reg[COMM_DIST_LEFT_FRONT_MSB] = (dist[LIN][LEFT][FRONT] & 0xff00) >> 8;
	comm_reg[COMM_SENS_IMPASSE_1_LSB] = groundsens_l & 0xff;				comm_reg[COMM_SENS_IMPASSE_1_MSB] = (groundsens_l & 0xff00) >> 8;
	comm_reg[COMM_SENS_IMPASSE_2_LSB] = groundsens_r & 0xff;				comm_reg[COMM_SENS_IMPASSE_2_MSB] = (groundsens_r & 0xff00) >> 8;
	comm_reg[COMM_DIST_RIGHT_FRONT_LSB] = dist[LIN][RIGHT][FRONT] & 0xff;	comm_reg[COMM_DIST_RIGHT_FRONT_MSB] = (dist[LIN][RIGHT][FRONT] & 0xff00) >> 8;
	comm_reg[COMM_DIST_FRONT_RIGHT_LSB] = dist[LIN][FRONT][RIGHT] & 0xff;	comm_reg[COMM_DIST_FRONT_RIGHT_MSB] = (dist[LIN][FRONT][RIGHT] & 0xff00) >> 8;
	comm_reg[COMM_BATTERY_MV_LSB] = batt_mV & 0xff;							comm_reg[COMM_BATTERY_MV_LSB] = (batt_mV & 0xff00) >> 8;
	comm_reg[COMM_BATTERY_PERCENT] = batt_percent;
	comm_reg[COMM_MOT_ENC_L_LSB_0] = (uint8_t)(mot.d[LEFT].enc & 0x000000ff);
		comm_reg[COMM_MOT_ENC_L_1] = (uint8_t)((mot.d[LEFT].enc & 0x0000ff00) >> 8);
		comm_reg[COMM_MOT_ENC_L_2] = (uint8_t)((mot.d[LEFT].enc & 0x00ff0000) >> 16);
		comm_reg[COMM_MOT_ENC_L_MSB_3] = (uint8_t)((mot.d[LEFT].enc & 0xff000000) >> 24);
	comm_reg[COMM_MOT_ENC_R_LSB_0] = (uint8_t)(mot.d[RIGHT].enc & 0x000000ff);
		comm_reg[COMM_MOT_ENC_R_1] = (uint8_t)((mot.d[RIGHT].enc & 0x0000ff00) >> 8);
		comm_reg[COMM_MOT_ENC_R_2] = (uint8_t)((mot.d[RIGHT].enc & 0x00ff0000) >> 16);
		comm_reg[COMM_MOT_ENC_R_MSB_3] = (uint8_t)((mot.d[RIGHT].enc & 0xff000000) >> 24);
	comm_reg[COMM_MOT_SPEED_L_IS] = (uint8_t)mot.d[LEFT].speed.is;
	comm_reg[COMM_MOT_SPEED_R_IS] = (uint8_t)mot.d[RIGHT].speed.is;
	mot.d[LEFT].speed.to = (int8_t) comm_reg[COMM_MOT_SPEED_L_TO];
	mot.d[RIGHT].speed.to = (int8_t) comm_reg[COMM_MOT_SPEED_R_TO];
	mot_driver_standby = comm_reg[COMM_MOT_DRIVER_STANDBY];
	rgb_led_mode = comm_reg[COMM_LED_MODE];
	rgb_led_hue = comm_reg[COMM_LED_HUE];
	rgb_led_sat = comm_reg[COMM_LED_SAT];
	rgb_led_val = comm_reg[COMM_LED_VAL];
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
