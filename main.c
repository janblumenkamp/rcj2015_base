////////////////////////////////////////////////////////////////////////////////
////////////////////////////////JuFo 2015///////////////////////////////////////
/////////////////////////////////main.c/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///	AVR Subcontroller, Bereitstellung von AD-Wandlungen/Sensoren, Motorregelung,
/// Auswertung der Batterie, Debug LED; Kommunikation über UART1
///
/// Kopf des Programmes mit Endlosschleife und Timer ISR
///	-> Zusammenführung sämtlicher anderer Funktionen
///		- Auswerten der Encoder und des Inkrementalgebers
///		- Timer
///		- Aufrufen der Initialisierungsfunktion
///		- Batterieauswertung
///		- Fehleranzeige via RGB LED
////////////////////////////////////////////////////////////////////////////////
///	To do:
/// -
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "bluetooth.h"
#include "uart.h"
#include "system.h"
#include "funktionen.h"
#include "drive.h"
#include "memcheck.h"
#include "comm.h"

////////////////////////////////////////////////////////////////////
//UART:
#define UART_MCU_BAUD_RATE    115200

//////////////////////////////////////////////////////////////////////
#define WDT_TRIGGERED() if(MCUSR & (1<<WDRF)) check_res = 1;

////////////////////////////////////////////////////////////////////////////////
//Scheduler: http://www.riosscheduler.org/

task tasks[TASKS_NUM];

int8_t task_speedreg(int8_t state);
int8_t task_timer(int8_t state);
int8_t task_anasens(int8_t state);

uint8_t runningTasks[TASKS_NUM+1] = {255};	//Track running tasks, [0] always idleTask
uint8_t idleTask = 255;						// 0 highest priority, 255 lowest
uint8_t currentTask = 0;					// Index of highest priority task in runningTasks

uint16_t schedule_time = 0;

////////////////////////////////////////////////////////////////////////
uint8_t check_res = 0; //resetsource?
uint8_t fatal_err = 0; //Generel error, log has to be checked when it appears!

//////////////////////////////////////
uint8_t led_heartbeatColor = 85;	//Heartbeatcolor (0 == no heartbeat)
uint8_t led_fault = 0;				//Set to != 0, when an error has to be signalised (LED will brightly blink in the color of led_fault)
uint8_t led_top = 0;				//Point when the led swells down -> indirectly speed of led pulse/blink

#define LED_TOP_FAT_ERR 10
#define LED_TOP_NORMAL	30

#define TOGGLE_MAIN_LED() PORTD ^= (1<<PD5);
/////////////////////////////////

int8_t batt_percent = 0;
uint16_t batt_mV_old = 0xffff;
uint16_t batt_mV = 0;

///////////////////////////////Drehgeber/Encoder/Taster/////////////////////////

#define ENC_L_PHASE_A     (PINA & (1<<PA0))
#define ENC_L_PHASE_B     (PINE & (1<<PE7))

static int8_t enc_l_last = 0;
//Encoderwert in Datenstruktur für Motoren

#define ENC_R_PHASE_A     (PINE & (1<<PE6))
#define ENC_R_PHASE_B     (PINA & (1<<PA1))

static int8_t enc_r_last = 0;
//Encoderwert in Datenstruktur für Motoren

//////////
uint8_t hold_t1 = 0;
////////////////////////////////////Sonstiges///////////////////////////////////

uint8_t setup = 0; //Start the setup?

uint8_t debug = 0; //Debugmodus, Read from EEPROM in init_sys()
uint8_t debug_err_sendOneTime = 0;
///////////////////////////////Timer////////////////////////////////////////////

int8_t timer_entpr_tast = 0;
int8_t timer_bt_is_busy = 0;
int8_t timer_get_tast = 0;

uint8_t timer_25ms = 0; //make the upper timers decrement only every 25ms in the timer task

uint32_t timer = 0; //Timer, resolution 1ms, continuisly incrementing in the scheduler ISR

////////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect) //1kHz
{
	timer++;

	for(uint8_t i = 0; i < TASKS_NUM; i++) 					// Heart of scheduler code
	{
		if((tasks[i].elapsedTime >= tasks[i].period)	// Task ready
			&& (runningTasks[currentTask] > i)					// Task priority > current task priority
			&& (!tasks[i].running))											// Task not already running (no self-preemption)
		{
			cli();
			tasks[i].elapsedTime = 0;										// Reset time since last tick
			tasks[i].running = 1;												// Mark as running
			currentTask += 1;
			runningTasks[currentTask] = i;							// Add to runningTasks
			sei();

			tasks[i].state = tasks[i].task_fct(tasks[i].state);	// Execute tick

			cli();
			tasks[i].running = 0;												// Mark as not running
			runningTasks[currentTask] = idleTask;				// Remove from runningTasks
			currentTask -= 1;
			sei();
		}
		/*else if((tasks[i].elapsedTime >= tasks[i].period)	// Task ready
				&& (tasks[i].running))											// Task still running
		{
			displayvar[4] = i;
			tasks[i].period = 1000;
			tasks[i].running = 0;
		}*/
		tasks[i].elapsedTime += TASKPERIOD_GCD;
	}
}

/*### Mainloop ###*/
int main(void)
{
	WDT_TRIGGERED(); //Watchdogreset?

	init_sys();
	init_pwm();
	init_timer();
	dist_init();
	uart_init(UART_BAUD_SELECT(UART_MCU_BAUD_RATE,F_CPU)); //Bluetooth
	comm_init();
	init_adc();

	mot.off = 1;
	//The higher the task_i of the task is, the higher is the priority
	
	tasks[TASK_TIMER_ID].state = -1;
	tasks[TASK_TIMER_ID].period = TASK_PERIOD_TIMER;
	tasks[TASK_TIMER_ID].elapsedTime = 0;
	tasks[TASK_TIMER_ID].running = 0;
	tasks[TASK_TIMER_ID].task_fct = &task_timer;

	tasks[TASK_SPEEDREG_ID].state = -1;
	tasks[TASK_SPEEDREG_ID].period = TASK_PERIOD_SPEEDREG;
	tasks[TASK_SPEEDREG_ID].elapsedTime = 0;
	tasks[TASK_SPEEDREG_ID].running = 0;
	tasks[TASK_SPEEDREG_ID].task_fct = &task_speedreg;

	tasks[TASK_ANASENS_ID].state = -1;
	tasks[TASK_ANASENS_ID].period = TASK_PERIOD_ANASENS;
	tasks[TASK_ANASENS_ID].elapsedTime = 0;
	tasks[TASK_ANASENS_ID].running = 0;
	tasks[TASK_ANASENS_ID].task_fct = &task_anasens;

	if(get_incrOk())
	{
		motor_activate(0); //Shut down motor driver
		setup = 1;
	}
	else
	{
		motor_activate(1); //Activate motor driver
		setup = 0;
	}
	
	sei(); //Enable global interrupts. The Operating System and every task in it is running now and the cam already can regulate its initial aparture

	bt_putStr_P(PSTR("\r\n\n\n\n\n\n\n\n"));
	bt_putStr_P(PSTR("–––––––––––––––––––––––\r\n"));
	bt_putStr_P(PSTR("| RIOS Scheduler v1.0 |\r\n"));
	bt_putStr_P(PSTR("–––––––––––––––––––––––\r\n"));
	bt_putStr_P(PSTR("Jugend Forscht 2015 v1.0\r\n"));
	bt_putStr_P(PSTR("Subcontroller ATmega2560\r\n"));
	bt_putStr_P(PSTR("\n\r")); bt_putLong(timer); bt_putStr_P(PSTR(": System initialized, ")); bt_putLong(TASKS_NUM); bt_putStr_P(PSTR(" running tasks."));
	
	if(check_res)
	{
		motor_activate(0); //Shut down motor driver
		if(debug > 0){bt_putStr_P(PSTR("\n\r")); bt_putLong(timer); bt_putStr(PSTR(": WARNING: RECOVERED AFTER AN UNEXPECTED SHUTDOWN!!!"));}
		_delay_ms(5000);
	}

	wdt_enable(WDTO_1S); //activate watchdog

	while(1)
    {
		wdt_reset();

		TOGGLE_MAIN_LED(); //Toggle LED on the RNmega Board

		////////////////////////////////////////////////////////////////////////////
		if(get_t1())
		{
			if((timer_get_tast == 0) && (timer_entpr_tast == 0) && (!hold_t1))
			{
				timer_get_tast = TIMER_GET_TAST;
				hold_t1 = 1;
			}
			else if((timer_get_tast == 0) && hold_t1)
			{
				hold_t1 = 0;
				mot.off ^= 1;
				timer_entpr_tast = TIMER_ENTPR_TAST;
			}
		}
		else
		{
			timer_get_tast = 0;
			hold_t1 = 0;
		}

		////////////////////Sensorcoordination//////////////////////////////////////

		if(check_res || fatal_err)	//Flashing of the Info LED when there is an error
			led_fault = 1;
		else
			led_fault = 0;
		
		if(fatal_err)
			led_top = LED_TOP_FAT_ERR;
		else
			led_top = LED_TOP_NORMAL;
			
		if(check_res)
		{
			if(!(debug_err_sendOneTime & (1<<0)))
			{
				if(debug > 1){bt_putStr_P(PSTR("\n\r")); bt_putLong(timer); bt_putStr_P(PSTR(": ERROR: RESET"));}
				debug_err_sendOneTime |= (1<<0);
			}
		}
		else	debug_err_sendOneTime &= ~(1<<0);
				
		//Batterie/Akku
		if(batt_raw > 0)
		{
			batt_mV = (batt_raw*15)-300;

			if(batt_mV < batt_mV_old)
			{
				batt_mV_old = batt_mV;
				batt_percent = (0.037*batt_mV) - 363;
			}
			if(batt_percent < 20) //Batterie
				led_heartbeatColor = batt_percent;
		}

		////////////////////////////////////////////////////////////////////////////
		//LED heartbeat

		led_rgb(led_heartbeatColor, led_fault, led_top);
  }
	
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////TASK SPEEDREG/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int8_t task_speedreg(int8_t state)
{
	//Turn motor off?
	if(	mot.off || mot.off_invisible)
	{
		mot.d[LEFT].speed.to = 0;
		mot.d[RIGHT].speed.to = 0;
	}
	controlSpeed(); //Speed Regulation
	
	return 0;
}

int8_t task_anasens(int8_t state)
{
	//analog
	get_analogSensors(); //Sharp infrared distance sensors, groundsensor
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////TASK TIMER//////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int8_t task_timer(int8_t state)
{
	//////(rotary) encoder///////////
	//Source: http://www.mikrocontroller.net/articles/Drehgeber

	int8_t new, diff;
			 
	new = 0;
	if(ENC_L_PHASE_A)		new = 3;
	if(ENC_L_PHASE_B)		new ^= 1;		// convert gray to binary
	diff = enc_l_last - new;				// difference last - new
	if( diff & 1 ){									// bit 0 = value (1)
		enc_l_last = new;							// store new as next last

		mot.d[LEFT].enc += (diff & 2) - 1;	// bit 1 = direction (+/-)
		mot.d[LEFT].enc_abs += abs((diff & 2) -1);
	}

	new = 0;
	if(ENC_R_PHASE_A)		new = 3;
	if(ENC_R_PHASE_B)		new ^= 1;		// convert gray to binary
	diff = enc_r_last - new;				// difference last - new
	if( diff & 1 ){									// bit 0 = value (1)
		enc_r_last = new;							// store new as next last
		mot.d[RIGHT].enc += (diff & 2) - 1;	// bit 1 = direction (+/-)
		mot.d[RIGHT].enc_abs += abs((diff & 2) -1);
	}

	mot.enc = ((mot.d[LEFT].enc/2) + (mot.d[RIGHT].enc/2));

	//////Timer/////////////

	timer_25ms ++;
	if(timer_25ms == 25) //40Hz
	{
		if(timer_entpr_tast > 0)
			timer_entpr_tast --;
		if(timer_bt_is_busy > 0)
			timer_bt_is_busy --;
		if(timer_get_tast > 0)
			timer_get_tast --;

		timer_25ms = 0;
	}

	return 0;
}
