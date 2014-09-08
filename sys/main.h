////////////////////////////////////////////////////////////////////////////////
//////////////////////////////teamohnename.de///////////////////////////////////
///////////////////////////RoboCup Junior 2014//////////////////////////////////
/////////////////////////////////main.h/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//	Siehe robocup.c
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>
#include <avr/pgmspace.h> 	// Program memory (=Flash ROM) access routines.
#include <avr/wdt.h> //watchdog
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <math.h>
#include <avr/eeprom.h>

///////////////////////////////////////////
#define TASKS_NUM				3
#define TASKPERIOD_GCD			1

#define TASK_PERIOD_TIMER		1
	#define TASK_TIMER_ID			0
#define TASK_PERIOD_SPEEDREG	25
	#define TASK_SPEEDREG_ID		1
#define TASK_PERIOD_ANASENS		50
	#define TASK_ANASENS_ID			2

struct _task
{
	uint8_t running;						// 1 indicates task is running
	int8_t state;								// Current state of state machine
	uint16_t period;						// Rate at which the task should tick
	uint16_t elapsedTime;				// Time since task's previous tick
	int8_t (*task_fct)(int8_t);	// Function to call for task's tick
};

typedef struct _task task;
extern task tasks[TASKS_NUM];

/////////////////////////////////////////

#define TOGGLE_MAIN_LED() PORTD ^= (1<<PD5);

enum DIRECTIONS {LEFT, RIGHT, FRONT, BACK};

extern uint8_t system_status;
extern uint8_t mot_driver_standby;

extern uint8_t rgb_led_mode;
	extern uint8_t rgb_led_hue;
	extern uint8_t rgb_led_sat;
	extern uint8_t rgb_led_val;

extern int8_t batt_percent;

extern uint16_t batt_mV;

extern uint8_t fatal_err;

extern uint8_t debug;

#define TIMER_ENTPR_TAST	500/25  //Timer zum Entprellen von Taster
#define TIMER_GET_TAST		50/25	//So lange muss der Taster gedrückt werden, bis eine Eingabe registriert wird
#define TIMER_BT_IS_BUSY	127 //Timer für Bluetooth DIsplayanzeige (Nach Funkstille Anzeige deaktivieren)
#define TIMER_MAINLOOP		1
#define TIMER_COMM_TIMEOUT	1
#define TIMER_COMM_MOT_TO	2000/25 //If there came no new speed value for this time

extern uint32_t timer;
extern int8_t timer_entpr_tast;
extern int8_t timer_bt_is_busy;
extern int8_t timer_get_tast;
extern int8_t timer_mainloop;
extern int8_t timer_comm_timeout;
extern int8_t timer_comm_mot_to;

#define TRUE 1
#define FALSE 0
