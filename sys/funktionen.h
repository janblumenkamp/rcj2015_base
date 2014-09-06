////////////////////////////////////////////////////////////////////////////////
//////////////////////////////teamohnename.de///////////////////////////////////
///////////////////////////RoboCup Junior 2014//////////////////////////////////
//////////////////////////////funktionen.h//////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//	Siehe funktionen.c
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>

extern int16_t dist[2][4][4];
extern int16_t dist_down;
extern int16_t groundsens_l;
extern int16_t groundsens_r;

extern int16_t batt_raw;

struct _sensblock {
	unsigned left:1;
	unsigned right:1;
	unsigned mid:1;
};

struct _sensinfo {
	struct _sensblock request;
	struct _sensblock newDat;
};

typedef struct _sensinfo __sensinfo;

#define DIST_MAX_SRP_OLD 200
#define DIST_MAX_SRP_NEW 400

#define RAW 0
#define LIN 1
extern __sensinfo sensinfo;
////////////////////

extern void limitVar(int16_t *var, int16_t limit);

extern void abs_ptr(int16_t *var);

extern void led_hsvToRgb (uint8_t hue, uint8_t  saturation, uint8_t value); //Farbe, Sättigung, Helligkeit

extern void led_rgb(uint16_t heartbeat_color, uint16_t led_error, uint8_t led_top);

extern int16_t *get_sorted(uint8_t cnt, int16_t *data, uint8_t get);

extern void dist_init(void);

extern int16_t get_adc(uint8_t channel);

extern void dist_setSensors(uint8_t block, uint8_t set);

extern void get_analogSensors(void);
