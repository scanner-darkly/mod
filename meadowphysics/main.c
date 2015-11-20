/* issues

tune timing range
refresh preset while in ext trigger mode?

*/

#include <stdio.h>

// asf
#include "delay.h"
#include "compiler.h"
#include "flashc.h"
#include "preprocessor.h"
#include "print_funcs.h"
#include "intc.h"
#include "pm.h"
#include "gpio.h"
#include "spi.h"
#include "sysclk.h"

// skeleton
#include "types.h"
#include "events.h"
#include "i2c.h"
#include "init.h"
#include "interrupts.h"
#include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
#include "ftdi.h"
#include "twi.h"

// this
#include "conf_board.h"
#include "ii.h"
	

#define FIRSTRUN_KEY 0x22

#define L2 12
#define L1 8
#define L0 4

// s8 positions[8] = {3,1,2,2,3,3,5,7};
// s8 points[8] = {3,1,2,2,3,3,5,7};
// s8 points_save[8] = {3,1,2,2,3,3,5,7};
// u8 triggers[8] = {0,0,0,0,0,0,0,0};
// u8 trig_dests[8] = {0,0,0,0,0,0,0,0};
// u8 rules[8] = {0,0,0,0,0,0,0,0};
// u8 rule_dests[8] = {0,1,2,3,4,5,6,7};

u8 edit_row, key_count = 0, mode = 0, prev_mode = 0;
s8 kcount = 0;

const u8 sign[8][8] = {{0,0,0,0,0,0,0,0},         // o
       {0,24,24,126,126,24,24,0},     // +
       {0,0,0,126,126,0,0,0},       // -
       {0,96,96,126,126,96,96,0},     // >
       {0,6,6,126,126,6,6,0},       // <
       {0,102,102,24,24,102,102,0},   // * rnd
       {0,120,120,102,102,30,30,0},   // <> up/down
       {0,126,126,102,102,126,126,0}};  // [] return

const u8 outs[8] = {B00, B01, B02, B03, B04, B05, B06, B07};


typedef struct {
	s8 positions[8];
	s8 points[8];
	s8 points_save[8];
	u8 triggers[8];
	u8 trig_dests[8];
	u8 rules[8];
	u8 rule_dests[8];
	u8 mutes[8];
	u8 freezes[8];
} mp_set;

typedef const struct {
	u8 fresh;
	u8 preset_select;
	u8 glyph[8][8];
	mp_set m[8];
} nvram_data_t;

mp_set m;

u8 preset_mode, preset_select, front_timer;
u8 glyph[8];

u8 clock_phase;
u16 clock_time, clock_temp;

u16 adc[4];
u8 SIZE, LENGTH, VARI;

u8 held_keys[32], key_times[256];

typedef void(*re_t)(void);
re_t re;


// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;




////////////////////////////////////////////////////////////////////////////////
// prototypes

static void refresh(void);
static void refresh_mono(void);
static void refresh_preset(void);
static void clock(u8 phase);

// start/stop monome polling/refresh timers
extern void timers_set_monome(void);
extern void timers_unset_monome(void);

// check the event queue
static void check_events(void);

// handler protos
static void handler_None(s32 data) { ;; }
static void handler_KeyTimer(s32 data);
static void handler_Front(s32 data);
static void handler_ClockNormal(s32 data);


u8 flash_is_fresh(void);
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);


static void mp_process_ii(uint8_t i, int d);


static void cascades_trigger(u8 n);



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application clock code

void clock(u8 phase) {
	static u8 i;

	if(phase) {
		gpio_set_gpio_pin(B10);

		// clear last round
		for(i=0;i<8;i++)
			m.triggers[i] = 0;

		// main
		if(!m.freezes[0])
			cascades_trigger(0);

		// ensure bounds, output triggers
		for(i=0;i<8;i++) {
			if(m.positions[i] < 0)
				m.positions[i] = 0;
			else if(m.positions[i] > m.points[i])
				m.positions[i] = m.points[i];

			// send out
			if(m.triggers[i] && !m.mutes[i])
				gpio_set_gpio_pin(outs[i]);
		}

		monomeFrameDirty++;
	}
	else {
		for(i=0;i<8;i++) gpio_clr_gpio_pin(outs[i]);

		gpio_clr_gpio_pin(B10);
 	}
}



////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer  = { .next = NULL, .prev = NULL };



static void clockTimer_callback(void* o) {  
	if(clock_external == 0) {
		// print_dbg("\r\ntimer.");

		clock_phase++;
		if(clock_phase>1) clock_phase=0;
		(*clock_pulse)(clock_phase);
	}
}

static void keyTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventKeyTimer;
	e.data = 0;
	event_post(&e);
}

static void adcTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventPollADC;
	e.data = 0;
	event_post(&e);
}


// monome polling callback
static void monome_poll_timer_callback(void* obj) {
  // asynchronous, non-blocking read
  // UHC callback spawns appropriate events
	ftdi_read();
}

// monome refresh callback
static void monome_refresh_timer_callback(void* obj) {
	if(monomeFrameDirty > 0) {
		static event_t e;
		e.type = kEventMonomeRefresh;
		event_post(&e);
	}
}

// monome: start polling
void timers_set_monome(void) {
	// print_dbg("\r\n setting monome timers");
	timer_add(&monomePollTimer, 20, &monome_poll_timer_callback, NULL );
	timer_add(&monomeRefreshTimer, 30, &monome_refresh_timer_callback, NULL );
}

// monome stop polling
void timers_unset_monome(void) {
	// print_dbg("\r\n unsetting monome timers");
	timer_remove( &monomePollTimer );
	timer_remove( &monomeRefreshTimer ); 
}



////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) { ftdi_setup(); }
static void handler_FtdiDisconnect(s32 data) { 
	timers_unset_monome();
	// event_t e = { .type = kEventMonomeDisconnect };
	// event_post(&e);
}

static void handler_MonomeConnect(s32 data) {
	// print_dbg("\r\n// monome connect /////////////////"); 
	key_count = 0;
	SIZE = monome_size_x();
	LENGTH = SIZE - 1;
	// print_dbg("\r monome size: ");
	// print_dbg_ulong(SIZE);
	VARI = monome_is_vari();
	// print_dbg("\r monome vari: ");
	// print_dbg_ulong(VARI);

	if(VARI) re = &refresh;
	else re = &refresh_mono;


	// monome_set_quadrant_flag(0);
	// monome_set_quadrant_flag(1);
	timers_set_monome();
}

static void handler_MonomePoll(s32 data) { monome_read_serial(); }
static void handler_MonomeRefresh(s32 data) {
	if(monomeFrameDirty) {
		if(preset_mode == 0) (*re)(); //refresh_mono();
		else refresh_preset();

		(*monome_refresh)();
	}
}


static void handler_Front(s32 data) {
	print_dbg("\r\n FRONT HOLD");

	if(data == 0) {
		front_timer = 15;
		if(preset_mode) preset_mode = 0;
		else preset_mode = 1;
	}
	else {
		front_timer = 0;
	}

	monomeFrameDirty++;
}

static void handler_PollADC(s32 data) {
	u16 i;
	adc_convert(&adc);

	// CLOCK POT INPUT
	i = adc[0];
	i = i>>2;
	if(i != clock_temp) {
		// 500ms - 12ms
		clock_time = 12500 / (i + 25);
		// print_dbg("\r\nclock (ms): ");
		// print_dbg_ulong(clock_time);

		timer_set(&clockTimer, clock_time);
	}
	clock_temp = i;
}

static void handler_SaveFlash(s32 data) {
	flash_write();
}

static void handler_KeyTimer(s32 data) {
	static u16 i1;

	if(front_timer) {
		if(front_timer == 1) {
			static event_t e;
			e.type = kEventSaveFlash;
			event_post(&e);

			preset_mode = 0;
			front_timer--;
		}
		else front_timer--;
	}

	for(i1=0;i1<key_count;i1++) {
		if(key_times[held_keys[i1]])
		if(--key_times[held_keys[i1]]==0) {
			if(preset_mode == 1) {
				if(held_keys[i1] % 16 == 0) {
					preset_select = held_keys[i1] / 16;
					// flash_write();
					static event_t e;
					e.type = kEventSaveFlash;
					event_post(&e);
					preset_mode = 0;
				}
			}

			// print_dbg("\rlong press: "); 
			// print_dbg_ulong(held_keys[i1]);
		}
	}
}

static void handler_ClockNormal(s32 data) {
	clock_external = !gpio_get_pin_value(B09); 
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application grid code

static void handler_MonomeGridKey(s32 data) { 
	u8 x, y, z, index, i1, found;
	monome_grid_key_parse_event_data(data, &x, &y, &z);
	// print_dbg("\r\n monome event; x: "); 
	// print_dbg_hex(x); 
	// print_dbg("; y: 0x"); 
	// print_dbg_hex(y); 
	// print_dbg("; z: 0x"); 
	// print_dbg_hex(z);

	//// TRACK LONG PRESSES
	index = y*16 + x;
	if(z) {
		held_keys[key_count] = index;
		key_count++;
		key_times[index] = 10;		//// THRESHOLD key hold time
	} else {
		found = 0; // "found"
		for(i1 = 0; i1<key_count; i1++) {
			if(held_keys[i1] == index) 
				found++;
			if(found) 
				held_keys[i1] = held_keys[i1+1];
		}
		key_count--;

		// FAST PRESS
		if(key_times[index] > 0) {
			if(preset_mode == 1) {
				if(x == 0 && y != preset_select) {
					preset_select = y;
					for(i1=0;i1<8;i1++)
						glyph[i1] = flashy.glyph[preset_select][i1];
				}
 				else if(x==0 && y == preset_select) {
					flash_read();

					preset_mode = 0;
				}

				monomeFrameDirty++;	
			}
			// print_dbg("\r\nfast press: ");
			// print_dbg_ulong(index);
			// print_dbg(": ");
			// print_dbg_ulong(key_times[index]);
		}
	}

	// PRESET SCREEN
	if(preset_mode) {
		// glyph magic
		if(z && x>7) {
			glyph[y] ^= 1<<(x-8);
		}

		monomeFrameDirty++;	
	}
	// NOT PRESET
	else {

		prev_mode = mode;

		// mode check
		if(x == 0) {
			kcount += (z<<1)-1;

			if(kcount < 0)
				kcount = 0;

			// print_dbg("\r\nkey count: ");
			// print_dbg_ulong(kcount);

			if(kcount == 1 && z == 1)
				mode = 1; 
			else if(kcount == 0)
				mode = 0;

			if(z == 1 && mode == 1) {
				edit_row = y;
				monomeFrameDirty++;
			}
		}
		else if(x == 1 && mode != 0) {
			if(mode == 1 && z == 1)
				mode = 2;
			else if(mode == 2 && z == 0)
				mode = 1;
		}
		else if(mode == 0 && z == 1) {
			m.points[y] = x;
			m.points_save[y] = x;
			m.positions[y] = x;
			monomeFrameDirty++;
		}
		else if(mode == 1 && z == 1) {
			if(x > 1 && x < 7) {
				if(y != edit_row) {    // filter out self-triggering
					m.trig_dests[edit_row] ^= (1<<y);
					monomeFrameDirty++;
				  // post("\ntrig_dests", edit_row, ":", trig_dests[edit_row]);
				}
			}
			else if(x == 15)
				m.freezes[y] ^= 1;
			else if(x == 14)
				m.mutes[y] ^= 1;
		}
		else if(mode == 2 && z == 1) {
			if(x > 1 && x < 7) {
				m.rule_dests[edit_row] = y;
				monomeFrameDirty++;
			  // post("\nrule_dests", edit_row, ":", rule_dests[edit_row]);
			}
			else if(x > 6) {
				m.rules[edit_row] = y;
				monomeFrameDirty++;
			  // post("\nrules", edit_row, ":", rules[edit_row]);
			}
		}

		if(mode != prev_mode) {
			monomeFrameDirty++;
			// post("\nnew mode", mode);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// application grid redraw
static void refresh() {
	u8 i1, i2, i3;

	// clear grid
	for(i1=0;i1<128;i1++)
		monomeLedBuffer[i1] = 0;

	// SET POSITIONS
	if(mode == 0) {
		for(i1=0;i1<8;i1++) {
			for(i2=m.positions[i1];i2<=m.points[i1];i2++)
				monomeLedBuffer[i1*16 + i2] = L1;

			monomeLedBuffer[i1*16 + m.positions[i1]] = L2;
		}
	}
	// SET ROUTING
	else if(mode == 1) {
		for(i1=0;i1<8;i1++) {
			if((m.trig_dests[edit_row] & (1<<i1)) != 0) {
				for(i2=0;i2<=m.points[i1];i2++)
					monomeLedBuffer[i1*16 + i2] = L0;
				monomeLedBuffer[i1*16 + m.positions[i1]] = L2;
			}
			else
				monomeLedBuffer[i1*16 + m.positions[i1]] = L0;

			if(m.freezes[i1])
				monomeLedBuffer[i1*16+15] = L2;
			if(m.mutes[i1])
				monomeLedBuffer[i1*16+14] = L1;

		}

		monomeLedBuffer[edit_row * 16] = L2;
	}
	// SET RULES
	else if(mode == 2) {
		monomeLedBuffer[edit_row * 16] = L1;
		monomeLedBuffer[edit_row * 16 + 1] = L1;

		for(i1=2;i1<7;i1++)
			monomeLedBuffer[m.rule_dests[edit_row] * 16 + i1] = L2;

		for(i1=8;i1<16;i1++)
			monomeLedBuffer[m.rules[edit_row] * 16 + i1] = L0;

		for(i1=0;i1<8;i1++) 
			monomeLedBuffer[i1*16 + m.positions[i1]] = L0;

		for(i1=0;i1<8;i1++) {
			i3 = sign[m.rules[edit_row]][i1];
			for(i2=0;i2<8;i2++) {
				if((i3 & (1<<i2)) != 0)
					monomeLedBuffer[i1*16 + 8 + i2] = L2;
			}
		}

		monomeLedBuffer[m.rules[edit_row] * 16 + 7] = L2;
	}

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}


// application grid redraw without varibright
static void refresh_mono() {
	refresh();

	// monome_set_quadrant_flag(0);
	// monome_set_quadrant_flag(1);
}


static void refresh_preset() {
	u8 i1,i2;

	for(i1=0;i1<128;i1++)
		monomeLedBuffer[i1] = 0;

	monomeLedBuffer[preset_select * 16] = 11;

	for(i1=0;i1<8;i1++)
		for(i2=0;i2<8;i2++)
			if(glyph[i1] & (1<<i2))
				monomeLedBuffer[i1*16+i2+8] = 11;

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}



static void cascades_trigger(u8 n) {
  u8 i;

  m.positions[n]--;

  // ****** the trigger # check is so we don't cause a trigger/rules multiple times per NEXT
  // a rules-based jump to position-point does not current cause a trigger. should it?
  if(m.positions[n] < 0 && m.triggers[n] == 0) {
    m.triggers[n]++;
  
    if(m.rules[n] == 1) {     // inc
      if(m.points[m.rule_dests[n]] < (LENGTH)) {
        m.points[m.rule_dests[n]]++;
        // m.positions[m.rule_dests[n]] = m.points[m.rule_dests[n]];
      }
    }
    else if(m.rules[n] == 2) {  // dec
      if(m.points[m.rule_dests[n]] > 0) {
        m.points[m.rule_dests[n]]--;
        // m.positions[m.rule_dests[n]] = m.points[m.rule_dests[n]];
      }
    }
    else if(m.rules[n] == 3) {  // max
      m.points[m.rule_dests[n]] = (LENGTH);
      // m.positions[m.rule_dests[n]] = m.points[m.rule_dests[n]];
    }
    else if(m.rules[n] == 4) {  // min
      m.points[m.rule_dests[n]] = 0;
      // m.positions[m.rule_dests[n]] = m.points[m.rule_dests[n]];
    }
    else if(m.rules[n] == 5) {  // rnd
      m.points[m.rule_dests[n]] = rnd() % SIZE;
      
      // print_dbg("\r\n RANDOM: ");
      // print_dbg_hex(m.points[m.rule_dests[n]]);
      // print_dbg_hex(rnd() % 11);

      // m.positions[m.rule_dests[n]] = m.points[m.rule_dests[n]];
    }
    else if(m.rules[n] == 6) {  // up/down
      m.points[m.rule_dests[n]] += rnd() % 3;
      m.points[m.rule_dests[n]]--;


      if(m.points[m.rule_dests[n]] < 0) m.points[m.rule_dests[n]] = 0;
      else if(m.points[m.rule_dests[n]] > (LENGTH)) m.points[m.rule_dests[n]] = LENGTH;
      // m.positions[m.rule_dests[n]] = m.points[m.rule_dests[n]];  

      // print_dbg("\r\n WANDER: ");
      // print_dbg_hex(m.points[m.rule_dests[n]]);   
    }
    else if(m.rules[n] == 7) {  // return
      m.points[m.rule_dests[n]] = m.points_save[m.rule_dests[n]];
    }


    //reset
    m.positions[n] += m.points[n] + 1;

    //triggers
    for(i=0;i<8;i++)
      if(((m.trig_dests[n] & (1<<i)) != 0) && !m.freezes[i])
        cascades_trigger(i);
        // post("\ntrigger",n," -> ", m);
  }
}


static void mp_process_ii(uint8_t i, int d) {
	switch(i) {
		case MP_PRESET:
			if(d<0 || d>7)
				break;
			preset_select = d;
			flash_read();
			break;
		case MP_RESET:
			if(d) {
				m.positions[0] = m.points[0]+1;
				for(int n=1;n<8;n++)
					m.positions[n] = m.points[n];
			}
			break;
		case MP_SYNC:
			if(d) {
				m.positions[0] = m.points[0]+1;
				for(int n=1;n<8;n++)
					m.positions[n] = m.points[n];
				timer_set(&clockTimer,clock_time);
				clock_phase = 1;
				(*clock_pulse)(clock_phase);
			}
			break;
		case MP_MUTE:
			if(d<1 || d>8)
				break;
			d--;
			m.mutes[d] = 1;
			break;
		case MP_UNMUTE:
			if(d<1 || d>8)
				break;
			d--;
			m.mutes[d] = 0;
			break;
		case MP_FREEZE:
			if(d<1 || d>8)
				break;
			d--;
			m.freezes[d] = 1;
			break;
		case MP_UNFREEZE:
			if(d<1 || d>8)
				break;
			d--;
			m.freezes[d] = 0;
			break;
		default:
			break;
	}
  // print_dbg("\r\nmp: ");
  // print_dbg_ulong(i);
  // print_dbg(" ");
  // print_dbg_ulong(d);
}







// assign event handlers
static inline void assign_main_event_handlers(void) {
	app_event_handlers[ kEventFront ]	= &handler_Front;
	// app_event_handlers[ kEventTimer ]	= &handler_Timer;
	app_event_handlers[ kEventPollADC ]	= &handler_PollADC;
	app_event_handlers[ kEventKeyTimer ] = &handler_KeyTimer;
	app_event_handlers[ kEventSaveFlash ] = &handler_SaveFlash;
	app_event_handlers[ kEventClockNormal ] = &handler_ClockNormal;
	app_event_handlers[ kEventFtdiConnect ]	= &handler_FtdiConnect ;
	app_event_handlers[ kEventFtdiDisconnect ]	= &handler_FtdiDisconnect ;
	app_event_handlers[ kEventMonomeConnect ]	= &handler_MonomeConnect ;
	app_event_handlers[ kEventMonomeDisconnect ]	= &handler_None ;
	app_event_handlers[ kEventMonomePoll ]	= &handler_MonomePoll ;
	app_event_handlers[ kEventMonomeRefresh ]	= &handler_MonomeRefresh ;
	app_event_handlers[ kEventMonomeGridKey ]	= &handler_MonomeGridKey ;
}

// app event loop
void check_events(void) {
	static event_t e;
	if( event_next(&e) ) {
		(app_event_handlers)[e.type](e.data);
	}
}

// flash commands
u8 flash_is_fresh(void) {
  return (flashy.fresh != FIRSTRUN_KEY);
  // flashc_memcpy((void *)&flashy.fresh, &i, sizeof(flashy.fresh),   true);
  // flashc_memset32((void*)&(flashy.fresh), fresh_MAGIC, 4, true);
  // flashc_memset((void *)nvram_data, 0x00, 8, sizeof(*nvram_data), true);
}

// write fresh status
void flash_unfresh(void) {
  flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
}

void flash_write(void) {
	// print_dbg("\r write preset ");
	// print_dbg_ulong(preset_select);
	flashc_memcpy((void *)&flashy.m[preset_select], &m, sizeof(m), true);
	flashc_memcpy((void *)&flashy.glyph[preset_select], &glyph, sizeof(glyph), true);
	flashc_memset8((void*)&(flashy.preset_select), preset_select, 1, true);
}

void flash_read(void) {
	u8 i1;

	print_dbg("\r\n read preset ");
	print_dbg_ulong(preset_select);

	// for(i1=0;i1<16;i1++) {
	// 	for(i2=0;i2<16;i2++) {
	// 		w.wp[i1].steps[i2] = flashy.w[preset_select].wp[i1].steps[i2];
	// 		w.wp[i1].step_probs[i2] = flashy.w[preset_select].wp[i1].step_probs[i2];
	// 		w.wp[i1].cv_probs[0][i2] = flashy.w[preset_select].wp[i1].cv_probs[0][i2];
	// 		w.wp[i1].cv_probs[1][i2] = flashy.w[preset_select].wp[i1].cv_probs[1][i2];
	// 		w.wp[i1].cv_curves[0][i2] = flashy.w[preset_select].wp[i1].cv_curves[0][i2];
	// 		w.wp[i1].cv_curves[1][i2] = flashy.w[preset_select].wp[i1].cv_curves[1][i2];
	// 		w.wp[i1].cv_steps[0][i2] = flashy.w[preset_select].wp[i1].cv_steps[0][i2];
	// 		w.wp[i1].cv_steps[1][i2] = flashy.w[preset_select].wp[i1].cv_steps[1][i2];
	// 		w.wp[i1].cv_values[i2] = flashy.w[preset_select].wp[i1].cv_values[i2];
	// 	}

	// 	w.wp[i1].step_choice = flashy.w[preset_select].wp[i1].step_choice;
	// 	w.wp[i1].loop_end = flashy.w[preset_select].wp[i1].loop_end;
	// 	w.wp[i1].loop_len = flashy.w[preset_select].wp[i1].loop_len;
	// 	w.wp[i1].loop_start = flashy.w[preset_select].wp[i1].loop_start;
	// 	w.wp[i1].loop_dir = flashy.w[preset_select].wp[i1].loop_dir;
	// 	w.wp[i1].step_mode = flashy.w[preset_select].wp[i1].step_mode;
	// 	w.wp[i1].cv_mode[0] = flashy.w[preset_select].wp[i1].cv_mode[0];
	// 	w.wp[i1].cv_mode[1] = flashy.w[preset_select].wp[i1].cv_mode[1];
	// }

	for(i1=0;i1<8;i1++) {
		m.positions[i1] = flashy.m[preset_select].positions[i1];
		m.points[i1] = flashy.m[preset_select].points[i1];
		m.points_save[i1] = flashy.m[preset_select].points_save[i1];
		m.triggers[i1] = flashy.m[preset_select].triggers[i1];
		m.trig_dests[i1] = flashy.m[preset_select].trig_dests[i1];
		m.rules[i1] = flashy.m[preset_select].rules[i1];
		m.rule_dests[i1] = flashy.m[preset_select].rule_dests[i1];
	}
}






////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// main


int main(void) {
	u8 i1;

	sysclk_init();

	init_dbg_rs232(FMCK_HZ);

	init_gpio();
	assign_main_event_handlers();
	init_events();
	init_tc();
	init_spi();
	init_adc();

	irq_initialize_vectors();
	register_interrupts();
	cpu_irq_enable();

	init_usb_host();
	init_monome();

	init_i2c_slave(0x30);

	print_dbg("\r\n\n// meadowphysics //////////////////////////////// ");
	print_dbg_ulong(sizeof(flashy));

	print_dbg(" ");
	print_dbg_ulong(sizeof(m));


	if(flash_is_fresh()) {
		print_dbg("\r\nfirst run.");
		flash_unfresh();
		flashc_memset32((void*)&(flashy.preset_select), 0, 4, true);


		// clear out some reasonable defaults
		for(i1=0;i1<8;i1++) {
			m.positions[i1] = i1;
			m.points[i1] = i1;
			m.points_save[i1] = i1;
			m.triggers[i1] = 0;
			m.trig_dests[i1] = 0;
			m.rules[i1] = 0;
			m.rule_dests[i1] = i1;
		}

		m.positions[0] = m.points[0] = 3;
		m.trig_dests[0] = 254;

		// save all presets, clear glyphs
		for(i1=0;i1<8;i1++) {
			flashc_memcpy((void *)&flashy.m[i1], &m, sizeof(m), true);
			glyph[i1] = (1<<i1);
			flashc_memcpy((void *)&flashy.glyph[i1], &glyph, sizeof(glyph), true);
		}
	}
	else {
		// load from flash at startup
		preset_select = flashy.preset_select;
		flash_read();
		for(i1=0;i1<8;i1++)
			glyph[i1] = flashy.glyph[preset_select][i1];
	}

	LENGTH = 15;
	SIZE = 16;

	re = &refresh;

	process_ii = &mp_process_ii;

	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	timer_add(&clockTimer,120,&clockTimer_callback, NULL);
	timer_add(&keyTimer,50,&keyTimer_callback, NULL);
	timer_add(&adcTimer,100,&adcTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo

	// setup daisy chain for two dacs
	// spi_selectChip(SPI,DAC_SPI);
	// spi_write(SPI,0x80);
	// spi_write(SPI,0xff);
	// spi_write(SPI,0xff);
	// spi_unselectChip(SPI,DAC_SPI);

	while (true) {
		check_events();
	}
}
