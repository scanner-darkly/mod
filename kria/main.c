/* issues


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

// this
#include "conf_board.h"
#include "ii.h"
	

#define L2 12
#define L1 8
#define L0 4

#define FIRSTRUN_KEY 0x22


// for i in range(0,120):
// print '%.f, ' % (i * 4092.0 / 120.0)
const u16 ET[120] = {
	0, 34, 68, 102, 136, 170, 205, 239, 273, 307, 341, 375, 409, 443, 477, 511, 545, 580, 614, 648, 682, 716, 750, 784, 818, 852, 886,
	920, 955, 989, 1023, 1057, 1091, 1125, 1159, 1193, 1227, 1261, 1295, 1330, 1364, 1398, 1432, 1466, 1500, 1534, 1568, 1602, 1636,
	1670, 1705, 1739, 1773, 1807, 1841, 1875, 1909, 1943, 1977, 2011, 2046, 2080, 2114, 2148, 2182, 2216, 2250, 2284, 2318, 2352,
	2386, 2421, 2455, 2489, 2523, 2557, 2591, 2625, 2659, 2693, 2727, 2761, 2796, 2830, 2864, 2898, 2932, 2966, 3000, 3034, 3068,
	3102, 3136, 3171, 3205, 3239, 3273, 3307, 3341, 3375, 3409, 3443, 3477, 3511, 3546, 3580, 3614, 3648, 3682, 3716, 3750, 3784,
	3818, 3852, 3886, 3921, 3955, 3989, 4023, 4057
};

const u8 SCALE[49] = {
	2, 2, 1, 2, 2, 2, 1,	// ionian
	2, 1, 2, 2, 2, 1, 2,	// dorian
	1, 2, 2, 2, 1, 2, 2,	// phyrgian
	2, 2, 2, 1, 2, 2, 1,	// lydian
	2, 2, 1, 2, 2, 1, 2,	// mixolydian
 	2, 1, 2, 2, 1, 2, 2,	// aeolian
	1, 2, 2, 1, 2, 2, 2 	// locrian
};

typedef enum {
	mTr, mDur, mNote, mScale, mTrans, mScaleEdit, mPattern
} modes;

typedef enum {
	tTr, tAc, tOct, tDur, tNote, tTrans, tScale 
} time_params;

typedef enum {
	modNone, modLoop, modTime
} mod_modes;

#define NUM_PARAMS 7


typedef struct {
	u8 tr[16];
	u8 ac[16];
	u8 oct[16];
	u8 note[16];
	u8 dur[16];
	u8 sc[16];
	u8 trans[16];

	u8 dur_mul;

	u8 lstart[NUM_PARAMS];
	u8 lend[NUM_PARAMS];
	u8 llen[NUM_PARAMS];
	u8 lswap[NUM_PARAMS];
	time_params tmul[NUM_PARAMS];
	time_params tdiv[NUM_PARAMS];
} kria_pattern;

// TO 96
typedef struct {
	kria_pattern kp[2][16];
	u8 pscale[7];
} kria_set;

typedef const struct {
	u8 fresh;
	modes mode;
	u8 preset_select;
	u8 glyph[8][8];
	kria_set k[8];
	u8 scales[42][7];
} nvram_data_t;

kria_set k;

u8 preset_mode, preset_select, front_timer;
u8 glyph[8];

modes mode = mTr;
mod_modes mod_mode = modNone;
u8 p, p_next;
u8 ch;
u8 pos[2][NUM_PARAMS];
u8 trans_edit;
u8 pscale_edit;
u8 scales[42][7];

u8 key_alt, mod1, mod2;
u8 held_keys[32], key_count, key_times[256];
u8 keyfirst_pos, keysecond_pos;
s8 keycount_pos;
s8 loop_count;
u8 loop_edit;

u8 clock_phase;
u16 clock_time, clock_temp;

u32 basetime;
u32 calctimes[2][NUM_PARAMS];
u32 timeerrors[2][NUM_PARAMS];
u8 need0off;
u8 need1off;

u8 cur_scale[2][7];

u16 adc[4];

u8 tr[2];
u8 ac[2];
u8 oct[2];
u16 dur[2];
u8 note[2];
u8 sc[2];
u16 trans[2];
u16 cv0, cv1;

typedef void(*re_t)(void);
re_t re;


// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;




////////////////////////////////////////////////////////////////////////////////
// prototypes

static void refresh(void);
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

// static void ww_process_ii(uint8_t i, int d);

u8 flash_is_fresh(void);
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);



static void adjust_loop_start(u8 x, u8 m);
static void adjust_loop_end(u8 x, u8 m);

static void calc_scale(u8 c);

static void phase_reset0(void);
static void phase_reset1(void);



////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer  = { .next = NULL, .prev = NULL };

static softTimer_t note0offTimer = { .next = NULL, .prev = NULL };
static softTimer_t trans0Timer = { .next = NULL, .prev = NULL };
static softTimer_t sc0Timer = { .next = NULL, .prev = NULL };
static softTimer_t note0Timer = { .next = NULL, .prev = NULL };
static softTimer_t dur0Timer = { .next = NULL, .prev = NULL };
static softTimer_t oct0Timer = { .next = NULL, .prev = NULL };
static softTimer_t ac0Timer = { .next = NULL, .prev = NULL };
static softTimer_t tr0Timer = { .next = NULL, .prev = NULL };

static softTimer_t note1offTimer = { .next = NULL, .prev = NULL };
static softTimer_t trans1Timer = { .next = NULL, .prev = NULL };
static softTimer_t sc1Timer = { .next = NULL, .prev = NULL };
static softTimer_t note1Timer = { .next = NULL, .prev = NULL };
static softTimer_t dur1Timer = { .next = NULL, .prev = NULL };
static softTimer_t oct1Timer = { .next = NULL, .prev = NULL };
static softTimer_t ac1Timer = { .next = NULL, .prev = NULL };
static softTimer_t tr1Timer = { .next = NULL, .prev = NULL };



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

static void note0offTimer_callback(void* o) {
	if(need0off) {
		need0off = 0;
		timer_reset(&note0offTimer, 10000);
		gpio_clr_gpio_pin(B00);
		gpio_clr_gpio_pin(B02);
	}
}

static void trans0Timer_callback(void* o) {
	static u32 t;

	t = calctimes[0][tTrans] + timeerrors[0][tTrans];
    timeerrors[0][tTrans] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&trans0Timer, t);

	if(pos[0][tTrans] == k.kp[0][p].lend[tTrans])
		pos[0][tTrans] = k.kp[0][p].lstart[tTrans];
	else {
		pos[0][tTrans]++;
		if(pos[0][tTrans] > 15)
			pos[0][tTrans] = 0;
	}

	trans[0] = k.kp[0][p].trans[pos[0][tTrans]];

	monomeFrameDirty++;
}

static void sc0Timer_callback(void* o) {
	static u32 t;

	t = calctimes[0][tScale] + timeerrors[0][tScale];
    timeerrors[0][tScale] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&sc0Timer, t);

	if(pos[0][tScale] == k.kp[0][p].lend[tScale])
		pos[0][tScale] = k.kp[0][p].lstart[tScale];
	else {
		pos[0][tScale]++;
		if(pos[0][tScale] > 15)
			pos[0][tScale] = 0;
	}

	if(sc[0] != k.kp[0][p].sc[pos[0][tScale]]) {
		sc[0] = k.kp[0][p].sc[pos[0][tScale]];
		calc_scale(0);
	}

	monomeFrameDirty++;
}

static void note0Timer_callback(void* o) {
	static u32 t;

	t = calctimes[0][tNote] + timeerrors[0][tNote];
    timeerrors[0][tNote] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&note0Timer, t);

	if(pos[0][tNote] == k.kp[0][p].lend[tNote])
		pos[0][tNote] = k.kp[0][p].lstart[tNote];
	else {
		pos[0][tNote]++;
		if(pos[0][tNote] > 15)
			pos[0][tNote] = 0;
	}

	note[0] = k.kp[0][p].note[pos[0][tNote]];

	monomeFrameDirty++;
}

static void dur0Timer_callback(void* o) {
	static u32 t;

	t = calctimes[0][tDur] + timeerrors[0][tDur];
    timeerrors[0][tDur] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&dur0Timer, t);

	if(pos[0][tDur] == k.kp[0][p].lend[tDur])
		pos[0][tDur] = k.kp[0][p].lstart[tDur];
	else {
		pos[0][tDur]++;
		if(pos[0][tDur] > 15)
			pos[0][tDur] = 0;
	}

	dur[0] = (k.kp[0][p].dur[pos[0][tDur]]+1) * (k.kp[0][p].dur_mul<<2);

	monomeFrameDirty++;
}

static void oct0Timer_callback(void* o) {
	static u32 t;

	t = calctimes[0][tOct] + timeerrors[0][tOct];
    timeerrors[0][tOct] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&oct0Timer, t);

	if(pos[0][tOct] == k.kp[0][p].lend[tOct])
		pos[0][tOct] = k.kp[0][p].lstart[tOct];
	else {
		pos[0][tOct]++;
		if(pos[0][tOct] > 15)
			pos[0][tOct] = 0;
	}	

	oct[0] = k.kp[0][p].oct[pos[0][tOct]];

	monomeFrameDirty++;
}

static void ac0Timer_callback(void* o) {
	static u32 t;

	t = calctimes[0][tAc] + timeerrors[0][tAc];
    timeerrors[0][tAc] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&ac0Timer, t);

    // print_dbg("\r\nt ");
	// print_dbg_ulong(t >> 16);


	if(pos[0][tAc] == k.kp[0][p].lend[tAc])
		pos[0][tAc] = k.kp[0][p].lstart[tAc];
	else {
		pos[0][tAc]++;
		if(pos[0][tAc] > 15)
			pos[0][tAc] = 0;
	}

	ac[0] = k.kp[0][p].ac[pos[0][tAc]];

	monomeFrameDirty++;
}


static void tr0Timer_callback(void* o) {
	static u32 t;

	if(p_next != p) {
		p = p_next;
		phase_reset0();
		phase_reset1();
		return;
	}

	t = calctimes[0][tTr] + timeerrors[0][tTr];
    timeerrors[0][tTr] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&tr0Timer, t);

	if(pos[0][tTr] == k.kp[0][p].lend[tTr])
		pos[0][tTr] = k.kp[0][p].lstart[tTr];
	else {
		pos[0][tTr]++;
		if(pos[0][tTr] > 15)
			pos[0][tTr] = 0;
	}

	if(k.kp[0][p].tr[pos[0][tTr]]) {
		gpio_set_gpio_pin(B00);
		if(ac[0])
			gpio_set_gpio_pin(B02);

		cv0 = ET[cur_scale[0][note[0]] + (oct[0] * 12) + (trans[0] & 0xf) + ((trans[0] >> 4)*5)];

		need0off = 1;
		timer_reset(&note0offTimer, dur[0]);

		tr[0] = 1;
	}
	else
		tr[0] = 0;

	monomeFrameDirty++;

	// write to DAC
	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x31);	// update A
	spi_write(SPI,cv0>>4);
	spi_write(SPI,cv0<<4);
	spi_unselectChip(SPI,DAC_SPI);

	// spi_selectChip(SPI,DAC_SPI);
	// spi_write(SPI,0x38);	// update B
	// spi_write(SPI,cv1>>4);
	// spi_write(SPI,cv1<<4);
	// spi_unselectChip(SPI,DAC_SPI);
}


static void note1offTimer_callback(void* o) {
	if(need1off) {
		need1off = 0;
		timer_reset(&note1offTimer, 10000);
		gpio_clr_gpio_pin(B01);
		gpio_clr_gpio_pin(B03);
	}
}

static void trans1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tTrans] + timeerrors[1][tTrans];
    timeerrors[1][tTrans] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&trans1Timer, t);

	if(pos[1][tTrans] == k.kp[1][p].lend[tTrans])
		pos[1][tTrans] = k.kp[1][p].lstart[tTrans];
	else {
		pos[1][tTrans]++;
		if(pos[1][tTrans] > 15)
			pos[1][tTrans] = 0;
	}

	trans[1] = k.kp[1][p].trans[pos[1][tTrans]];

	monomeFrameDirty++;
}

static void sc1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tScale] + timeerrors[1][tScale];
    timeerrors[1][tScale] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&sc1Timer, t);

	if(pos[1][tScale] == k.kp[1][p].lend[tScale])
		pos[1][tScale] = k.kp[1][p].lstart[tScale];
	else {
		pos[1][tScale]++;
		if(pos[1][tScale] > 15)
			pos[1][tScale] = 0;
	}

	if(sc[1] != k.kp[1][p].sc[pos[1][tScale]]) {
		sc[1] = k.kp[1][p].sc[pos[1][tScale]];
		calc_scale(1);
	}

	monomeFrameDirty++;
}

static void note1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tNote] + timeerrors[1][tNote];
    timeerrors[1][tNote] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&note1Timer, t);

	if(pos[1][tNote] == k.kp[1][p].lend[tNote])
		pos[1][tNote] = k.kp[1][p].lstart[tNote];
	else {
		pos[1][tNote]++;
		if(pos[1][tNote] > 15)
			pos[1][tNote] = 0;
	}

	note[1] = k.kp[1][p].note[pos[1][tNote]];

	monomeFrameDirty++;
}

static void dur1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tDur] + timeerrors[1][tDur];
    timeerrors[1][tDur] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&dur1Timer, t);

	if(pos[1][tDur] == k.kp[1][p].lend[tDur])
		pos[1][tDur] = k.kp[1][p].lstart[tDur];
	else {
		pos[1][tDur]++;
		if(pos[1][tDur] > 15)
			pos[1][tDur] = 0;
	}

	dur[1] = (k.kp[1][p].dur[pos[1][tDur]]+1) * (k.kp[1][p].dur_mul<<2);

	monomeFrameDirty++;
}

static void oct1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tOct] + timeerrors[1][tOct];
    timeerrors[1][tOct] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&oct1Timer, t);

	if(pos[1][tOct] == k.kp[1][p].lend[tOct])
		pos[1][tOct] = k.kp[1][p].lstart[tOct];
	else {
		pos[1][tOct]++;
		if(pos[1][tOct] > 15)
			pos[1][tOct] = 0;
	}	

	oct[1] = k.kp[1][p].oct[pos[1][tOct]];

	monomeFrameDirty++;
}

static void ac1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tAc] + timeerrors[1][tAc];
    timeerrors[1][tAc] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&ac1Timer, t);

    // print_dbg("\r\nt ");
	// print_dbg_ulong(t >> 16);


	if(pos[1][tAc] == k.kp[1][p].lend[tAc])
		pos[1][tAc] = k.kp[1][p].lstart[tAc];
	else {
		pos[1][tAc]++;
		if(pos[1][tAc] > 15)
			pos[1][tAc] = 0;
	}

	ac[1] = k.kp[1][p].ac[pos[1][tAc]];

	monomeFrameDirty++;
}


static void tr1Timer_callback(void* o) {
	static u32 t;

	t = calctimes[1][tTr] + timeerrors[1][tTr];
    timeerrors[1][tTr] = t & 0xffff;
    t >>= 16;
    if(t<10) t = 10;

    timer_set(&tr1Timer, t);

	if(pos[1][tTr] == k.kp[1][p].lend[tTr])
		pos[1][tTr] = k.kp[1][p].lstart[tTr];
	else {
		pos[1][tTr]++;
		if(pos[1][tTr] > 15)
			pos[1][tTr] = 0;
	}

	if(k.kp[1][p].tr[pos[1][tTr]]) {
		gpio_set_gpio_pin(B01);
		if(ac[1])
			gpio_set_gpio_pin(B03);

		cv1 = ET[cur_scale[1][note[1]] + (oct[1] * 12) + (trans[1] & 0xf) + ((trans[1] >> 4)*5)];

		need1off = 1;
		timer_reset(&note1offTimer, dur[1]);

		tr[1] = 1;
	}
	else
		tr[1] = 0;

	monomeFrameDirty++;

	// write to DAC
	// spi_selectChip(SPI,DAC_SPI);
	// spi_write(SPI,0x31);	// update A
	// spi_write(SPI,cv0>>4);
	// spi_write(SPI,cv0<<4);
	// spi_unselectChip(SPI,DAC_SPI);

	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x38);	// update B
	spi_write(SPI,cv1>>4);
	spi_write(SPI,cv1<<4);
	spi_unselectChip(SPI,DAC_SPI);
}

static void phase_reset0() {
	timer_manual(&trans0Timer);
	timer_manual(&sc0Timer);
	timer_manual(&note0Timer);
	timer_manual(&dur0Timer);
	timer_manual(&oct0Timer);
	timer_manual(&ac0Timer);
	timer_manual(&tr0Timer);
	for(u8 i1=0;i1<NUM_PARAMS;i1++)
		pos[0][i1] = k.kp[0][p].lend[i1];
}

static void phase_reset1() {
	timer_manual(&trans1Timer);
	timer_manual(&sc1Timer);
	timer_manual(&note1Timer);
	timer_manual(&dur1Timer);
	timer_manual(&oct1Timer);
	timer_manual(&ac1Timer);
	timer_manual(&tr1Timer);
	for(u8 i1=0;i1<NUM_PARAMS;i1++)
		pos[1][i1] = k.kp[1][p].lend[1];
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application clock code

void clock(u8 phase) {
	if(phase) {
		gpio_set_gpio_pin(B10);
 	}
	else
		gpio_clr_gpio_pin(B10);

	// print_dbg("\r\nt ");
	// print_dbg_ulong(timer_ticks(&clockTimer));
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
	keycount_pos = 0;
	key_count = 0;
	// SIZE = monome_size_x();
	// LENGTH = SIZE - 1;
	// VARI = monome_is_vari();

	re = &refresh;

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
	// print_dbg("\r\n FRONT HOLD");

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
	i = 25000 / (i + 25);
	if(i != clock_temp) {
		// 1000ms - 24ms
		clock_time = i;
		basetime = i << 16;

		calctimes[0][tTrans] = (basetime * k.kp[0][p].tmul[tTrans]) / k.kp[0][p].tdiv[tTrans];
		calctimes[0][tScale] = (basetime * k.kp[0][p].tmul[tScale]) / k.kp[0][p].tdiv[tScale];
		calctimes[0][tNote] = (basetime * k.kp[0][p].tmul[tNote]) / k.kp[0][p].tdiv[tNote];
		calctimes[0][tDur] = (basetime * k.kp[0][p].tmul[tDur]) / k.kp[0][p].tdiv[tDur];
		calctimes[0][tOct] = (basetime * k.kp[0][p].tmul[tOct]) / k.kp[0][p].tdiv[tOct];
		calctimes[0][tAc] = (basetime * k.kp[0][p].tmul[tAc]) / k.kp[0][p].tdiv[tAc];
		calctimes[0][tTr] = (basetime * k.kp[0][p].tmul[tTr]) / k.kp[0][p].tdiv[tTr];

		calctimes[1][tTrans] = (basetime * k.kp[1][p].tmul[tTrans]) / k.kp[1][p].tdiv[tTrans];
		calctimes[1][tScale] = (basetime * k.kp[1][p].tmul[tScale]) / k.kp[1][p].tdiv[tScale];
		calctimes[1][tNote] = (basetime * k.kp[1][p].tmul[tNote]) / k.kp[1][p].tdiv[tNote];
		calctimes[1][tDur] = (basetime * k.kp[1][p].tmul[tDur]) / k.kp[1][p].tdiv[tDur];
		calctimes[1][tOct] = (basetime * k.kp[1][p].tmul[tOct]) / k.kp[1][p].tdiv[tOct];
		calctimes[1][tAc] = (basetime * k.kp[1][p].tmul[tAc]) / k.kp[1][p].tdiv[tAc];
		calctimes[1][tTr] = (basetime * k.kp[1][p].tmul[tTr]) / k.kp[1][p].tdiv[tTr];
		// print_dbg("\r\nnew clock (ms): ");
		// print_dbg_ulong(calctimes[tOct]);

		timer_set(&clockTimer, clock_time);
	}
	clock_temp = i;

	/*
	// PARAM POT INPUT
	if(param_accept && edit_prob) {
		*param_dest8 = adc[1] >> 4; // scale to 0-255;
		// print_dbg("\r\nnew prob: ");
		// print_dbg_ulong(*param_dest8);
		// print_dbg("\t" );
		// print_dbg_ulong(adc[1]);
	}
	else if(param_accept) {
		if(quantize_in)
			*param_dest = (adc[1] / 34) * 34;
		else
			*param_dest = adc[1];
		monomeFrameDirty++;
	}
	else if(key_meta) {
		i = adc[1]>>6;
		if(i > 58)
			i = 58;
		if(i != scroll_pos) {
			scroll_pos = i;
			monomeFrameDirty++;
			// print_dbg("\r scroll pos: ");
			// print_dbg_ulong(scroll_pos);
		}
	}
	*/
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
			if(preset_mode == 0) {
				/*
				// preset copy
				if(held_keys[i1] / 16 == 2) {
					x = held_keys[i1] % 16;
					for(n1=0;n1<16;n1++) {
						w.wp[x].steps[n1] = w.wp[pattern].steps[n1];
						w.wp[x].step_probs[n1] = w.wp[pattern].step_probs[n1];
						w.wp[x].cv_values[n1] = w.wp[pattern].cv_values[n1];
						w.wp[x].cv_steps[0][n1] = w.wp[pattern].cv_steps[0][n1];
						w.wp[x].cv_curves[0][n1] = w.wp[pattern].cv_curves[0][n1];
						w.wp[x].cv_probs[0][n1] = w.wp[pattern].cv_probs[0][n1];
						w.wp[x].cv_steps[1][n1] = w.wp[pattern].cv_steps[1][n1];
						w.wp[x].cv_curves[1][n1] = w.wp[pattern].cv_curves[1][n1];
						w.wp[x].cv_probs[1][n1] = w.wp[pattern].cv_probs[1][n1];
					}

					w.wp[x].cv_mode[0] = w.wp[pattern].cv_mode[0];
					w.wp[x].cv_mode[1] = w.wp[pattern].cv_mode[1];

					w.wp[x].loop_start = w.wp[pattern].loop_start;
					w.wp[x].loop_end = w.wp[pattern].loop_end;
					w.wp[x].loop_len = w.wp[pattern].loop_len;
					w.wp[x].loop_dir = w.wp[pattern].loop_dir;

					w.wp[x].tr_mode = w.wp[pattern].tr_mode;
					w.wp[x].step_mode = w.wp[pattern].step_mode;

					pattern = x;
					next_pattern = x;
					
					monomeFrameDirty++;

					// print_dbg("\r\n saved pattern: ");
					// print_dbg_ulong(x);
				}
				*/
			}
			else if(preset_mode == 1) {
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
			// PRESET MODE FAST PRESS DETECT
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
			monomeFrameDirty++;	
		}
	}
	// NOT PRESET
	else {
		// bottom row
		if(y == 7) {
			if(x == 15)
				key_alt = z;
			else if(x == 3) {
				if(z) mode = mTr;
			}
			else if(x == 4) {
				if(z) mode = mDur;
			}
			else if(x == 5) {
				if(z) mode = mNote;
			}
			else if(x == 6) {
				if(z) mode = mScale;
			}
			else if(x == 7) {
				if(z) mode = mTrans;
			}
			else if(x == 9) {
				if(z) {
					mod_mode = modLoop;
					loop_count = 0;
				}
				else
					mod_mode = modNone;
			}
			else if(x == 10) {
				if(z)
					mod_mode = modTime;
				else
					mod_mode = modNone;
			}
			else if(x == 0) {
				if(z) {
					ch = 0;
					if(key_alt)
						phase_reset0();
				}
			}
			else if(x == 1) {
				if(z) {
					ch = 1;
					if(key_alt)
						phase_reset1();
				}
			}
			else if(x == 12) {
				if(z) mode = mScaleEdit;
			}
			else if(x == 13) {
				if(z) mode = mPattern;
			}

			monomeFrameDirty++;
		}


		// toggle steps 
		else if(mode == mTr) {
			if(mod_mode == modNone) {
				if(z) {
					if(y==0) 
						k.kp[ch][p].tr[x] ^= 1;
					else if(y==1)
						k.kp[ch][p].ac[x] ^= 1;
					else if(y>1) 
						k.kp[ch][p].oct[x] = 6-y;
					monomeFrameDirty++;
				}
			}
			else if(mod_mode == modLoop) {
				if(z) {
					loop_count++;

					if(key_alt) {
						if(y==0) {
							adjust_loop_start(x, tTr);
							adjust_loop_end(x, tTr);
						}
						else if(y==1) {
							adjust_loop_start(x, tAc);
							adjust_loop_end(x, tAc);
						}
						else if(y>1) {
							adjust_loop_start(x, tOct);
							adjust_loop_end(x, tOct);
						}
					}
					else if(loop_count == 1) {
						loop_edit = y;
						if(y==0)
							adjust_loop_start(x, tTr);
						else if(y==1)
							adjust_loop_start(x, tAc);
						else if(y>1)
							adjust_loop_start(x, tOct);
					}
					else if(y == loop_edit) {
						if(y==0)
							adjust_loop_end(x, tTr);
						else if(y==1)
							adjust_loop_end(x, tAc);
						else if(y>1)
							adjust_loop_end(x, tOct);
					}
					monomeFrameDirty++;
				}
				else {
					loop_count--;
				}
			}
			else if(mod_mode == modTime) {
				if(z) {
					if(y == 0) {
						k.kp[ch][p].tmul[tTr] = x+1;
						calctimes[ch][tTr] = (basetime * k.kp[ch][p].tmul[tTr]) / k.kp[ch][p].tdiv[tTr];
					}
					else if(y == 1) {
						k.kp[ch][p].tdiv[tTr] = x+1;
						calctimes[ch][tTr] = (basetime * k.kp[ch][p].tmul[tTr]) / k.kp[ch][p].tdiv[tTr];
					}
					else if(y == 2) {
						k.kp[ch][p].tmul[tAc] = x+1;
						calctimes[ch][tAc] = (basetime * k.kp[ch][p].tmul[tAc]) / k.kp[ch][p].tdiv[tAc];
					}
					else if(y == 3) {
						k.kp[ch][p].tdiv[tAc] = x+1;
						calctimes[ch][tAc] = (basetime * k.kp[ch][p].tmul[tAc]) / k.kp[ch][p].tdiv[tAc];
					}
					else if(y == 4) {
						k.kp[ch][p].tmul[tOct] = x+1;
						calctimes[ch][tOct] = (basetime * k.kp[ch][p].tmul[tOct]) / k.kp[ch][p].tdiv[tOct];
					}
					else if(y == 5) {
						k.kp[ch][p].tdiv[tOct] = x+1;
						calctimes[ch][tOct] = (basetime * k.kp[ch][p].tmul[tOct]) / k.kp[ch][p].tdiv[tOct];
					}
				}
			}
		}
		else if(mode == mDur) {
			if(z) {
				if(mod_mode != modTime) {
					if(y==0)
						 k.kp[ch][p].dur_mul = x+1;
					else {
						if(mod_mode == modNone)
							k.kp[ch][p].dur[x] = y-1;
						else if(mod_mode == modLoop) {
							loop_count++;
							if(key_alt) {
								adjust_loop_start(x, tDur);
								adjust_loop_end(x, tDur);
							}
							else if(loop_count == 1) 
								adjust_loop_start(x, tDur);
							else adjust_loop_end(x, tDur);
						}
					}
				}
				else {
					if(y == 0) {
						k.kp[ch][p].tmul[tDur] = x+1;
						calctimes[ch][tDur] = (basetime * k.kp[ch][p].tmul[tDur]) / k.kp[ch][p].tdiv[tDur];
					}
					else if(y == 1) {
						k.kp[ch][p].tdiv[tDur] = x+1;
						calctimes[ch][tDur] = (basetime * k.kp[ch][p].tmul[tDur]) / k.kp[ch][p].tdiv[tDur];
					}
				}

				monomeFrameDirty++;
			}
			else if(mod_mode == modLoop) loop_count--;
		}
		else if(mode == mNote) {
			if(z) {
				if(mod_mode != modTime) {
					if(mod_mode == modNone)
						k.kp[ch][p].note[x] = 6-y;
					else if(mod_mode == modLoop) {
						loop_count++;
						if(key_alt) {
							adjust_loop_start(x, tNote);
							adjust_loop_end(x, tNote);
						}
						else if(loop_count == 1)
							adjust_loop_start(x, tNote);
						else adjust_loop_end(x, tNote);
					}
				}
				else {
					if(y == 0) {
						k.kp[ch][p].tmul[tNote] = x+1;
						calctimes[ch][tNote] = (basetime * k.kp[ch][p].tmul[tNote]) / k.kp[ch][p].tdiv[tNote];
					}
					else if(y == 1) {
						k.kp[ch][p].tdiv[tNote] = x+1;
						calctimes[ch][tNote] = (basetime * k.kp[ch][p].tmul[tNote]) / k.kp[ch][p].tdiv[tNote];
					}
				}

				monomeFrameDirty++;
			}
			else if(mod_mode == modLoop) loop_count--;
		}
		else if(mode == mScale) {
			if(z) {
				if(mod_mode != modTime) {
					if(mod_mode == modNone)
						k.kp[ch][p].sc[x] = y;
					else if(mod_mode == modLoop) {
						loop_count++;
						if(key_alt) {
							adjust_loop_start(x, tScale);
							adjust_loop_end(x, tScale);
						}
						else if(loop_count == 1) 
							adjust_loop_start(x, tScale);
						else adjust_loop_end(x, tScale);
					}
				}
				else {
					if(y == 0) {
						k.kp[ch][p].tmul[tScale] = x+1;
						calctimes[ch][tScale] = (basetime * k.kp[ch][p].tmul[tScale]) / k.kp[ch][p].tdiv[tScale];
					}
					else if(y == 1) {
						k.kp[ch][p].tdiv[tScale] = x+1;
						calctimes[ch][tScale] = (basetime * k.kp[ch][p].tmul[tScale]) / k.kp[ch][p].tdiv[tScale];
					}
				}
				
				monomeFrameDirty++;
			}
			else if(mod_mode == modLoop) loop_count--;
		}
		else if(mode == mTrans) {
			if(z) {
				if(mod_mode != modTime) {
					if(y == 0) {
						if(mod_mode == modNone)
							trans_edit = x;
						else if(mod_mode == modLoop) {
							loop_count++;
							if(key_alt) {
								adjust_loop_start(x, tTrans);
								adjust_loop_end(x, tTrans);
							}
							else if(loop_count == 1) 
								adjust_loop_start(x, tTrans);
							else adjust_loop_end(x, tTrans);
						}
					}
					else
						k.kp[ch][p].trans[trans_edit] = x + ((6-y)<<4);
				}
				else {
					if(y == 0) {
						k.kp[ch][p].tmul[tTrans] = x+1;
						calctimes[ch][tTrans] = (basetime * k.kp[ch][p].tmul[tTrans]) / k.kp[ch][p].tdiv[tTrans];
					}
					else if(y == 1) {
						k.kp[ch][p].tdiv[tTrans] = x+1;
						calctimes[ch][tTrans] = (basetime * k.kp[ch][p].tmul[tTrans]) / k.kp[ch][p].tdiv[tTrans];
					}
				}
				monomeFrameDirty++;
			}
			else if(mod_mode == modLoop) loop_count--;
		}
		else if(mode == mScaleEdit) {
			if(z) {
				if(x==0) {
					pscale_edit = y;
				}
				else if(y == 6 && x < 8) {
					for(i1=0;i1<6;i1++)
						scales[k.pscale[pscale_edit]][i1+1] = SCALE[(x-1)*7+i1];
					scales[k.pscale[pscale_edit]][0] = 0;

					if(sc[0] == pscale_edit)
						calc_scale(0);
					if(sc[1] == pscale_edit)
						calc_scale(1);
				}
				else if(x > 0 && x < 8) {
					if(key_alt) {
						for(i1=0;i1<7;i1++)
							scales[k.pscale[x-1 + y*7]][i1] = scales[k.pscale[pscale_edit]][i1];
					}

					k.pscale[pscale_edit] = x-1 + y*7;

					if(sc[0] == pscale_edit)
						calc_scale(0);
					if(sc[1] == pscale_edit)
						calc_scale(1);
					
				}
				else if(x>7) {
					if(key_alt) {
						if(y!=0) {
							s8 diff, change;
							diff = (x-8) - scales[k.pscale[pscale_edit]][6-y];
							change = scales[k.pscale[pscale_edit]][6-y+1] - diff;
							if(change<0) change = 0;
							if(change>7) change = 7;
							scales[k.pscale[pscale_edit]][6-y+1] = change;
						}

						scales[k.pscale[pscale_edit]][6-y] = x-8;
					}
					else scales[k.pscale[pscale_edit]][6-y] = x-8;

					if(sc[0] == pscale_edit)
						calc_scale(0);
					if(sc[1] == pscale_edit)
						calc_scale(1);
				}

				monomeFrameDirty++;
			}
			else if(mod_mode == modLoop) loop_count--;
		}
		else if(mode==mPattern) {
			if(z && y==0) {
				if(key_alt) {
					p_next = x;

					for(i1=0;i1<2;i1++) {
						for(u8 i2=0;i2<16;i2++) {
							k.kp[i1][p_next].tr[i2] = k.kp[i1][p].tr[i2];
							k.kp[i1][p_next].ac[i2] = k.kp[i1][p].ac[i2];
							k.kp[i1][p_next].oct[i2] = k.kp[i1][p].oct[i2];
							k.kp[i1][p_next].dur[i2] = k.kp[i1][p].dur[i2];
							k.kp[i1][p_next].note[i2] = k.kp[i1][p].note[i2];
							k.kp[i1][p_next].trans[i2] = k.kp[i1][p].trans[i2];
							k.kp[i1][p_next].sc[i2] = k.kp[i1][p].sc[i2];
						}

						k.kp[i1][p_next].dur_mul = k.kp[i1][p].dur_mul;

						for(u8 i2=0;i2<NUM_PARAMS;i2++) {
							k.kp[i1][p_next].lstart[i2] = k.kp[i1][p].lstart[i2];
							k.kp[i1][p_next].lend[i2] = k.kp[i1][p].lend[i2];
							k.kp[i1][p_next].llen[i2] = k.kp[i1][p].llen[i2];
							k.kp[i1][p_next].lswap[i2] = k.kp[i1][p].lswap[i2];
							k.kp[i1][p_next].tmul[i2] = k.kp[i1][p].tmul[i2];
							k.kp[i1][p_next].tdiv[i2] = k.kp[i1][p].tdiv[i2];
						}
					}

					p = p_next;
				}
				else 
					p_next = x;

				monomeFrameDirty++;
			}
		}
	}
}


static void adjust_loop_start(u8 x, u8 m) {
	s8 temp;

	temp = pos[ch][m] - k.kp[ch][p].lstart[m] + x;
	if(temp < 0) temp += 16;
	else if(temp > 15) temp -= 16;
	pos[ch][m] = temp;

	k.kp[ch][p].lstart[m] = x;
	temp = x + k.kp[ch][p].llen[m]-1;
	if(temp > 15) {
		k.kp[ch][p].lend[m] = temp - 16;
		k.kp[ch][p].lswap[m] = 1;
	}
	else {
		k.kp[ch][p].lend[m] = temp;
		k.kp[ch][p].lswap[m] = 0;
	}
}

static void adjust_loop_end(u8 x, u8 m) {
	s8 temp;

	k.kp[ch][p].lend[m] = x;
	temp = k.kp[ch][p].lend[m] - k.kp[ch][p].lstart[m];
	if(temp < 0) {
		k.kp[ch][p].llen[m] = temp + 17;
		k.kp[ch][p].lswap[m] = 1;
	}
	else {
		k.kp[ch][p].llen[m] = temp+1;
		k.kp[ch][p].lswap[m] = 0;
	}

	temp = pos[ch][m];
	if(k.kp[ch][p].lswap[m]) {
		if(temp < k.kp[ch][p].lstart[m] && temp > k.kp[ch][p].lend[m])
			pos[ch][m] = k.kp[ch][p].lstart[m];
	}
	else {
		if(temp < k.kp[ch][p].lstart[m] || temp > k.kp[ch][p].lend[m])
			pos[ch][m] = k.kp[ch][p].lstart[m];
	}
}

static void calc_scale(u8 c) {
	static u8 i1;
	// cur_scale[c][0] = SCALE[sc[c]*7];
	cur_scale[c][0] = scales[k.pscale[sc[c]]][0];

	for(i1=1;i1<7;i1++)
		// cur_scale[c][i1] = cur_scale[c][i1-1] + SCALE[sc[c]*7 + i1];
		cur_scale[c][i1] = cur_scale[c][i1-1] + scales[k.pscale[sc[c]]][i1];
	// print_dbg("\r\nscale: ");
	// for(i1=0;i1<7;i1++) {
	// 	print_dbg_ulong(cur_scale[c][i1]);
	// 	print_dbg(" ");
	// }
}

////////////////////////////////////////////////////////////////////////////////
// application grid redraw
static void refresh() {
	u8 i1,i2;

	// bottom strip

	monomeLedBuffer[112+(ch==0)] = L0;
	monomeLedBuffer[112+ch] = L2;

	for(i1=0;i1<5;i1++)
		monomeLedBuffer[115+i1] = L0;
	if(mode <= mTrans) 
		monomeLedBuffer[115 + mode] = L2;

	monomeLedBuffer[121] = L0 + ((mod_mode == modLoop) << 2);
	monomeLedBuffer[122] = L0 + ((mod_mode == modTime) << 2);

	monomeLedBuffer[124] = L0 + ((mode == mScaleEdit) << 2);
	monomeLedBuffer[125] = L0 + ((mode == mPattern) << 2);

	if(key_alt) monomeLedBuffer[127] = L2;
	else monomeLedBuffer[127] = 0;

	// modes

	if(mode == mTr) {
		if(mod_mode != modTime) {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;
			
			if(mod_mode == modLoop) {
				if(k.kp[ch][p].lswap[tTr]) {
					for(i1=0;i1<k.kp[ch][p].llen[tTr];i1++)
						monomeLedBuffer[(i1+k.kp[ch][p].lstart[tTr])%16] = L0;
				}
				else {
					for(i1=k.kp[ch][p].lstart[tTr];i1<=k.kp[ch][p].lend[tTr];i1++)
						monomeLedBuffer[i1] = L0;
				}
				if(k.kp[ch][p].lswap[tAc]) {
					for(i1=0;i1<k.kp[ch][p].llen[tAc];i1++)
						monomeLedBuffer[16 + (i1 + k.kp[ch][p].lstart[tAc]) % 16] = L0;
				}
				else {
					for(i1=k.kp[ch][p].lstart[tAc];i1<=k.kp[ch][p].lend[tAc];i1++)
						monomeLedBuffer[16 + i1] = L0;
				}
			}
			
			for(i1=0;i1<16;i1++) {
				if(k.kp[ch][p].tr[i1])
					monomeLedBuffer[i1] = L1;

				if(k.kp[ch][p].ac[i1])
					monomeLedBuffer[16+i1] = L1;
				
				for(i2=0;i2<=k.kp[ch][p].oct[i1];i2++)
					monomeLedBuffer[96-16*i2+i1] = L0;

				if(i1 == pos[ch][tTr])
					monomeLedBuffer[i1] += 4;
				if(i1 == pos[ch][tAc])
					monomeLedBuffer[16+i1] += 4;
				if(i1 == pos[ch][tOct])
					monomeLedBuffer[96 - k.kp[ch][p].oct[i1]*16 + i1] += 4;
			}

			if(k.kp[ch][p].lswap[tTr]) {
				for(i1=0;i1<16;i1++)
					if(monomeLedBuffer[i1])
						if((i1 < k.kp[ch][p].lstart[tTr]) && (i1 > k.kp[ch][p].lend[tTr]))
							monomeLedBuffer[i1] -= 6;
			}
			else {
				for(i1=0;i1<16;i1++)
					if(monomeLedBuffer[i1])
						if((i1 < k.kp[ch][p].lstart[tTr]) || (i1 > k.kp[ch][p].lend[tTr]))
							monomeLedBuffer[i1] -= 6;
			}

			if(k.kp[ch][p].lswap[tAc]) {
				for(i1=0;i1<16;i1++)
					if(monomeLedBuffer[16+i1])
						if((i1 < k.kp[ch][p].lstart[tAc]) && (i1 > k.kp[ch][p].lend[tAc]))
							monomeLedBuffer[16+i1] -= 6;
			}
			else {
				for(i1=0;i1<16;i1++)
					if(monomeLedBuffer[16+i1])
						if((i1 < k.kp[ch][p].lstart[tAc]) || (i1 > k.kp[ch][p].lend[tAc]))
							monomeLedBuffer[16+i1] -= 6;
			}

			if(k.kp[ch][p].lswap[tOct]) {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tOct]) && (i1 > k.kp[ch][p].lend[tOct]))
						for(i2=0;i2<=k.kp[ch][p].oct[i1];i2++)
							monomeLedBuffer[96-16*i2+i1] -= 2;
			}
			else {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tOct]) || (i1 > k.kp[ch][p].lend[tOct]))
						for(i2=0;i2<=k.kp[ch][p].oct[i1];i2++)
							monomeLedBuffer[96-16*i2+i1] -= 2;
			}
		}
		else {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			monomeLedBuffer[pos[ch][tTr]] = L0;
			monomeLedBuffer[pos[ch][tTr]+16] = L0;
			monomeLedBuffer[k.kp[ch][p].tmul[tTr] - 1] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tTr] - 1 + 16] = L1;

			monomeLedBuffer[pos[ch][tAc]+32] = L0;
			monomeLedBuffer[pos[ch][tAc]+48] = L0;
			monomeLedBuffer[k.kp[ch][p].tmul[tAc] - 1 + 32] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tAc] - 1 + 48] = L1;

			monomeLedBuffer[pos[ch][tOct]+64] = L0;
			monomeLedBuffer[pos[ch][tOct]+80] = L0;
			monomeLedBuffer[k.kp[ch][p].tmul[tOct] - 1 + 64] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tOct] - 1 + 80] = L1;
		}
	}
	else if(mode == mDur) {
		if(mod_mode != modTime) {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			monomeLedBuffer[k.kp[ch][p].dur_mul - 1] = L1;

			for(i1=0;i1<16;i1++) {
				for(i2=0;i2<=k.kp[ch][p].dur[i1];i2++)
					monomeLedBuffer[16+16*i2+i1] = L0;

				if(i1 == pos[ch][tDur])
					monomeLedBuffer[16+i1+16*k.kp[ch][p].dur[i1]] += 4;
			}

			if(k.kp[ch][p].lswap[tDur]) {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tDur]) && (i1 > k.kp[ch][p].lend[tDur]))
						for(i2=0;i2<=k.kp[ch][p].dur[i1];i2++)
							monomeLedBuffer[16+16*i2+i1] -= 2;
			}
			else {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tDur]) || (i1 > k.kp[ch][p].lend[tDur]))
						for(i2=0;i2<=k.kp[ch][p].dur[i1];i2++)
							monomeLedBuffer[16+16*i2+i1] -= 2;
			}
		}
		else {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			monomeLedBuffer[pos[ch][tDur]] = L0;
			monomeLedBuffer[pos[ch][tDur]+16] = L0;

			monomeLedBuffer[k.kp[ch][p].tmul[tDur] - 1] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tDur] - 1 + 16] = L1;
		}
	}
	else if(mode == mNote) {
		if(mod_mode != modTime) {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;
			for(i1=0;i1<16;i1++)
				monomeLedBuffer[i1+(6-k.kp[ch][p].note[i1])*16] = L1;
			monomeLedBuffer[pos[ch][tNote] + (6-k.kp[ch][p].note[pos[ch][tNote]])*16] += 4;

			if(k.kp[ch][p].lswap[tNote]) {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tNote]) && (i1 > k.kp[ch][p].lend[tNote]))
						monomeLedBuffer[i1+(6-k.kp[ch][p].note[i1])*16] -= 4;
			}
			else {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tNote]) || (i1 > k.kp[ch][p].lend[tNote]))
						monomeLedBuffer[i1+(6-k.kp[ch][p].note[i1])*16] -= 4;
			}
		}
		else {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			monomeLedBuffer[pos[ch][tNote]] = L0;
			monomeLedBuffer[pos[ch][tNote]+16] = L0;

			monomeLedBuffer[k.kp[ch][p].tmul[tNote] - 1] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tNote] - 1 + 16] = L1;
		}
	}
	else if(mode == mScale) {
		if(mod_mode != modTime) {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			for(i1=0;i1<16;i1++) {
				for(i2=0;i2<=k.kp[ch][p].sc[i1];i2++)
							monomeLedBuffer[16*i2+i1] = L0;

				monomeLedBuffer[i1+16*k.kp[ch][p].sc[i1]] = L1;

				if(i1 == pos[ch][tScale])
					monomeLedBuffer[i1+16*k.kp[ch][p].sc[i1]] += 4;
			}

			if(k.kp[ch][p].lswap[tScale]) {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tScale]) && (i1 > k.kp[ch][p].lend[tScale]))
						for(i2=0;i2<=k.kp[ch][p].sc[i1];i2++)
							monomeLedBuffer[16*i2+i1] -= 4;
			}
			else {
				for(i1=0;i1<16;i1++)
					if((i1 < k.kp[ch][p].lstart[tScale]) || (i1 > k.kp[ch][p].lend[tScale]))
						for(i2=0;i2<=k.kp[ch][p].sc[i1];i2++)
							monomeLedBuffer[16*i2+i1] -= 4;;
			}
		}
		else {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			monomeLedBuffer[pos[ch][tScale]] = L0;
			monomeLedBuffer[pos[ch][tScale]+16] = L0;

			monomeLedBuffer[k.kp[ch][p].tmul[tScale] - 1] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tScale] - 1 + 16] = L1;
		}
	}
	else if(mode == mTrans) {
		if(mod_mode != modTime) {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			if(mod_mode == modLoop) {
				if(k.kp[ch][p].lswap[tTrans]) {
					for(i1=0;i1<k.kp[ch][p].llen[tTrans];i1++)
						monomeLedBuffer[(i1+k.kp[ch][p].lstart[tTrans])%16] = L0;
				}
				else {
					for(i1=k.kp[ch][p].lstart[tTrans];i1<=k.kp[ch][p].lend[tTrans];i1++)
						monomeLedBuffer[i1] = L0;
				}
			}

			monomeLedBuffer[trans_edit] = L1;
			monomeLedBuffer[pos[ch][tTrans]] += 4;

			for(i1=0;i1<16;i1++) {
				monomeLedBuffer[(k.kp[ch][p].trans[i1] & 0xf) + 16*(6-(k.kp[ch][p].trans[i1] >> 4))] = L0;
			}

			monomeLedBuffer[(k.kp[ch][p].trans[pos[ch][tTrans]] & 0xf) + 16*(6-(k.kp[ch][p].trans[pos[ch][tTrans]] >> 4))] += 4;
			monomeLedBuffer[(k.kp[ch][p].trans[trans_edit] & 0xf) + 16*(6-(k.kp[ch][p].trans[trans_edit] >> 4))] = L2;
		}
		else {
			for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;

			monomeLedBuffer[pos[ch][tTrans]] = L0;
			monomeLedBuffer[pos[ch][tTrans]+16] = L0;

			monomeLedBuffer[k.kp[ch][p].tmul[tTrans] - 1] = L2;
			monomeLedBuffer[k.kp[ch][p].tdiv[tTrans] - 1 + 16] = L1;
		}
	}
	else if(mode==mScaleEdit) {
		monomeLedBuffer[112] = L1;
		monomeLedBuffer[113] = L1;

		for(i1=0;i1<112;i1++)
				monomeLedBuffer[i1] = 0;
		for(i1=0;i1<7;i1++) {
			monomeLedBuffer[i1*16] = 4;
			monomeLedBuffer[97+i1] = L0;
			monomeLedBuffer[8+16*i1] = L0;
		}
		monomeLedBuffer[k.kp[0][p].sc[pos[0][tScale]] * 16] = L1;
		monomeLedBuffer[k.kp[1][p].sc[pos[1][tScale]] * 16] = L1;
		monomeLedBuffer[pscale_edit * 16] = L2;

		monomeLedBuffer[(k.pscale[pscale_edit] / 7) * 16 + 1 + (k.pscale[pscale_edit] % 7)] = L2;

		for(i1=0;i1<7;i1++)
			monomeLedBuffer[scales[k.pscale[pscale_edit]][i1] + 8 + (6-i1)*16] = L1;

		if(sc[0] == pscale_edit && tr[0]) {
			i1 = k.kp[0][p].note[pos[0][tNote]];
			monomeLedBuffer[scales[k.pscale[pscale_edit]][i1] + 8 + (6-i1)*16] = L2;
		}

		if(sc[1] == pscale_edit && tr[1]) {
			i1 = k.kp[1][p].note[pos[1][tNote]];
			monomeLedBuffer[scales[k.pscale[pscale_edit]][i1] + 8 + (6-i1)*16] = L2;
		}
	}
	else if(mode==mPattern) {
		monomeLedBuffer[112] = L1;
		monomeLedBuffer[113] = L1;
		
		for(i1=0;i1<112;i1++)
			monomeLedBuffer[i1] = 0;
		for(i1=0;i1<16;i1++)
			monomeLedBuffer[i1] = L0;
		if(p_next != p)
			monomeLedBuffer[p_next] = L1;
		monomeLedBuffer[p] = L2;
	}

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
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



	
static void ww_process_ii(uint8_t i, int d) {
	switch(i) {
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
	flashc_memcpy((void *)&flashy.k[preset_select], &k, sizeof(k), true);
	flashc_memcpy((void *)&flashy.scales, &scales, sizeof(scales), true);
	flashc_memcpy((void *)&flashy.glyph[preset_select], &glyph, sizeof(glyph), true);
	flashc_memset8((void*)&(flashy.preset_select), preset_select, 1, true);
	// flashc_memset32((void*)&(flashy.edit_mode), edit_mode, 4, true);
}

void flash_read(void) {
	u8 i1, i2, c;

	print_dbg("\r\n read preset ");
	print_dbg_ulong(preset_select);

	for(c=0;c<2;c++) {
		for(i1=0;i1<16;i1++) {
			for(i2=0;i2<16;i2++) {
				k.kp[c][i1].tr[i2] = flashy.k[preset_select].kp[c][i1].tr[i2];
				k.kp[c][i1].ac[i2] = flashy.k[preset_select].kp[c][i1].ac[i2];
				k.kp[c][i1].oct[i2] = flashy.k[preset_select].kp[c][i1].oct[i2];
				k.kp[c][i1].dur[i2] = flashy.k[preset_select].kp[c][i1].dur[i2];
				k.kp[c][i1].note[i2] = flashy.k[preset_select].kp[c][i1].note[i2];
				k.kp[c][i1].sc[i2] = flashy.k[preset_select].kp[c][i1].sc[i2];
				k.kp[c][i1].trans[i2] = flashy.k[preset_select].kp[c][i1].trans[i2];
			}
			k.kp[c][i1].dur_mul = flashy.k[preset_select].kp[c][i1].dur_mul;
			for(i2=0;i2<NUM_PARAMS;i2++) {
				k.kp[c][i1].lstart[i2] = flashy.k[preset_select].kp[c][i1].lstart[i2];
				k.kp[c][i1].lend[i2] = flashy.k[preset_select].kp[c][i1].lend[i2];
				k.kp[c][i1].llen[i2] = flashy.k[preset_select].kp[c][i1].llen[i2];
				k.kp[c][i1].lswap[i2] = flashy.k[preset_select].kp[c][i1].lswap[i2];
				k.kp[c][i1].tmul[i2] = flashy.k[preset_select].kp[c][i1].tmul[i2];
				k.kp[c][i1].tdiv[i2] = flashy.k[preset_select].kp[c][i1].tdiv[i2];
			}
			// k.kp[c][i1].sync = flashy.k[preset_select].kp[c][i1].sync;
		}
	}

	for(i1=0;i1<7;i1++)
		k.pscale[i1] = flashy.k[preset_select].pscale[i1];

	for(i1=0;i1<42;i1++)
		for(i2=0;i2<7;i2++)
			scales[i1][i2] = flashy.scales[i1][i2];

	calc_scale(0);
	calc_scale(1);
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// main

int main(void)
{
	u8 i1,i2,c;

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

	init_i2c_slave(0x10);

	gpio_clr_gpio_pin(B00);
	gpio_clr_gpio_pin(B01);
	gpio_clr_gpio_pin(B02);
	gpio_clr_gpio_pin(B03);


	print_dbg("\r\n\n// kria //////////////////////////////// ");
	print_dbg_ulong(sizeof(flashy));

	print_dbg(" ");
	print_dbg_ulong(sizeof(k));

	print_dbg(" ");
	print_dbg_ulong(sizeof(glyph));

	if(flash_is_fresh()) {
		print_dbg("\r\nfirst run.");
		flash_unfresh();
		// flashc_memset8((void*)&(flashy.edit_mode), mTrig, 4, true);
		flashc_memset32((void*)&(flashy.preset_select), 0, 4, true);


		// clear out some reasonable defaults
		for(c=0;c<2;c++) {
			for(i1=0;i1<16;i1++) {
				for(i2=0;i2<16;i2++) {
					k.kp[c][i1].tr[i2] = 0;
					k.kp[c][i1].ac[i2] = 0;
					k.kp[c][i1].oct[i2] = 0;
					k.kp[c][i1].dur[i2] = 0;
					k.kp[c][i1].note[i2] = 0;
					k.kp[c][i1].sc[i2] = 0;
					k.kp[c][i1].trans[i2] = 0;
				}

				// k.kp[c][i1].sync = 1;
				k.kp[c][i1].dur_mul = 4;

				for(i2=0;i2<NUM_PARAMS;i2++) {
					k.kp[c][i1].lstart[i2] = 0;
					k.kp[c][i1].lend[i2] = 5;
					k.kp[c][i1].llen[i2] = 6;
					k.kp[c][i1].lswap[i2] = 0;
					k.kp[c][i1].tmul[i2] = 1;
					k.kp[c][i1].tdiv[i2] = 1;
				}
			}
		}

		for(i1=0;i1<7;i1++)
			k.pscale[i1] = i1;

		for(i1=0;i1<42;i1++) {
			scales[i1][0] = 0;
			for(i2=0;i2<6;i2++)
				scales[i1][i2+1] = 1;
		}

		// save all presets, clear glyphs
		for(i1=0;i1<8;i1++) {
			flashc_memcpy((void *)&flashy.k[i1], &k, sizeof(k), true);
			glyph[i1] = (1<<i1);
			flashc_memcpy((void *)&flashy.glyph[i1], &glyph, sizeof(glyph), true);
		}

		flashc_memcpy((void *)&flashy.scales, &scales, sizeof(scales), true);
	}
	else {
		// load from flash at startup
		preset_select = flashy.preset_select;
		// edit_mode = flashy.edit_mode;
		flash_read();
		for(i1=0;i1<8;i1++)
			glyph[i1] = flashy.glyph[preset_select][i1];
	}

	re = &refresh;

	process_ii = &ww_process_ii;

	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	timer_add(&note0offTimer,10000,&note0offTimer_callback, NULL);
	timer_add(&sc0Timer,1000,&sc0Timer_callback, NULL);
	timer_add(&note0Timer,1000,&note0Timer_callback, NULL);
	timer_add(&trans0Timer,1000,&trans0Timer_callback, NULL);
	timer_add(&dur0Timer,1000,&dur0Timer_callback, NULL);
	timer_add(&oct0Timer,1000,&oct0Timer_callback, NULL);
	timer_add(&ac0Timer,1000,&ac0Timer_callback, NULL);
	timer_add(&tr0Timer,1000,&tr0Timer_callback, NULL);

	timer_add(&note1offTimer,10000,&note1offTimer_callback, NULL);
	timer_add(&trans1Timer,1000,&trans1Timer_callback, NULL);
	timer_add(&sc1Timer,1000,&sc1Timer_callback, NULL);
	timer_add(&note1Timer,1000,&note1Timer_callback, NULL);
	timer_add(&dur1Timer,1000,&dur1Timer_callback, NULL);
	timer_add(&oct1Timer,1000,&oct1Timer_callback, NULL);
	timer_add(&ac1Timer,1000,&ac1Timer_callback, NULL);
	timer_add(&tr1Timer,1000,&tr1Timer_callback, NULL);

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
