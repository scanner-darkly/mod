#include <stdio.h>	//sprintf
#include <ctype.h>  //toupper
#include <string.h> //memcpy

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
#include "uhi_msc.h"
#include "fat.h"
#include "file.h"
#include "fs_com.h"
#include "navigation.h"
#include "usb_protocol_msc.h"
#include "uhi_msc_mem.h"

// system
#include "types.h"
#include "events.h"
#include "init.h"
#include "interrupts.h"
#include "i2c.h"
#include "kbd.h"
// #include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
// #include "ftdi.h"
#include "hid.h"
#include "screen.h"
#include "region.h"
#include "font.h"


// this
#include "conf_board.h"
#include "teletype.h"
#include "help.h"

#define METRO_SCRIPT 8
#define INIT_SCRIPT 9

#define RATE_CLOCK 10
#define RATE_CV 6

#define SCENE_SLOTS 32
#define SCENE_SLOTS_ 31

#define SCENE_TEXT_LINES 32
#define SCENE_TEXT_CHARS 32


uint8_t preset, preset_select, front_timer, preset_edit_line, preset_edit_offset, offset_view, last_mode;

u16 adc[4];

typedef struct {
	u16 now;
	u16 off;
	u16 target;
	u16 slew;
	u16 step;
	s32 delta;
	u32 a;
} aout_t;

aout_t aout[4];

volatile uint8_t input_states[8];

u8 mutes[8];

error_t status;

char input[32];
char input_buffer[32];
int num_buffer;
uint8_t pos;

uint8_t knob_now;
uint8_t knob_last;

tele_script_t script[10];
tele_script_t history;
uint8_t edit, edit_line, edit_index, edit_pattern, offset_index;
char scene_text[SCENE_TEXT_LINES][SCENE_TEXT_CHARS];

uint8_t metro_act;
unsigned int metro_time;

uint8_t mod_SH;
uint8_t mod_ALT;
uint8_t mod_CTRL;
uint8_t mod_META;

uint16_t hold_up;
uint16_t hold_down;
uint16_t hold_right;
uint16_t hold_left;

uint8_t help_page;
uint8_t help_length[8] = {HELP1_LENGTH, HELP2_LENGTH, HELP3_LENGTH, HELP4_LENGTH, HELP5_LENGTH, HELP6_LENGTH, HELP7_LENGTH, HELP8_LENGTH };


#define FIRSTRUN_KEY 0x22

typedef const struct {
	tele_script_t script[10];
	tele_pattern_t patterns[4];
	char text[SCENE_TEXT_LINES][SCENE_TEXT_CHARS];
} tele_scene_t;

typedef const struct {
	tele_scene_t s[SCENE_SLOTS];
	uint8_t scene;
	uint8_t mode;
	uint8_t fresh;
} nvram_data_t;

// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t f;



#define M_LIVE 0
#define M_EDIT 1
#define M_TRACK 2
#define M_PRESET_W 3
#define M_PRESET_R 4
#define M_HELP 5

uint8_t mode;

#define R_PRESET (1<<0)
#define R_INPUT (1<<1)
#define R_MESSAGE (1<<2)
#define R_LIST (1<<3)
#define R_ALL 0xf
uint8_t r_edit_dirty;

#define A_METRO 0x1
#define A_TR 0x2
#define A_SLEW 0x4
#define A_DELAY 0x8
#define A_Q 0x10
#define A_X 0x20
#define A_REFRESH 0x40
#define A_MUTES 0x80
uint8_t activity;
uint8_t activity_prev;

static region line[8] = {
	{.w = 128, .h = 8, .x = 0, .y = 0},
	{.w = 128, .h = 8, .x = 0, .y = 8},
	{.w = 128, .h = 8, .x = 0, .y = 16},
	{.w = 128, .h = 8, .x = 0, .y = 24},
	{.w = 128, .h = 8, .x = 0, .y = 32},
	{.w = 128, .h = 8, .x = 0, .y = 40},
	{.w = 128, .h = 8, .x = 0, .y = 48},
	{.w = 128, .h = 8, .x = 0, .y = 56}
};

uint8_t sdirty;


////////////////////////////////////////////////////////////////////////////////
// prototypes

// check the event queue
static void check_events(void);

// handler protos
// static void handler_None(s32 data) { ;; }
static void handler_KeyTimer(s32 data);
static void handler_Front(s32 data);
static void handler_HidConnect(s32 data);
static void handler_HidDisconnect(s32 data);
static void handler_HidPacket(s32 data);
static void handler_Trigger(s32 data);
static void handler_ScreenRefresh(s32 data);
static void handler_II(s32 data);


static u8 flash_is_fresh(void);
static void flash_unfresh(void);
static void flash_write(void);
static void flash_read(void);

static void render_init(void);

static void set_mode(uint8_t);

static void tele_metro(int16_t, int16_t, uint8_t);
static void tele_tr(uint8_t i, int16_t v);
static void tele_cv(uint8_t i, int16_t v, uint8_t s);
static void tele_cv_slew(uint8_t i, int16_t v);
static void tele_delay(uint8_t i);
static void tele_s(uint8_t i);
static void tele_cv_off(uint8_t i, int16_t v);
static void tele_ii(uint8_t i, int16_t d);
static void tele_scene(uint8_t i);
static void tele_pi(void);
static void tele_script(uint8_t a);
static void tele_kill(void);
static void tele_mute(uint8_t, uint8_t);
static void tele_input_state(uint8_t);

static void tele_usb_disk(void);
static void tele_mem_clear(void);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application


////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t refreshTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t cvTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t hidTimer = { .next = NULL, .prev = NULL };
static softTimer_t metroTimer = { .next = NULL, .prev = NULL };


static void cvTimer_callback(void* o) { 
	u8 i, r=0;
	u16 a;

	activity &= ~A_SLEW;

	for(i=0;i<4;i++)
		if(aout[i].step) {
			aout[i].step--;

			if(aout[i].step == 0) {
				aout[i].now = aout[i].target;
			}
			else {
				aout[i].a += aout[i].delta;
				aout[i].now = aout[i].a >> 16;
				activity |= A_SLEW;
			}

			r++;
		}

	if(r) {
		spi_selectChip(SPI,DAC_SPI);
		spi_write(SPI,0x31);
		a = aout[2].now >> 2;
		spi_write(SPI,a>>4);
		spi_write(SPI,a<<4);
		spi_write(SPI,0x31);
		a = aout[0].now >> 2;
		spi_write(SPI,a>>4);
		spi_write(SPI,a<<4);
		spi_unselectChip(SPI,DAC_SPI);

		spi_selectChip(SPI,DAC_SPI);
		spi_write(SPI,0x38);
		a = aout[3].now >> 2;
		spi_write(SPI,a>>4);
		spi_write(SPI,a<<4);
		spi_write(SPI,0x38);
		a = aout[1].now >> 2;
		spi_write(SPI,a>>4);
		spi_write(SPI,a<<4);
		spi_unselectChip(SPI,DAC_SPI);
	}
}

static void clockTimer_callback(void* o) {  
	// static event_t e;
	// e.type = kEventTimer;
	// e.data = 0;
	// event_post(&e);
	// print_dbg("\r\ntimer.");

	// clock_phase++;
	// if(clock_phase>1) clock_phase=0;
	// (*clock_pulse)(clock_phase);

	// clock_time++;

	tele_tick(RATE_CLOCK);

	// i2c_master_tx(d);
}

static void refreshTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventScreenRefresh;
	e.data = 0;
	event_post(&e);

	if(sdirty) {
		for(int i=0;i<8;i++)
			region_draw(&line[i]);

		sdirty = 0;
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

static void hidTimer_callback(void* o) {
	static event_t e;
	e.type = kEventHidTimer;
	e.data = 0;
	event_post(&e);
}

static void metroTimer_callback(void* o) {
	// print_dbg("*");
	uint8_t i;

	if(script[METRO_SCRIPT].l) {
		activity |= A_METRO;
		for(i=0;i<script[METRO_SCRIPT].l;i++)
			process(&script[METRO_SCRIPT].c[i]);
	}
	else
		activity &= ~A_METRO;
}






///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_Front(s32 data) {
	// print_dbg("\r\n //// FRONT HOLD");

	if(data == 0) {
		if(mode != M_PRESET_R) {
			front_timer = 0;
			knob_last = adc[1]>>7;
			last_mode = mode;
			set_mode(M_PRESET_R);
		}
		else
			front_timer = 15;
	}
	else {
		if(front_timer) {
			set_mode(last_mode);
		}
		front_timer = 0;
	}
}

static void handler_PollADC(s32 data) {
	adc_convert(&adc);

	tele_set_val(V_IN,adc[0]<<2);	// IN

	if(mode == M_TRACK && mod_CTRL) {
		if(mod_SH)
			tele_patterns[edit_pattern].v[edit_index+offset_index] = adc[1]>>2;
		else
			tele_patterns[edit_pattern].v[edit_index+offset_index] = adc[1]>>7;
		r_edit_dirty |= R_ALL;
	}
	else if(mode == M_PRESET_R) {
		knob_now = adc[1]>>7;
		if(knob_now != knob_last) {
			preset_select = knob_now;
			r_edit_dirty = R_ALL; 
		}
		knob_last = knob_now;
	}
	else
		tele_set_val(V_PARAM,adc[1]<<2);	// PARAM

	// print_dbg("\r\nadc:\t"); print_dbg_ulong(adc[0]);
	// print_dbg("\t"); print_dbg_ulong(adc[1]);
	// print_dbg("\t"); print_dbg_ulong(adc[2]);
	// print_dbg("\t"); print_dbg_ulong(adc[3]);
}

static void handler_SaveFlash(s32 data) {
	flash_write();
}

static void handler_KeyTimer(s32 data) {
	if(front_timer) {
		if(front_timer == 1) {
			flash_read();

			for(int i=0;i<script[INIT_SCRIPT].l;i++)
				process(&script[INIT_SCRIPT].c[i]);

			set_mode(last_mode);

			front_timer--;
		}
		else front_timer--;
	}

	#define KEY_REPEAT_HOLD 4
	if(hold_up) {
		if(mode == M_TRACK) {
			if(hold_up > KEY_REPEAT_HOLD) {
				if(edit_index)
					edit_index--;
				else if(offset_index)
					offset_index--;
				r_edit_dirty |= R_ALL;
			}
		}
		else if(mode == M_PRESET_R) {
			if(preset_edit_offset) {
				preset_edit_offset--;
				r_edit_dirty |= R_ALL;
			}
		}
		else if(mode == M_HELP) {
			if(offset_view) {
				offset_view--;
				r_edit_dirty |= R_ALL;
			}
		}	
		hold_up++;
	}
	if(hold_down) {
		if(mode == M_TRACK) {
			if(hold_down > KEY_REPEAT_HOLD) {
				edit_index++;
				if(edit_index == 8) {
					edit_index = 7;
					if(offset_index < 56) {
						offset_index++;
					}
				}
				r_edit_dirty |= R_ALL;
			}
		}
		else if(mode == M_PRESET_R) {
			if(preset_edit_offset < 24) {
				preset_edit_offset++;
				r_edit_dirty |= R_ALL;
			}
		}
		else if(mode == M_HELP) {
			if(offset_view < help_length[help_page] - 8) {
				offset_view++;
				r_edit_dirty |= R_ALL;
			}
		}
		hold_down++;
	}
	if(hold_left) {
		if(hold_left > KEY_REPEAT_HOLD) {
			if(pos) {
				pos--;
				r_edit_dirty |= R_INPUT;
			}
		}
		hold_left++;
	}
	if(hold_right) {
		if(hold_right > KEY_REPEAT_HOLD) {
			if(pos < strlen(input)) {
				pos++;
				r_edit_dirty |= R_INPUT;
			}
		}
		hold_right++;
	}


}

static void handler_HidConnect(s32 data) {
	// print_dbg("\r\nhid connect\r\n");
	timer_add(&hidTimer,47,&hidTimer_callback, NULL);
}

static void handler_HidDisconnect(s32 data) {
	timer_remove(&hidTimer);
	// print_dbg("\r\nno more hid");

}


///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// keys

static void handler_HidTimer(s32 data) {
	u8 i,n;

	const s8* frame;
	if(hid_get_frame_dirty()) {
		frame = (const s8*)hid_get_frame_data();

     	for(i=2;i<8;i++) {
     		if(frame[i] == 0) {
     			mod_SH = frame[0] & SHIFT;
     			mod_CTRL = frame[0] & CTRL;
     			mod_ALT = frame[0] & ALT;
     			mod_META = frame[0] & META;
     			if(i==2) {
     				hold_up = 0;
     				hold_down = 0;
     				hold_right = 0;
     				hold_left = 0;
     			}
     			break;
     		}

     		if(frame_compare(frame[i]) == false) {
     			// CTRL = 1
     			// SHIFT = 2
     			// ALT = 4
     			// META = 8

     			// print_dbg("\r\nk: ");
     			// print_dbg_hex(frame[i]);
     			// print_dbg("\r\nmod: ");
     			// print_dbg_hex(frame[0]);
     			switch(frame[i]) {
     				case 0x2B: // tab
     					if(mode == M_LIVE)
     						set_mode(M_EDIT);
     					else
     						set_mode(M_LIVE);
     					break;
     				case 0x35: // ~
     					if(mode == M_TRACK)
     						set_mode(last_mode);
     					else {
     						last_mode = mode;
     						set_mode(M_TRACK);
     					}
     					break;
     				case 0x29: // ESC
     					if(mod_ALT) {
     						last_mode = mode;
     						set_mode(M_PRESET_W);
     					}
     					else if(mod_META) {
     						clear_delays();
     						for(int i=0;i<4;i++) {
     							aout[i].step = 1;
     						}
     					}
     					else if(mode == M_PRESET_R)
     						set_mode(last_mode);
     					else {
     						last_mode = mode;
     						set_mode(M_PRESET_R);
     					}

     					break;
     				case 0x3A: // F1
     					if(mode == M_HELP) 
     						set_mode(last_mode);
     					else {
     						last_mode = mode;
     						set_mode(M_HELP);
     					}
     					break;
     				case 0x51: // down
     					hold_down = 1;
     					if(mode == M_TRACK) {
     						if(mod_ALT) {
     							if(offset_index < 48)
     								offset_index += 8;
     							else {
     								offset_index = 56;
     								edit_index = 7;
     							}
     						}
     						else {
     							edit_index++;
	     						if(edit_index == 8) {
	     							edit_index = 7;
	     							if(offset_index < 56) {
	     								offset_index++;
	     							}
	     						}
	     					}
	     					r_edit_dirty |= R_ALL;
     					}
     					else if(mode == M_PRESET_W) {
     						if((preset_edit_offset + preset_edit_line) < 31) {
     							if(preset_edit_line == 5)
     								preset_edit_offset++;
     							else
     								preset_edit_line++;
     							strcpy(input,scene_text[preset_edit_line + preset_edit_offset]);
	     						pos = strlen(input);
     							r_edit_dirty |= R_ALL;
     						}
     					}
     					else if(mode == M_PRESET_R) {
     						if(preset_edit_offset < 24) {
     							preset_edit_offset++;
     							r_edit_dirty |= R_ALL;
     						}
     					}
     					else if(mode == M_HELP) {
     						if(offset_view < help_length[help_page] - 8) {
     							offset_view++;
     							r_edit_dirty |= R_ALL;
     						}
     					}
     					else if(edit_line < SCRIPT_MAX_COMMANDS_) {
     						if(mode == M_LIVE) {
     							edit_line++;
     							strcpy(input,print_command(&history.c[edit_line]));
     							pos = strlen(input);
     							for(n = pos;n < 32;n++) input[n] = 0;
	 						}
     						else if(script[edit].l > edit_line) {
     							edit_line++;
     							strcpy(input,print_command(&script[edit].c[edit_line]));
     							pos = strlen(input);
     							for(n = pos;n < 32;n++) input[n] = 0;
	 							r_edit_dirty |= R_LIST;
	 						}
	 					}
	 					else if(mode == M_LIVE) {
	 						edit_line=SCRIPT_MAX_COMMANDS;
	 						pos = 0;
     						for(n = 0;n < 32;n++) input[n] = 0;
	 					}
     					break;

     				case 0x52: // up
     					hold_up = 1;
     					if(mode == M_TRACK) {
     						if(mod_ALT) {
     							if(offset_index > 8) {
     								offset_index -= 8;
     							}
     							else {
     								offset_index = 0;
     								edit_index = 0;
     							}
     						}
     						else {
	     						if(edit_index)
	     							edit_index--;
	     						else if(offset_index)
	     							offset_index--;
	     					}
	     					r_edit_dirty |= R_ALL;
     					}
     					else if(mode == M_PRESET_W) {
     						if(preset_edit_line + preset_edit_offset) {
     							if(preset_edit_line)
     								preset_edit_line--;
     							else
     								preset_edit_offset--;
     							strcpy(input,scene_text[preset_edit_line + preset_edit_offset]);
     							pos = strlen(input);
     							r_edit_dirty |= R_ALL;
     						}
     					}
     					else if(mode == M_PRESET_R) {
     						if(preset_edit_offset) {
     							preset_edit_offset--;
     							r_edit_dirty |= R_ALL;
     						}
     					}
     					else if(mode == M_HELP) {
     						if(offset_view) {
     							offset_view--;
     							r_edit_dirty |= R_ALL;
     						}
     					}
     					else if(edit_line) {
     						edit_line--;
     						if(mode == M_LIVE)
     							strcpy(input,print_command(&history.c[edit_line]));
     						else
     							strcpy(input,print_command(&script[edit].c[edit_line]));

     						pos = strlen(input);
	     					for(n = pos;n < 32;n++) input[n] = 0;
 							if(mode != M_LIVE) r_edit_dirty |= R_LIST;
 						}
     					break;
     				case 0x50: // back
     					hold_left = 1;
     					if(mode == M_TRACK) {
     						if(mod_ALT) {
     							edit_index = 0;
     							offset_index = 0;
     						}
     						else {
	     						if(edit_pattern > 0)
	     							edit_pattern--;
	     					}
	     					r_edit_dirty |= R_ALL;
     					}
     					else if(pos) {
     						pos--;
     					}
     					break;

     				case 0x4f: // forward
     					hold_right = 1;
     					if(mode == M_TRACK) {
     						if(mod_ALT) {
     							edit_index = 7;
     							offset_index = 56;
     						}
     						else {
	     						if(edit_pattern < 3)
	     							edit_pattern++;
	     					}
	     					r_edit_dirty |= R_ALL;
     					}
     					else if(pos < strlen(input)) {
     						pos++;
     					}

     					break;

     				case 0x30: // ]
						if(mode == M_EDIT) {
	     					edit++;
	     					if(edit==10)
	     						edit = 0;
	     					if(edit_line > script[edit].l)
	     						edit_line = script[edit].l;
	     					strcpy(input,print_command(&script[edit].c[edit_line]));
	 						pos = strlen(input);
	     					for(n = pos;n < 32;n++) input[n] = 0;


	     					r_edit_dirty |= R_LIST;
 						}
 						else if(mode == M_PRESET_W || mode == M_PRESET_R) {
 							if(preset_select < SCENE_SLOTS_) preset_select++;
 							r_edit_dirty |= R_ALL;
 						}
 						else if(mode == M_TRACK) {
 							if(tele_patterns[edit_pattern].v[edit_index+offset_index] < 32766) {
 								tele_patterns[edit_pattern].v[edit_index+offset_index]++;
 								r_edit_dirty |= R_ALL;
 							}
 						}
 						else if(mode == M_HELP) {
 							if(help_page < 7) {
 								offset_view = 0;
 								help_page++;
 								r_edit_dirty |= R_ALL;
 							}
 						}
     					break;

     				case 0x2F: // [
     					if(mode == M_EDIT) {
	     					if(edit) 
	     						edit--;
	     					else 
	     						edit = 9;
	     					if(edit_line > script[edit].l)
	     						edit_line = script[edit].l;
	     					strcpy(input,print_command(&script[edit].c[edit_line]));
	 						pos = strlen(input);
	     					for(n = pos;n < 32;n++) input[n] = 0;
	 						r_edit_dirty |= R_LIST;
	 					}
	 					else if(mode == M_PRESET_W  || mode == M_PRESET_R) {
	 						if(preset_select) preset_select--;
	 						r_edit_dirty |= R_ALL;
	 					}
	 					else if(mode == M_TRACK) {
	 						if(tele_patterns[edit_pattern].v[edit_index+offset_index] > -32767) {
 								tele_patterns[edit_pattern].v[edit_index+offset_index]--;
 								r_edit_dirty |= R_ALL;
 							}
 						}
 						else if(mode == M_HELP) {
 							if(help_page) {
 								offset_view = 0;
 								help_page--;
 								r_edit_dirty |= R_ALL;
 							}
 						}
     					break;
     				case BACKSPACE:
     					if(mode == M_LIVE || mode == M_EDIT || mode == M_PRESET_W) {
     						if(mod_SH) {
     							for(n = 0;n < 32;n++)
		     						input[n] = 0;
		     					pos = 0;
     						}
	     					else if(pos) {
	     						pos--;
	     						// input[pos] = ' ';
	     						for(int x = pos; x < 31; x++)
	     							input[x] = input[x+1];
		     				}
		     			}
		     			else if(mode == M_TRACK) {
		     				if(mod_SH) {
		     					for(int i = edit_index+offset_index;i<63;i++)
		     						tele_patterns[edit_pattern].v[i] = tele_patterns[edit_pattern].v[i + 1];

		     					if(tele_patterns[edit_pattern].l > edit_index+offset_index)
		     						tele_patterns[edit_pattern].l--;
		     				}
		     				else {
			     				tele_patterns[edit_pattern].v[edit_index+offset_index] =
			     					tele_patterns[edit_pattern].v[edit_index+offset_index] / 10;
			     			}
		     				r_edit_dirty |= R_ALL;
		     			}
     					break;

     				case RETURN:
     					if(mode == M_EDIT || mode == M_LIVE) {
	     					status = parse(input);

	     					if(status == E_OK) {
								status = validate(&temp);

								if(status == E_OK) {
									if(mode == M_LIVE) {
										edit_line = SCRIPT_MAX_COMMANDS;

										if(temp.l) {
											memcpy(&history.c[0], &history.c[1], sizeof(tele_command_t));
											memcpy(&history.c[1], &history.c[2], sizeof(tele_command_t));
											memcpy(&history.c[2], &history.c[3], sizeof(tele_command_t));
											memcpy(&history.c[3], &history.c[4], sizeof(tele_command_t));
											memcpy(&history.c[4], &history.c[5], sizeof(tele_command_t));
											memcpy(&history.c[5], &temp, sizeof(tele_command_t));

											process(&temp);
										}

										for(n = 0;n < 32;n++)
				     						input[n] = 0;
				     					pos = 0;
									}
									else {
										if(temp.l == 0) {	// BLANK LINE
											if(script[edit].l && script[edit].c[edit_line].l) {
												// print_dbg("\r\nl ");
												// print_dbg_ulong(script[edit].l);

												script[edit].l--;

			     								for(n=edit_line;n<script[edit].l;n++)
			     									memcpy(&script[edit].c[n], &script[edit].c[n+1], sizeof(tele_command_t));

			     								script[edit].c[script[edit].l].l = 0;
			     								
			     								if(edit_line > script[edit].l)
			     									edit_line = script[edit].l;
			     								strcpy(input,print_command(&script[edit].c[edit_line]));
						 						pos = strlen(input);
						 						// print_dbg(" -> ");
												// print_dbg_ulong(script[edit].l);
			     							}
										}
										else if(mod_SH) { // SHIFT = INSERT
											for(n=script[edit].l;n>edit_line;n--) 
		     									memcpy(&script[edit].c[n], &script[edit].c[n-1], sizeof(tele_command_t));

		     								if(script[edit].l < SCRIPT_MAX_COMMANDS)
		     									script[edit].l++;

											memcpy(&script[edit].c[edit_line], &temp, sizeof(tele_command_t));
											if((edit_line == script[edit].l) && (script[edit].l < 4))
												script[edit].l++;
											if(edit_line < SCRIPT_MAX_COMMANDS_) {
												edit_line++;
												strcpy(input,print_command(&script[edit].c[edit_line]));
				     							pos = strlen(input);
				     							for(n = pos;n < 32;n++) input[n] = 0;
				     						}
				     					}
			     						else {
											memcpy(&script[edit].c[edit_line], &temp, sizeof(tele_command_t));
											if((edit_line == script[edit].l) && (script[edit].l < SCRIPT_MAX_COMMANDS))
												script[edit].l++;
											if(edit_line < SCRIPT_MAX_COMMANDS_) {
												edit_line++;
												strcpy(input,print_command(&script[edit].c[edit_line]));
				     							pos = strlen(input);
				     							for(n = pos;n < 32;n++) input[n] = 0;
				     						}
			     						}

				     					r_edit_dirty |= R_MESSAGE;
			     					}
			     					if(mode == M_EDIT)
			     						r_edit_dirty |= R_LIST;
								}
								else {
									// print_dbg("\r\nvalidate: ");
									// print_dbg(tele_error(status));
	 							}
							}
							else {
								// print_dbg("\r\nERROR: ");
								// print_dbg(tele_error(status));
							}

							// print_dbg("\r\n\n> ");

	     					r_edit_dirty |= R_MESSAGE;
	     				}
	     				else if(mode == M_PRESET_W) {
	     					if(mod_ALT) {
	     						strcpy(scene_text[preset_edit_line+preset_edit_offset],input);
 								flash_write();
 								for(n = 0;n < 32;n++) input[n] = 0;
		 							pos = 0;
 								set_mode(last_mode);
	     					}
	     					else {
		     					strcpy(scene_text[preset_edit_line+preset_edit_offset],input);
		     					if(preset_edit_line + preset_edit_offset < 31) {
			     					if(preset_edit_line == 5)
		 								preset_edit_offset++;
		 							else
		 								preset_edit_line++;
		 						}
	 							strcpy(input,scene_text[preset_edit_line + preset_edit_offset]);
	     						pos = strlen(input);
	 							r_edit_dirty |= R_ALL;
	 						}
	     				}
	     				else if(mode == M_PRESET_R) {
	     					flash_read();
	     					tele_set_val(V_SCENE, preset_select);

     						for(int i=0;i<script[INIT_SCRIPT].l;i++)
								process(&script[INIT_SCRIPT].c[i]);

							for(n = 0;n < 32;n++) input[n] = 0;
	 							pos = 0;
							set_mode(last_mode);
	     				}
	     				else if(mode == M_TRACK) {
	     					if(mod_SH) {
	     						for(int i=63;i>edit_index+offset_index;i--)
	     							tele_patterns[edit_pattern].v[i] = tele_patterns[edit_pattern].v[i-1];
	     						if(tele_patterns[edit_pattern].l < 63)
	     							tele_patterns[edit_pattern].l++;
	     						r_edit_dirty |= R_ALL;
	     					}
	     					else {
	     						if(edit_index+offset_index == tele_patterns[edit_pattern].l && tele_patterns[edit_pattern].l < 64) {
	     							tele_patterns[edit_pattern].l++;
	     							edit_index++;
		     						if(edit_index == 8) {
		     							edit_index = 7;
		     							if(offset_index < 56) {
		     								offset_index++;
		     							}
		     						}
	     							r_edit_dirty |= R_ALL;
	     						}
	     					}
	     				}
     					break;

 					default:
 						if(mod_ALT) {	// ALT
	     					if(frame[i] == 0x1b) {	// x CUT
	     						if(mode == M_EDIT || mode == M_LIVE) {
		     						memcpy(&input_buffer, &input, sizeof(input));
		     						if(mode == M_LIVE) {
		     							for(n = 0;n < 32;n++)
				     						input[n] = 0;
				     					pos = 0;
		     						}
		     						else {
		     							if(script[edit].l) {
		     								script[edit].l--;
		     								for(n=edit_line;n<script[edit].l;n++)
		     									memcpy(&script[edit].c[n], &script[edit].c[n+1], sizeof(tele_command_t));

		     								script[edit].c[script[edit].l].l = 0;
		     								if(edit_line > script[edit].l)
		     									edit_line = script[edit].l;
		     								strcpy(input,print_command(&script[edit].c[edit_line]));
					 						pos = strlen(input);
		     							}

		     							r_edit_dirty |= R_LIST;
		     						}
		     					}
		     					else if(mode == M_TRACK) {
 									num_buffer = tele_patterns[edit_pattern].v[edit_index+offset_index];
 									for(int i = edit_index+offset_index;i<63;i++)
			     						tele_patterns[edit_pattern].v[i] = tele_patterns[edit_pattern].v[i + 1];

			     					if(tele_patterns[edit_pattern].l > edit_index+offset_index)
			     						tele_patterns[edit_pattern].l--;
 									r_edit_dirty |= R_ALL;
 								}
	     					}
	     					else if(frame[i] == 0x06) { // c COPY
	     						if(mode == M_EDIT || mode == M_LIVE) {
	     							memcpy(&input_buffer, &input, sizeof(input));
	     						}
	     						else if(mode == M_TRACK) {
	     							num_buffer = tele_patterns[edit_pattern].v[edit_index+offset_index];
	     							r_edit_dirty |= R_ALL;
	     						}
	     					}
	     					else if(frame[i] == 0x19) { // v PASTE
	     						if(mode == M_EDIT || mode == M_LIVE) {
	     							memcpy(&input, &input_buffer, sizeof(input));
	     							pos = strlen(input);
	     						}
	     						else if(mode == M_TRACK) {
	     							if(mod_SH) {
	     								for(int i=63;i>edit_index+offset_index;i--)
			     							tele_patterns[edit_pattern].v[i] = tele_patterns[edit_pattern].v[i-1];
			     						if(tele_patterns[edit_pattern].l >= edit_index+offset_index)
			     							if(tele_patterns[edit_pattern].l < 63)
			     								tele_patterns[edit_pattern].l++;
	     							}
	     							tele_patterns[edit_pattern].v[edit_index+offset_index] = num_buffer;
	     							r_edit_dirty |= R_ALL;
	     						}
	     					}
	     					else if(mode == M_TRACK) {
	     						n = hid_to_ascii_raw(frame[i]);
	     						if(n == 'L') {
	     							if(tele_patterns[edit_pattern].l) {
	     								offset_index = ((tele_patterns[edit_pattern].l - 1) >> 3) << 3;
	     								edit_index = (tele_patterns[edit_pattern].l - 1) & 0x7;

	     								int8_t delta = edit_index - 3;

	     								if((offset_index + delta > 0) && (offset_index + delta < 56)) {
	     									offset_index += delta;
	     									edit_index = 3;
	     								}
	     							}
	     							else {
	     								offset_index = 0;
	     								edit_index = 0;
	     							}
	     							r_edit_dirty |= R_ALL;
	     						}
	     						else if(n == 'S') {
	     							if(tele_patterns[edit_pattern].start) {
	     								offset_index = ((tele_patterns[edit_pattern].start) >> 3) << 3;
	     								edit_index = (tele_patterns[edit_pattern].start) & 0x7;

	     								int8_t delta = edit_index - 3;

	     								if((offset_index + delta > 0) && (offset_index + delta < 56)) {
	     									offset_index += delta;
	     									edit_index = 3;
	     								}
	     							}
	     							else {
	     								offset_index = 0;
	     								edit_index = 0;
	     							}
	     							r_edit_dirty |= R_ALL;
	     						}
	     						else if(n == 'E') {
	     							if(tele_patterns[edit_pattern].end) {
	     								offset_index = ((tele_patterns[edit_pattern].end) >> 3) << 3;
	     								edit_index = (tele_patterns[edit_pattern].end) & 0x7;

	     								int8_t delta = edit_index - 3;

	     								if((offset_index + delta > 0) && (offset_index + delta < 56)) {
	     									offset_index += delta;
	     									edit_index = 3;
	     								}
	     							}
	     							else {
	     								offset_index = 0;
	     								edit_index = 0;
	     							}
	     							r_edit_dirty |= R_ALL;
	     						}
	     					}
	     				}
	     				else if(mod_SH && mode == M_TRACK) {
     						n = hid_to_ascii_raw(frame[i]);
     						if(n == 'L') {
     							tele_patterns[edit_pattern].l = edit_index + offset_index + 1;
     							r_edit_dirty |= R_ALL;
     						}
     						else if(n == 'S') {
     							tele_patterns[edit_pattern].start = offset_index + edit_index;
     						}
     						else if(n == 'E') {
     							tele_patterns[edit_pattern].end = offset_index + edit_index;
     						}

	     				}
	     				else if(mod_META) {
     						n = hid_to_ascii_raw(frame[i]);

     						if(n > 0x30 && n < 0x039) {
     							if(mod_SH) {
     								mutes[n-0x31] ^= 1;
     								activity |= A_MUTES;
     							}
     							else
     								tele_script(n - 0x30);
     						}
     						else if(n == 'M') {
     							for(int i=0;i<script[METRO_SCRIPT].l;i++)
									process(&script[METRO_SCRIPT].c[i]);
     						}
     						else if(n == 'I') {
     							for(int i=0;i<script[INIT_SCRIPT].l;i++)
									process(&script[INIT_SCRIPT].c[i]);
     						}
	     				}
	     				else if(mode == M_TRACK) {
	     					n = hid_to_ascii(frame[i], frame[0]);

	     					if(n > 0x2F && n < 0x03A) {
	     						if(tele_patterns[edit_pattern].v[edit_index+offset_index]) {
	     							// limit range
	     							if(tele_patterns[edit_pattern].v[edit_index+offset_index] < 3276 &&
	     								tele_patterns[edit_pattern].v[edit_index+offset_index] > -3276)
	     							{
		     							tele_patterns[edit_pattern].v[edit_index+offset_index] =
			     							tele_patterns[edit_pattern].v[edit_index+offset_index] * 10;
			     						if(tele_patterns[edit_pattern].v[edit_index+offset_index] > 0)
			     							tele_patterns[edit_pattern].v[edit_index+offset_index] += n - 0x30;
			     						else
			     							tele_patterns[edit_pattern].v[edit_index+offset_index] -= n - 0x30;
			     					}
	     						}
	     						else
	     							tele_patterns[edit_pattern].v[edit_index+offset_index] = n - 0x30;
	     						r_edit_dirty |= R_ALL;
	     					}
	     					else if(n == 0x2D) { // - 
	     					    tele_patterns[edit_pattern].v[edit_index+offset_index] = -tele_patterns[edit_pattern].v[edit_index+offset_index];
     							r_edit_dirty |= R_ALL;
     						}
     						else if(n == 0x20) { // space
     							if(tele_patterns[edit_pattern].v[edit_index+offset_index])
     								tele_patterns[edit_pattern].v[edit_index+offset_index] = 0;
     							else
     								tele_patterns[edit_pattern].v[edit_index+offset_index] = 1;
     							r_edit_dirty |= R_ALL;
     						}
	     				}
	     				else {	/// NORMAL TEXT ENTRY
	     					if(frame[i] > 0x58 && frame[i] < 0x61) {
	     						tele_script(frame[i] - 0x58);
	     					}
	 						if(pos<29) {
		     					// print_dbg_char(hid_to_ascii(frame[i], frame[0]));
		     					n = hid_to_ascii(frame[i], frame[0]);
		     					if(n) {
		     						for(int x = 31; x > pos; x--)
		     							input[x] = input[x-1];

		     						input[pos] = n;
		     						pos++;
		     					}
		     					// pos++;
		     					// input[pos] = 0;
	     					}
	     				}

     					break;
     			}

     			r_edit_dirty |= R_INPUT;
     		}
     	}

     	set_old_frame(frame);

		// print_dbg("\r\nhid:\t");
		// for(i=0;i<8;i++) {
		// 	print_dbg_ulong( (int) frame[i] ); 
		// 	print_dbg("\t");  
		// }
	}

    hid_clear_frame_dirty();
}


static void handler_HidPacket(s32 data) {
	// print_dbg("\r\nhid packet");
}


static void handler_Trigger(s32 data) {
	// print_dbg("*");

	// for(int n=0;n<script.l;n++)
	if(mutes[data])
		for(int i=0;i<script[data].l;i++) {
			process(&script[data].c[i]);
		}
}



///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// refresh

static void handler_ScreenRefresh(s32 data) {
	static uint8_t a;
	static char s[32];

	uint8_t y,x,i;

	if(mode == M_TRACK) {
		if(r_edit_dirty & R_ALL) {
			for(y=0;y<8;y++) {
				region_fill(&line[y], 0);
				itoa(y+offset_index, s, 10);
				font_string_region_clip_right(&line[y], s, 4, 0, 0x1, 0);

				for(x=0;x<4;x++) {
					if(tele_patterns[x].l > y+offset_index) a = 6; else a = 1;
					itoa(tele_patterns[x].v[y+offset_index], s, 10);
					font_string_region_clip_right(&line[y], s, (x+1)*30+4, 0, a, 0);

					if(y+offset_index >= tele_patterns[x].start)
						if(y+offset_index <= tele_patterns[x].end)
							for(i=0;i<8;i+=2)
								line[y].data[i*128 + (x+1)*30+6] = 1;

					if(y+offset_index == tele_patterns[x].i) {
						line[y].data[2*128 + (x+1)*30+6] = 11;
						line[y].data[3*128 + (x+1)*30+6] = 11;
						line[y].data[4*128 + (x+1)*30+6] = 11;
					}

				}
			}

			itoa(tele_patterns[edit_pattern].v[edit_index + offset_index], s, 10);
			font_string_region_clip_right(&line[edit_index], s, (edit_pattern+1) * 30 + 4, 0, 0xf, 0);

			for(y=0;y<64;y+=2) {
				line[y>>3].data[(y & 0x7)*128 + 8] = 1;
			}

			for(y=0;y<8;y++) {
				line[(offset_index+y)>>3].data[((offset_index+y)&0x7)*128 + 8] = 6;
			}

			r_edit_dirty &= ~R_ALL;

			sdirty++;
		}
	}
	else if(mode == M_PRESET_W) {
		if(r_edit_dirty & R_ALL) {
			
			strcpy(s,">>> ");
			itoa(preset_select,s+4,10);
			region_fill(&line[0], 1);
			font_string_region_clip_right(&line[0], s, 126, 0, 0xf, 1);
			font_string_region_clip(&line[0], "WRITE", 2, 0, 0xf, 1);


			for(y=1;y<7;y++) {
				a = preset_edit_line == (y - 1);
				region_fill(&line[y], a);
				font_string_region_clip(&line[y], scene_text[preset_edit_offset+y-1], 2, 0, 0xa + a*5, a);
			}


			s[0] = '+';
	 		s[1] = ' ';
			s[2] = 0;

			strcat(s,input);
			strcat(s," ");

			region_fill(&line[7], 0);
			// region_string(&line[7], s, 0, 0, 0xf, 0, 0);
			// font_string_region_clip(&line[7], s, 0, 0, 0xf, 0);
			font_string_region_clip_hi(&line[7], s, 0, 0, 0xf, 0, pos+2);

			r_edit_dirty &= ~R_ALL;
			sdirty++;
		}
	}
	else if(mode == M_PRESET_R) {
		if(r_edit_dirty & R_ALL) {
			itoa(preset_select,s,10);
			region_fill(&line[0], 1);
			font_string_region_clip_right(&line[0], s, 126, 0, 0xf, 1);
			font_string_region_clip(&line[0], f.s[preset_select].text[0], 2, 0, 0xf, 1);
			

			for(y=1;y<8;y++) {
				region_fill(&line[y], 0);
				font_string_region_clip(&line[y], f.s[preset_select].text[preset_edit_offset+y], 2, 0, 0xa, 0);
			}

			r_edit_dirty &= ~R_ALL;
			sdirty++;
		}
	}
	else if(mode == M_HELP) {
		if(r_edit_dirty & R_ALL) {
			for(y=0;y<8;y++) {
				region_fill(&line[y], 0);
				/// fixme: make a pointer array
				if(help_page == 0) font_string_region_clip_tab(&line[y], help1[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 1) font_string_region_clip_tab(&line[y], help2[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 2) font_string_region_clip_tab(&line[y], help3[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 3) font_string_region_clip_tab(&line[y], help4[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 4) font_string_region_clip_tab(&line[y], help5[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 5) font_string_region_clip_tab(&line[y], help6[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 6) font_string_region_clip_tab(&line[y], help7[y+offset_view], 2, 0, 0xa, 0);
				else if(help_page == 7) font_string_region_clip_tab(&line[y], help8[y+offset_view], 2, 0, 0xa, 0);
			}

			r_edit_dirty &= ~R_ALL;
			sdirty++;
		}	
	}
	else if(mode == M_LIVE || mode == M_EDIT) {
		if(r_edit_dirty & R_INPUT) {
			s[0] = '>';
	 		s[1] = ' ';
			s[2] = 0;

			if(mode == M_EDIT) {
				if(edit == 8) s[0] = 'M';
				else if(edit == 9) s[0] = 'I';
				else s[0] = edit+49;
			}

			strcat(s,input);
			strcat(s," ");

			region_fill(&line[7], 0);
			// region_string(&line[7], s, 0, 0, 0xf, 0, 0);
			// font_string_region_clip(&line[7], s, 0, 0, 0xf, 0);
			font_string_region_clip_hi(&line[7], s, 0, 0, 0xf, 0, pos+2);
			sdirty++;
			r_edit_dirty &= ~R_INPUT;
		}
		if(r_edit_dirty & R_MESSAGE) {
			if(status) {
				strcpy(s,tele_error(status));
				if(error_detail[0]) {
					strcat(s, ": ");
					strcat(s, error_detail);
					error_detail[0] = 0;
				}
				status = E_OK;
			}
			else if(output_new) {
				output_new = 0;
				if(mode == M_LIVE)
					itoa(output, s, 10);
				// strcat(s, " ");
				// strcat(s, to_v(output));
				else
					s[0] = 0;
			}
			else {
				s[0] = 0;
			}
			region_fill(&line[6], 0);
			font_string_region_clip(&line[6], s, 0, 0, 0x4, 0);
			sdirty++;
			r_edit_dirty &= ~R_MESSAGE;
		}
		if(r_edit_dirty & R_LIST) {
			if(mode == M_LIVE) {
				for(int i=0;i<6;i++)
					region_fill(&line[i], 0);
			}
			else {
				for(int i=0;i<6;i++) {
					a = edit_line == i;
					region_fill(&line[i], a);
					if(script[edit].l > i) {
						strcpy(s,print_command(&script[edit].c[i]));
						region_string(&line[i], s, 2, 0, 0xf, a, 0);
					}
				}
			}

			
			sdirty++;
			r_edit_dirty &= ~R_LIST;
		}
		
		if((activity != activity_prev) && (mode == M_LIVE)) {
			region_fill(&line[0], 0);

			if(activity & A_SLEW) a = 15;
			else a = 1;

			line[0].data[ 98 + 0 + 512 ] = a;
			line[0].data[ 98 + 1 + 384 ] = a;
			line[0].data[ 98 + 2 + 256 ] = a;
			line[0].data[ 98 + 3 + 128 ] = a;
			line[0].data[ 98 + 4 + 0 ] = a;

			if(activity & A_DELAY) a = 15;
			else a = 1;

			line[0].data[ 106 + 0 + 0 ] = a;
			line[0].data[ 106 + 1 + 0 ] = a;
			line[0].data[ 106 + 2 + 0 ] = a;
			line[0].data[ 106 + 3 + 0 ] = a;
			line[0].data[ 106 + 4 + 0 ] = a;
			line[0].data[ 106 + 0 + 128 ] = a;
			line[0].data[ 106 + 0 + 256 ] = a;
			line[0].data[ 106 + 0 + 384 ] = a;
			line[0].data[ 106 + 0 + 512 ] = a;
			line[0].data[ 106 + 4 + 128 ] = a;
			line[0].data[ 106 + 4 + 256 ] = a;
			line[0].data[ 106 + 4 + 384 ] = a;
			line[0].data[ 106 + 4 + 512 ] = a;

			if(activity & A_Q) a = 15;
			else a = 1;

			line[0].data[ 114 + 0 + 0 ] = a;
			line[0].data[ 114 + 1 + 0 ] = a;
			line[0].data[ 114 + 2 + 0 ] = a;
			line[0].data[ 114 + 3 + 0 ] = a;
			line[0].data[ 114 + 4 + 0 ] = a;
			line[0].data[ 114 + 0 + 256 ] = a;
			line[0].data[ 114 + 1 + 256 ] = a;
			line[0].data[ 114 + 2 + 256 ] = a;
			line[0].data[ 114 + 3 + 256 ] = a;
			line[0].data[ 114 + 4 + 256 ] = a;
			line[0].data[ 114 + 0 + 512 ] = a;
			line[0].data[ 114 + 1 + 512 ] = a;
			line[0].data[ 114 + 2 + 512 ] = a;
			line[0].data[ 114 + 3 + 512 ] = a;
			line[0].data[ 114 + 4 + 512 ] = a;

			if(activity & A_METRO) a = 15;
			else a = 1;

			line[0].data[ 122 + 0 + 0 ] = a;
			line[0].data[ 122 + 0 + 128 ] = a;
			line[0].data[ 122 + 0 + 256 ] = a;
			line[0].data[ 122 + 0 + 384 ] = a;
			line[0].data[ 122 + 0 + 512 ] = a;
			line[0].data[ 122 + 1 + 128 ] = a;
			line[0].data[ 122 + 2 + 256 ] = a;
			line[0].data[ 122 + 3 + 128 ] = a;
			line[0].data[ 122 + 4 + 0 ] = a;
			line[0].data[ 122 + 4 + 128 ] = a;
			line[0].data[ 122 + 4 + 256 ] = a;
			line[0].data[ 122 + 4 + 384 ] = a;
			line[0].data[ 122 + 4 + 512 ] = a;

			// mutes

			line[0].data[ 87 + 0 + 128 ] = 15 - mutes[0] * 12;
			line[0].data[ 87 + 1 + 384 ] = 15 - mutes[1] * 12;
			line[0].data[ 87 + 2 + 128 ] = 15 - mutes[2] * 12;
			line[0].data[ 87 + 3 + 384 ] = 15 - mutes[3] * 12;
			line[0].data[ 87 + 4 + 128 ] = 15 - mutes[4] * 12;
			line[0].data[ 87 + 5 + 384 ] = 15 - mutes[5] * 12;
			line[0].data[ 87 + 6 + 128 ] = 15 - mutes[6] * 12;
			line[0].data[ 87 + 7 + 384 ] = 15 - mutes[7] * 12;

			activity &= ~A_MUTES;
			activity &= ~A_REFRESH;

			activity_prev = activity;

			// activity &= ~A_X;

			sdirty++;
		}
	}
}

static void handler_II(s32 data) {
	uint8_t i = data & 0xff;
	int16_t d = (int)(data >> 16);
	uint8_t addr = i & 0xf0;
	i2c_master_tx(addr, i, d);
	// print_dbg("\r\ni2c: ");
	// print_dbg_ulong(addr);
	// print_dbg(" ");
	// print_dbg_ulong(i);
	// print_dbg(" ");
	// if(d<0)
	// 	print_dbg(" -");
	// print_dbg_ulong(d);
}




// assign event handlers
static inline void assign_main_event_handlers(void) {
	app_event_handlers[ kEventFront ] = &handler_Front;
	app_event_handlers[ kEventPollADC ]	= &handler_PollADC;
	app_event_handlers[ kEventKeyTimer ] = &handler_KeyTimer;
	app_event_handlers[ kEventSaveFlash ] = &handler_SaveFlash;
	app_event_handlers[ kEventHidConnect ] = &handler_HidConnect;
	app_event_handlers[ kEventHidDisconnect ] = &handler_HidDisconnect;
	app_event_handlers[ kEventHidPacket ] = &handler_HidPacket;
	app_event_handlers[ kEventHidTimer ] = &handler_HidTimer;
	app_event_handlers[ kEventTrigger ]	= &handler_Trigger;
	app_event_handlers[ kEventScreenRefresh ] = &handler_ScreenRefresh;
	app_event_handlers[ kEventII ] = &handler_II;
}

// app event loop
void check_events(void) {
	static event_t e;
	if( event_next(&e) ) {
		(app_event_handlers)[e.type](e.data);
	}
}



///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// funcs

void set_mode(uint8_t m) {
	switch(m) {
		case M_LIVE:
			for(int n = 0;n < 32;n++) input[n] = 0;
			pos = 0;
			mode = M_LIVE;
			flashc_memset8((void*)&(f.mode), mode, 1, true);
			edit_line = SCRIPT_MAX_COMMANDS;
			activity |= A_REFRESH;
			r_edit_dirty |= R_ALL;
			break;
		case M_EDIT:
			mode = M_EDIT;
			edit_line = 0;
			strcpy(input,print_command(&script[edit].c[edit_line]));
			pos = strlen(input);
			for(int n = pos;n < 32;n++) input[n] = 0;
			r_edit_dirty |= R_ALL;
			break;
		case M_TRACK:
			mode = M_TRACK;
			flashc_memset8((void*)&(f.mode), mode, 1, true);
			r_edit_dirty = R_ALL;
			break;
		case M_PRESET_W:
			preset_edit_line = 0;
			preset_edit_offset = 0;
			strcpy(input,scene_text[preset_edit_line + preset_edit_offset]);
			pos = strlen(input);
			mode = M_PRESET_W;
			r_edit_dirty = R_ALL;
			break;
		case M_PRESET_R:
			preset_edit_offset = 0;
			knob_last = adc[1]>>7;
			mode = M_PRESET_R;
			r_edit_dirty = R_ALL;
			break;
		case M_HELP:
			mode = M_HELP;
			r_edit_dirty = R_ALL;
			break;
	}
}


u8 flash_is_fresh(void) {
  return (f.fresh != FIRSTRUN_KEY);
	// return 0;
  // flashc_memcpy((void *)&flashy.fresh, &i, sizeof(flashy.fresh),   true);
  // flashc_memset32((void*)&(flashy.fresh), fresh_MAGIC, 4, true);
  // flashc_memset((void *)nvram_data, 0x00, 8, sizeof(*nvram_data), true);
}

// write fresh status
void flash_unfresh(void) {
  flashc_memset8((void*)&(f.fresh), FIRSTRUN_KEY, 1, true);
}

void flash_write(void) {
	// print_dbg("\r\n:::: write flash");
	// print_dbg_ulong(preset_select);

	flashc_memcpy((void *)&f.s[preset_select].script, &script, sizeof(script), true);
	flashc_memcpy((void *)&f.s[preset_select].patterns, &tele_patterns, sizeof(tele_patterns), true);
	flashc_memcpy((void *)&f.s[preset_select].text, &scene_text, sizeof(scene_text), true);
	flashc_memset8((void*)&(f.scene), preset_select, 1, true);

	// flashc_memcpy((void *)&flashy.glyph[preset_select], &glyph, sizeof(glyph), true);
	// flashc_memset8((void*)&(flashy.preset_select), preset_select, 1, true);
}

void flash_read(void) {
	// print_dbg("\r\n:::: read flash ");
	// print_dbg_ulong(preset_select);
	memcpy(&script,&f.s[preset_select].script,sizeof(script));
	memcpy(&tele_patterns,&f.s[preset_select].patterns,sizeof(tele_patterns));
	memcpy(&scene_text,&f.s[preset_select].text,sizeof(scene_text));
	flashc_memset8((void*)&(f.scene), preset_select, 1, true);
}



void render_init(void) {
 	region_alloc(&line[0]);
	region_alloc(&line[1]);
	region_alloc(&line[2]);
	region_alloc(&line[3]);
	region_alloc(&line[4]);
	region_alloc(&line[5]);
	region_alloc(&line[6]);
	region_alloc(&line[7]);
}



static void tele_metro(int16_t m, int16_t m_act, uint8_t m_reset) {
	metro_time = m;

	if(m_act && !metro_act) {
		// print_dbg("\r\nTURN ON METRO");
		metro_act = 1;
		if(script[METRO_SCRIPT].l)
			activity |= A_METRO;
		timer_add(&metroTimer, metro_time, &metroTimer_callback, NULL);
	}
	else if(!m_act && metro_act) {
		// print_dbg("\r\nTURN OFF METRO");
		metro_act = 0;
		timer_remove(&metroTimer);
	}
	else if(!m_reset) {
		// print_dbg("\r\nSET METRO");
		timer_set(&metroTimer, metro_time);
	}
	else {
		// print_dbg("\r\nRESET METRO");
		timer_reset(&metroTimer);
	}

	if(!metro_act)
		activity &= ~A_METRO;
}

static void tele_tr(uint8_t i, int16_t v) {
	if(v)
		gpio_set_pin_high(B08+i);
	else
		gpio_set_pin_low(B08+i);
}

static void tele_cv(uint8_t i, int16_t v, uint8_t s) {
	int16_t t = v + aout[i].off;
	if(t < 0)
		t = 0;
	else if(t > 16383)
		t = 16383;
	aout[i].target = t;
 	if(s) {
 		aout[i].step = aout[i].slew;
 		aout[i].delta = ((aout[i].target - aout[i].now)<<16) / aout[i].step;
 	}
 	else {
 		aout[i].step = 1;
 		aout[i].now = aout[i].target;
 	}
	
	aout[i].a = aout[i].now<<16;

	timer_trigger(&adcTimer);
}

static void tele_cv_slew(uint8_t i, int16_t v) {
	aout[i].slew = v / RATE_CV;
	if(aout[i].slew == 0)
		aout[i].slew = 1;
}

static void tele_delay(uint8_t i) {
	if(i) {
		activity |= A_DELAY;
	}
	else
		activity &= ~A_DELAY;
}

static void tele_s(uint8_t i) {
	if(i) {
		activity |= A_Q;
	}
	else
		activity &= ~A_Q;
}

static void tele_cv_off(uint8_t i, int16_t v) {
	aout[i].off = v;
}

static void tele_ii(uint8_t i, int16_t d) {
	static event_t e;
	e.type = kEventII;
	e.data = (d<<16) + i;
	event_post(&e);
}

static void tele_scene(uint8_t i) {
	preset_select = i;
	flash_read();
}

static void tele_pi() {
	if(mode == M_TRACK)
		r_edit_dirty |= R_ALL;
}

int8_t script_caller;
static void tele_script(uint8_t a) {
	if(!script_caller) {
		script_caller = a;
		for(int i=0;i<script[a-1].l;i++)
			process(&script[a-1].c[i]);
	}
	else if(a != script_caller) {
		for(int i=0;i<script[a-1].l;i++)
			process(&script[a-1].c[i]);
	}

	script_caller = 0;
}

static void tele_kill() {
	for(int i = 0;i<4;i++)
		aout[i].step = 1;
}

static void tele_mute(uint8_t i, uint8_t s) {
	mutes[i] = s;
	activity |= A_MUTES;
}




///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// usb disk

static void tele_usb_disk() {
	uint8_t usb_retry = 10;
	print_dbg("\r\nusb");
	while(usb_retry--) {
		print_dbg(".");

		if(!uhi_msc_is_available()) {
			uint8_t lun, lun_state=0;

			for (lun = 0; (lun < uhi_msc_mem_get_lun()) && (lun < 8); lun++) {
				// print_dbg("\r\nlun: ");
				// print_dbg_ulong(lun);

				// Mount drive
				nav_drive_set(lun);
				if (!nav_partition_mount()) {
					if (fs_g_status == FS_ERR_HW_NO_PRESENT) {
						// The test can not be done, if LUN is not present
						lun_state &= ~(1 << lun); // LUN test reseted
						continue;
					}
					lun_state |= (1 << lun); // LUN test is done.
					print_dbg("\r\nfail");
					// ui_test_finish(false); // Test fail
					continue;
				}
				// Check if LUN has been already tested
				if (lun_state & (1 << lun)) {
					continue;
				}

				// WRITE SCENES
				char filename[13];
				strcpy(filename,"tt00s.txt");

				print_dbg("\r\nwriting scenes");
				strcpy(input_buffer,"WRITE");
				region_fill(&line[0], 0);
				font_string_region_clip_tab(&line[0], input_buffer, 2, 0, 0xa, 0);
				region_draw(&line[0]);

				for(int i=0;i<SCENE_SLOTS;i++) {
					strcat(input_buffer,".");
					region_fill(&line[0], 0);
					font_string_region_clip_tab(&line[0], input_buffer, 2, 0, 0xa, 0);
					region_draw(&line[0]);

					memcpy(&script,&f.s[i].script,sizeof(script));
					memcpy(&tele_patterns,&f.s[i].patterns,sizeof(tele_patterns));
					memcpy(&scene_text,&f.s[i].text,sizeof(scene_text));

					if (!nav_file_create((FS_STRING) filename)) {
						if (fs_g_status != FS_ERR_FILE_EXIST) {
							if (fs_g_status == FS_LUN_WP) {
								// Test can be done only on no write protected device
								continue;
							}
							lun_state |= (1 << lun); // LUN test is done.
							print_dbg("\r\nfail");
							continue;
						}
					}
					if (!file_open(FOPEN_MODE_W)) {
						if (fs_g_status == FS_LUN_WP) {
							// Test can be done only on no write protected device
							continue;
						}
						lun_state |= (1 << lun); // LUN test is done.
						print_dbg("\r\nfail");
						continue;
					}

					char blank=0;
					for(int l=0;l<SCENE_TEXT_LINES;l++) {
						if(strlen(scene_text[l])) {
							file_write_buf((uint8_t*) scene_text[l], strlen(scene_text[l]));
							file_putc('\n');
							blank=0;
						}
						else if(!blank) { file_putc('\n'); blank=1;}
					}
					
					char input[36];
					for(int s=0;s<10;s++) {
						file_putc('\n');
						file_putc('\n');
						file_putc('#');
						if(s==8) file_putc('M');
						else if(s==9) file_putc('I');
						else file_putc(s+49);

						for(int l=0;l<script[s].l;l++) {
							file_putc('\n');
							strcpy(input,print_command(&script[s].c[l]));
							file_write_buf((uint8_t*) input,strlen(input));
						}
					}

					file_putc('\n');
					file_putc('\n');
					file_putc('#');
					file_putc('P');
					file_putc('\n');

					for(int b=0;b<4;b++) {
						itoa(tele_patterns[b].l, input, 10);
						file_write_buf((uint8_t*) input, strlen(input));
						if(b==3) file_putc('\n');
						else file_putc('\t');
					}

					for(int b=0;b<4;b++) {
						itoa(tele_patterns[b].wrap, input, 10);
						file_write_buf((uint8_t*) input, strlen(input));
						if(b==3) file_putc('\n');
						else file_putc('\t');
					}

					for(int b=0;b<4;b++) {
						itoa(tele_patterns[b].start, input, 10);
						file_write_buf((uint8_t*) input, strlen(input));
						if(b==3) file_putc('\n');
						else file_putc('\t');
					}

					for(int b=0;b<4;b++) {
						itoa(tele_patterns[b].end, input, 10);
						file_write_buf((uint8_t*) input, strlen(input));
						if(b==3) file_putc('\n');
						else file_putc('\t');
					}

					file_putc('\n');

					for(int l=0;l<64;l++) {
						for(int b=0;b<4;b++) {
							itoa(tele_patterns[b].v[l], input, 10);
							file_write_buf((uint8_t*) input, strlen(input));
							if(b==3) file_putc('\n');
							else file_putc('\t');
						}
					}

					file_close();
					lun_state |= (1 << lun); // LUN test is done.

					if(filename[3] == '9') {
						filename[3] = '0';
						filename[2]++;
					}
					else filename[3]++;

					print_dbg(".");
				}

				nav_filelist_reset();


				// READ SCENES
				strcpy(filename,"tt00.txt");
				print_dbg("\r\nreading scenes...");

				strcpy(input_buffer,"READ");
				region_fill(&line[1], 0);
				font_string_region_clip_tab(&line[1], input_buffer, 2, 0, 0xa, 0);
				region_draw(&line[1]);

				for(int i=0;i<SCENE_SLOTS;i++) {
					strcat(input_buffer,".");
					region_fill(&line[1], 0);
					font_string_region_clip_tab(&line[1], input_buffer, 2, 0, 0xa, 0);
					region_draw(&line[1]);
					if(nav_filelist_findname(filename,0)) {
						print_dbg("\r\nfound: ");
						print_dbg(filename);
						if(!file_open(FOPEN_MODE_R))
							print_dbg("\r\ncan't open");
						else {
							tele_mem_clear();

							char c;
							uint8_t l = 0;
							uint8_t p = 0;
							int8_t s = 99;
							uint8_t b = 0;
							uint16_t num = 0;
							int8_t neg = 1;

							while(!file_eof() && s != -1) {
								c = toupper(file_getc());
								// print_dbg_char(c);

								if(c == '#') {
									if(!file_eof()) {
										c = toupper(file_getc());
										// print_dbg_char(c);

										if(c == 'M')
											s = 8;
										else if(c == 'I')
											s = 9;
										else if(c == 'P')
											s = 10;
										else {
											s = c - 49;
											if(s < 0 || s > 7)
												s = -1;
										}

										l = 0;
										p = 0;

										if(!file_eof())
											c = toupper(file_getc());
									}
									else s = -1;

									// print_dbg("\r\nsection: ");
									// print_dbg_ulong(s);

								}
								// SCENE TEXT
								else if(s == 99) {
									if(c == '\n') {
										l++;
										p=0;
									}
									else {
										if(l < SCENE_TEXT_LINES && p < SCENE_TEXT_CHARS) {
											scene_text[l][p] = c;
											p++;
										}
									}
								}
								// SCRIPTS
								else if(s >= 0  && s <= 9) {
									if(c == '\n') {
										if(p && l < SCRIPT_MAX_COMMANDS) {
 											status = parse(input);

					     					if(status == E_OK) {
					     						// print_dbg("\r\nparsed: ");
					     						// print_dbg(input);
												status = validate(&temp);

												if(status == E_OK) {
													memcpy(&script[s].c[l], &temp, sizeof(tele_command_t));
													// print_dbg("\r\nvalidated: ");
													// print_dbg(print_command(&script[s].c[l]));
													memset(input,0,sizeof(input));			
													script[s].l++;
												}
												else {
													print_dbg("\r\nvalidate: ");
													print_dbg(tele_error(status));
					 							}
											}
											else {
												print_dbg("\r\nERROR: ");
												print_dbg(tele_error(status));
												print_dbg(" >> ");
												print_dbg(print_command(&script[s].c[l]));
											}

											l++;
											p = 0;
										}
									}
									else {
										if(p < 32)
											input[p] = c;
										p++;
									}
								}
								// PATTERNS
								// tele_patterns[]. l wrap start end v[64]
								else if(s == 10) {
									if(c == '\n' || c == '\t') {
										if(b < 4) {
											if(l>3) {
												tele_patterns[b].v[l-4] = neg * num;

												// print_dbg("\r\nset: ");
												// print_dbg_ulong(b);
												// print_dbg(" ");
												// print_dbg_ulong(l-4);
												// print_dbg(" ");
												// print_dbg_ulong(num);
											}
											else if(l==0) {
												tele_patterns[b].l = num;
											}
											else if(l==1) {
												tele_patterns[b].wrap = num;
											}
											else if(l==2) {
												tele_patterns[b].start = num;
											}
											else if(l==3) {
												tele_patterns[b].end = num;
											}
										}

										b++;
										num = 0;
										neg = 1;

										if(c == '\n') {
											if(p) 
												l++;
											if(l > 68)
												s = -1;
											b = 0;
											p = 0;	
										}
									}
									else {
										if(c == '-')
											neg = -1;
										else if(c >= '0' && c <= '9') {
											num = num * 10 + (c-48);
											// print_dbg("\r\nnum: ");
											// print_dbg_ulong(num);
										}
										p++;
									}
								}
							} 
							

							file_close();

							preset_select = i;
							flash_write();
						}
					}
					else
						nav_filelist_reset();

					if(filename[3] == '9') {
						filename[3] = '0';
						filename[2]++;
					}
					else filename[3]++;

					preset_select = 0;
				}
			}

			usb_retry = 0;

			nav_exit();
			region_fill(&line[0], 0);
			region_fill(&line[1], 0);
			tele_mem_clear();
		}
		delay_ms(100);
	}
}

void tele_mem_clear(void) {
	memset(&script,0,sizeof(script));
	memset(&tele_patterns,0,sizeof(tele_patterns));
	memset(&scene_text,0,sizeof(scene_text));
}

void tele_input_state(uint8_t n) {
	input_states[n] = gpio_get_pin_value(A00+n);
}




///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// main

int main(void)
{
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
	// init_monome();

	init_oled();

	init_i2c_master();

	print_dbg("\r\n\n// teletype! //////////////////////////////// ");
	print_dbg("\r\nflash size: ");
	print_dbg_ulong(sizeof(f));

	tele_init();

	if(flash_is_fresh()) {
		print_dbg("\r\n:::: first run, clearing flash");

		for(preset_select=0;preset_select<SCENE_SLOTS;preset_select++) {
			flash_write();
			print_dbg(".");
		}
		preset_select = 0;
		flashc_memset8((void*)&(f.scene), preset_select, 1, true);
		flashc_memset8((void*)&(f.mode), M_LIVE, 1, true);
		flash_unfresh();

		// clear out some reasonable defaults

		// save all presets, clear glyphs
		// for(i1=0;i1<8;i1++) {
		// 	flashc_memcpy((void *)&flashy.es[i1], &es, sizeof(es), true);
		// 	glyph[i1] = (1<<i1);
		// 	flashc_memcpy((void *)&flashy.glyph[i1], &glyph, sizeof(glyph), true);
		// }
	}
	else {
		preset_select = f.scene;
		tele_set_val(V_SCENE, preset_select);
		flash_read();
		// load from flash at startup
	}

	// screen init
	render_init();

	// usb disk check
	tele_usb_disk();

	// setup daisy chain for two dacs
	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x80);
	spi_write(SPI,0xff);
	spi_write(SPI,0xff);
	spi_unselectChip(SPI,DAC_SPI);

	timer_add(&clockTimer, RATE_CLOCK, &clockTimer_callback, NULL);
	timer_add(&cvTimer, RATE_CV, &cvTimer_callback, NULL);
	timer_add(&keyTimer, 71, &keyTimer_callback, NULL);
	timer_add(&adcTimer, 61, &adcTimer_callback, NULL);
	timer_add(&refreshTimer, 63, &refreshTimer_callback, NULL);
	
	metro_act = 1;
	metro_time = 1000;
	timer_add(&metroTimer, metro_time ,&metroTimer_callback, NULL);

	update_metro = &tele_metro;
	update_tr = &tele_tr;
	update_cv = &tele_cv;
	update_cv_slew = &tele_cv_slew;
	update_delay = &tele_delay;
	update_s = &tele_s;
	update_cv_off = &tele_cv_off;
	update_ii = &tele_ii;
	update_scene = &tele_scene;
	update_pi = &tele_pi;
	run_script = &tele_script;
	update_kill = &tele_kill;
	update_mute = &tele_mute;
	update_input = &tele_input_state;

	clear_delays();

	aout[0].slew = 1;
	aout[1].slew = 1;
	aout[2].slew = 1;
	aout[3].slew = 1;

	for(int i=0;i<8;i++)
		mutes[i] = 1;

	status = 1;
	error_detail[0] = 0;
	mode = f.mode;
	edit_line = SCRIPT_MAX_COMMANDS;
	r_edit_dirty = R_MESSAGE | R_INPUT;
	activity = 0;
	activity_prev = 0xff;

	for(int i=0;i<script[INIT_SCRIPT].l;i++)
		process(&script[INIT_SCRIPT].c[i]);

	while (true) {
		check_events();
	}
}
