#ifndef _TELETYPE_H_
#define _TELETYPE_H_

#include <stdint.h>

#define SCRIPT_MAX_COMMANDS 6
#define SCRIPT_MAX_COMMANDS_ 5
#define COMMAND_MAX_LENGTH 12
#define STACK_SIZE 8
#define TELE_STACK_SIZE 8
#define TELE_D_SIZE 8

#define WELCOME "TELETYPE 1.11"

#define TRUE 1
#define FALSE 0

enum varnames {V_I, V_TIME, V_TIME_ACT, V_IN, V_PARAM, V_PRESET, V_M, V_M_ACT,
	V_X, V_Y, V_Z, V_T, V_A, V_B, V_C, V_D, V_O, V_DRUNK, V_Q, V_Q_N, V_Q_AVG, V_SCENE, V_P_N,
	V_P_L, V_P_I, V_P_HERE, V_P_NEXT, V_P_PREV, V_P_WRAP, V_P_START, V_P_END, V_FLIP, 
	V_O_MIN, V_O_MAX, V_O_WRAP, V_O_DIR, V_DRUNK_MIN, V_DRUNK_MAX, V_DRUNK_WRAP };

typedef enum { 
	E_OK,
	E_WELCOME,
	E_PARSE, 
	E_LENGTH,
	E_NEED_PARAMS,
	E_EXTRA_PARAMS,
	E_NO_MOD_HERE,
	E_MANY_SEP,
	E_NEED_SEP,
	E_PLACE_SEP,
	E_NOT_LEFT
} error_t;

typedef enum {NUMBER, MOD, SEP, OP, VAR, ARRAY, KEY} tele_word_t;

typedef struct {
	tele_word_t t;
	int16_t v;
} tele_data_t;

typedef struct {
	uint8_t l;
	signed char separator;
	tele_data_t data[COMMAND_MAX_LENGTH];
} tele_command_t;

typedef struct {
	uint8_t l;
	tele_command_t c[SCRIPT_MAX_COMMANDS];
} tele_script_t;

typedef struct {
	const char *name;
	void (*func)(uint8_t);
	int16_t v;
	tele_word_t t;
} tele_var_t;

typedef struct {
	const char *name;
	int16_t v;
} tele_key_t;

typedef struct {
	const char *name;
	int16_t v[4];
	void (*func)(uint8_t);
	tele_word_t t[4];
} tele_array_t;

typedef struct {
	const char *name;
	void (*func)(tele_command_t *c);
	char params;
	const char *doc;
} tele_mod_t;

typedef struct {
	const char *name;
	void (*func)(void);
	char params;
	int8_t returns;
	const char* doc;
} tele_op_t;

typedef struct {
	int16_t i;
	uint16_t l;
	uint16_t wrap;
	int16_t start, end;
	int16_t v[64];
} tele_pattern_t;


error_t parse(char *cmd);
error_t validate(tele_command_t *c);
void process(tele_command_t *c);
char * print_command(const tele_command_t *c);

void tele_tick(uint8_t);

void clear_delays(void);

void tele_init(void);

int16_t tele_get_array(uint8_t a, uint8_t i);
void tele_set_array(uint8_t a, uint8_t i, uint16_t v);
void tele_set_val(uint8_t i, uint16_t v);

const char * tele_error(error_t);
const char * to_v(int16_t);

extern tele_command_t temp;

extern tele_pattern_t tele_patterns[4];

typedef void(*update_metro_t)(int16_t, int16_t, uint8_t);
extern volatile update_metro_t update_metro;

typedef void (*update_tr_t)(uint8_t, int16_t);
extern volatile update_tr_t update_tr;

typedef void (*update_cv_t)(uint8_t, int16_t, uint8_t);
extern volatile update_cv_t update_cv;

typedef void (*update_cv_slew_t)(uint8_t, int16_t);
extern volatile update_cv_slew_t update_cv_slew;

typedef void (*update_delay_t)(uint8_t);
extern volatile update_delay_t update_delay;

typedef void (*update_s_t)(uint8_t);
extern volatile update_s_t update_s;

typedef void (*update_cv_off_t)(uint8_t, int16_t v);
extern volatile update_cv_off_t update_cv_off;

typedef void (*update_ii_t)(uint8_t, int16_t);
extern volatile update_ii_t update_ii;

typedef void (*update_scene_t)(uint8_t);
extern volatile update_scene_t update_scene;

typedef void (*update_pi_t)(void);
extern volatile update_pi_t update_pi;

typedef void (*run_script_t)(uint8_t);
extern volatile run_script_t run_script;

typedef void (*update_kill_t)(void);
extern volatile update_kill_t update_kill;

typedef void (*update_mute_t)(uint8_t, uint8_t);
extern volatile update_mute_t update_mute;

typedef void (*update_input_t)(uint8_t);
extern volatile update_input_t update_input;

extern char error_detail[16];
extern int16_t output, output_new;

extern volatile uint8_t input_states[8];

#endif
