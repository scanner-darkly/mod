#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include <inttypes.h>

#include "teletype.h"
#include "../../system/util.h"


void tele_metro(int16_t m, int16_t m_act, uint8_t m_reset) {
	printf("METRO  m:%" PRIi16 " m_act:%" PRIi16 "m_reset:%" PRIu8, m, m_act, m_reset);
	printf("\n");
}

void tele_tr(uint8_t i, int16_t v) {
	printf("TR  i:%" PRIu8 " v:%" PRId16, i, v);
	printf("\n");
}

void tele_cv(uint8_t i, int16_t v, uint8_t s) {
	printf("CV  i:%" PRIu8 " v:%" PRId16 " s:%" PRIu8, i, v, s);
	printf("\n");
}

void tele_cv_slew(uint8_t i, int16_t v) {
	printf("CV_SLEW  i:%" PRIu8 " v:%" PRId16, i, v);
	printf("\n");
}

void tele_delay(uint8_t i) {
	printf("DELAY  i:%" PRIu8, i);
	printf("\n");
}

void tele_s(uint8_t i) {
	printf("S  i:%" PRIu8, i);
	printf("\n");
}

void tele_cv_off(uint8_t i, int16_t v) {
	printf("CV_OFF  i:%" PRIu8 " v:%" PRId16, i, v);
	printf("\n");
}

void tele_ii(uint8_t i, int16_t d) {
	printf("II  i:%" PRIu8 " d:%" PRId16, i, d);
	printf("\n");
}

void tele_scene(uint8_t i) {
	printf("SCENE  i:%" PRIu8, i);
	printf("\n");
}

void tele_pi() {
	printf("PI");
	printf("\n");
}
void tele_script(uint8_t a) {
	printf("SCRIPT  a:%" PRIu8, a);
	printf("\n");
}

void tele_kill() {
	printf("KILL");
	printf("\n");
}

void tele_mute(uint8_t i, uint8_t s) {
	printf("MUTE  i:%" PRIu8 " s:%" PRIu8, i, s);
	printf("\n");
}

void tele_input_state(uint8_t n) {
	printf("INPUT_STATE  n:%" PRIu8, n);
	printf("\n");
}

int main() {
	char *in;
	time_t t;
	error_t status;
	int i;

	srand((unsigned) time(&t));

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

	// tele_command_t stored;
	// stored.data[0].t = OP;
	// stored.data[0].v = 2;
	// stored.data[1].t = NUMBER;
	// stored.data[1].v = 8;
	// stored.data[2].t = NUMBER;
	// stored.data[2].v = 10;
	// stored.separator = -1;
	// stored.l = 3;
	// printf("\nstored process: ");
	// process(&stored);

	in = malloc(256);

	printf("teletype. (blank line quits)\n\n");

	do {
		printf("> ");
		fgets(in, 256, stdin);

		i = 0;
		while(in[i]) {
			in[i] = toupper(in[i]);
			i++;
		}

		status = parse(in);
		if(status == E_OK) {
			status = validate(&temp);
			printf("validate: %s", tele_error(status));
			if(error_detail[0])
				printf(": %s",error_detail);
			error_detail[0] = 0;
			printf("\n");
			if(status == E_OK)
				process(&temp);
		}
		else {
			printf("ERROR: %s", tele_error(status));
			if(error_detail[0])
				printf(": %s",error_detail);
			error_detail[0] = 0;
			printf("\n");
		}

		// tele_tick(100);
		printf("\n");
	}
	while(in[0] != 10);

	free(in);

	printf("(teletype exit.)\n");
}

