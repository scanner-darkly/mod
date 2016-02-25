#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_
#define AVERAGING_TAPS 3

// global count of uptime, and overflow flag.
volatile u64 tcTicks;
volatile u8 tcOverflow;

extern volatile u8 clock_external;

volatile u64 last_external_ticks;
volatile u32 external_clock_pulse_width;
volatile u32 external_taps[AVERAGING_TAPS];
volatile u8 external_taps_index, external_taps_count;

typedef void(*clock_pulse_t)(u8 phase);
extern volatile clock_pulse_t clock_pulse;

extern void register_interrupts(void);

void clock_null(u8);

u32 get_external_clock_average(void);

#endif