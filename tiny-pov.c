#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

#define N_LEDS 6

const static uint8_t PIN[N_LEDS] = {
	PD0,
	PD1,
	PD2,
	PD3,
	PD4,
	PD5,
};

static volatile struct {
	unsigned long last_duration;
	unsigned long current;
} clock = {1024L, 1};

static void init(void) {
	for (int i=0; i<N_LEDS; i++) {
		set_output(DDRD, PIN[i]);
		output_high(PORTD, PIN[i]);
	}

	OCR1A = 2;
	TCCR1A = 0x00;
	// WGM1=4, prescale at 1024
	// TCCR1B = (0 << WGM13)|(1 << WGM12)|(1 << CS12)|(0 << CS11)|(1 << CS10);
	TCCR1B = (0 << WGM13)|(1 << WGM12)|(0 << CS12)|(0 << CS11)|(1 << CS10);
	//Set bit 6 in TIMSK to enable Timer 1 compare interrupt
	TIMSK |= (1 << OCIE1A);
	sei();
}

SIGNAL(SIG_TIMER1_COMPA) {
	clock.current++;
}

static void cycle_finished(void) {
	clock.last_duration = clock.current;
	clock.current -= clock.last_duration;
}

static double cycle_percent(void) {
	return 100.0*clock.current/(double)clock.last_duration;
}

int main(void) {
	init();
	while(1) {
		double p = cycle_percent();
		if ( (p>=0 && p<=25) || (p>=50 && p<=75) ) {
			PORTD = 0;
		} else {
			PORTD = ~(0);
		}
		// We should query some kind of hall sensor for this
		if (p > 100) {
			cycle_finished();
		}
	}
	return 0;
}
