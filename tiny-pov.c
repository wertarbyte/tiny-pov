#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

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
} clock = {1, 1};

static void init(void) {
	for (int i=0; i<N_LEDS; i++) {
		set_output(DDRD, PIN[i]);
		output_high(PORTD, PIN[i]);
	}
	set_input(DDRB, PB0);

	OCR1A = 2;
	TCCR1A = 0x00;
	// WGM1=4, prescale at 1024
	TCCR1B = (0 << WGM13)|(1 << WGM12)|(1 << CS12)|(0 << CS11)|(1 << CS10);
	//TCCR1B = (0 << WGM13)|(1 << WGM12)|(0 << CS12)|(0 << CS11)|(1 << CS10);
	//Set bit 6 in TIMSK to enable Timer 1 compare interrupt
	TIMSK |= (1 << OCIE1A);

	sei();
}

static void cycle_finished(void) {
	clock.last_duration = clock.current;
	clock.current = 0;
}

static volatile uint8_t trigger_state = 0;

SIGNAL(SIG_TIMER1_COMPA) {
	uint8_t temp = PINB & (1<<PB0);
	if ( temp && !trigger_state && clock.current > 20) {
		cycle_finished();
	} else {
		clock.current++;
	}
	trigger_state = temp;

}

#define CYCLE_POS_MAX 255
#define min(a,b) ((a)>(b)?(b):(a))
static uint8_t cycle_position(void) {
	return min( CYCLE_POS_MAX, (CYCLE_POS_MAX*clock.current/clock.last_duration));
}

int main(void) {
	init();
	while(1) {
		uint8_t p = cycle_position();
		if (p < CYCLE_POS_MAX/4 || (p > CYCLE_POS_MAX/2 && p < 3*CYCLE_POS_MAX/4) ) {
			PORTD = 0;
		} else {
			PORTD = ~0;
		}
	}
	return 0;
}
