#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
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

static void slumber(void) {
	// disable timer interrupt
	TIMSK &= ~(1 << OCIE1A);
	// make sure we are not interrupted anymore
	_delay_ms(10);
	// turn off the lights
	PORTD = ~0;
	// enable PCINT0
	PCMSK |= (1<<PCINT0);
	GIMSK |= (1<<PCIE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	sleep_mode();
	// restore interrupt configuration and reset clock
	GIMSK &= ~(1<<PCIE);
	clock.current = 0;
	clock.last_duration = 1;
	TIMSK |= 1 << OCIE1A;
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
	if (clock.current > 600) {
		slumber();
	}
}

SIGNAL (SIG_PCINT) {}

#ifdef ARROW
#define IMG_ROWS 13
static const uint8_t arrow[IMG_ROWS] = {
	0b000000,
	0b001100,
	0b001100,
	0b001100,
	0b001100,
	0b001100,
	0b111111,
	0b111111,
	0b011110,
	0b011110,
	0b001100,
	0b000000,
	0b000000
};
#else
#define IMG_ROWS 1
static const uint8_t arrow[IMG_ROWS] = {
	~0,
};
#endif

#define CYCLE_POS_MAX UINT8_MAX
#define min(a,b) ((a)>(b)?(b):(a))
static uint8_t cycle_position(void) {
	return min( CYCLE_POS_MAX, (CYCLE_POS_MAX*clock.current/clock.last_duration));
}

#define N_SPOKES 3
#define SEGMENT_WIDTH ((CYCLE_POS_MAX+1)/N_SPOKES)
#define W_SPOKES 50.0

#define OFFSET ((int)(-0.26*(CYCLE_POS_MAX+1)))

int main(void) {
	init();
	while(1) {
		uint8_t p = (cycle_position() + OFFSET)%(CYCLE_POS_MAX+1);
		if (p%(SEGMENT_WIDTH) < (W_SPOKES/100.0)*(SEGMENT_WIDTH)) {
			// this area will be painted
			double s_percent = 1.0*(p%SEGMENT_WIDTH)/((W_SPOKES/100.0)*(SEGMENT_WIDTH));
			uint8_t row = (int)(s_percent*IMG_ROWS);
			PORTD = ~(arrow[row]);
		} else {
			PORTD = ~0;
		}
	}
	return 0;
}
