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

#define CYCLE_POS_MAX UINT8_MAX

#define N_DAUGHTERS 1
const static struct {
	uint8_t offset;
} daughters[N_DAUGHTERS] = {
	{ CYCLE_POS_MAX/2 },
};

static volatile struct {
	unsigned long last_duration;
	unsigned long current;
} clock = {1, 1};

#define SR_PORT PORTB
#define SR_DATA PB1
#define SR_CLOCK PB2
#define SR_LATCH PB3

static void init(void) {
	for (int i=0; i<N_LEDS; i++) {
		set_output(DDRD, PIN[i]);
		output_high(PORTD, PIN[i]);
	}
	set_input(DDRB, PB0);
	set_output(DDRB, PB1);
	set_output(DDRB, PB2);
	set_output(DDRB, PB3);

	OCR1A = 2;
	TCCR1A = 0x00;
	// WGM1=4, prescale at 1024
	TCCR1B = (0 << WGM13)|(1 << WGM12)|(1 << CS12)|(0 << CS11)|(1 << CS10);
	//TCCR1B = (0 << WGM13)|(1 << WGM12)|(0 << CS12)|(0 << CS11)|(1 << CS10);
	//Set bit 6 in TIMSK to enable Timer 1 compare interrupt
	TIMSK |= (1 << OCIE1A);
	sei();
}

static void shift_out(uint8_t data) {
	for (int i = 0; i < 8; i++) {
		if (data & (1<<i)) {
			SR_PORT |= 1<<SR_DATA;
		} else {
			SR_PORT &= ~(1<<SR_DATA);
		}
		SR_PORT |= (1<<SR_CLOCK);
		SR_PORT &= ~(1<<SR_CLOCK);
	}
}

static void trigger_latch(void) {
	SR_PORT |= (1<<SR_LATCH);
	SR_PORT &= ~(1<<SR_LATCH);
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
	shift_out(~0);
	trigger_latch();
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

#define min(a,b) ((a)>(b)?(b):(a))
static uint8_t cycle_position(void) {
	return min( CYCLE_POS_MAX, (CYCLE_POS_MAX*clock.current/clock.last_duration));
}

#define N_SPOKES 3
#define SEGMENT_WIDTH ((CYCLE_POS_MAX+1)/N_SPOKES)
#define W_SPOKES 50.0

#define GLOBAL_OFFSET ((int)(-0.26*(CYCLE_POS_MAX+1)))

static uint8_t get_content(uint8_t pos) {
	if (pos%(SEGMENT_WIDTH) < (W_SPOKES/100.0)*(SEGMENT_WIDTH)) {
		// this area will be painted
		double s_percent = 1.0*(pos%SEGMENT_WIDTH)/((W_SPOKES/100.0)*(SEGMENT_WIDTH));
		uint8_t row = (int)(s_percent*IMG_ROWS);
		return arrow[row];
	} else {
		return 0;
	}
}

int main(void) {
	init();
	while(1) {
		uint8_t p = (cycle_position() + GLOBAL_OFFSET)%(CYCLE_POS_MAX+1);
		// handle any daughter boards
		for (uint8_t i=0; i<N_DAUGHTERS; i++) {
			uint8_t content = get_content((p+daughters[i].offset)%(CYCLE_POS_MAX+1));
			shift_out(~content);
		}
		// handle the main board
		uint8_t content = get_content(p);
		PORTD = ~content;
		// activate the daughter boards
		trigger_latch();
	}
	return 0;
}
