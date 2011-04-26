/*
 * tiny-pov by Stefan Tomanek <stefan@pico.ruhr.de>
 *
 * A simple POV system illuminating your bicycle wheel.
 *
 * 6 LED pixel arranged along a spoke are directly controlled by the I/O pins
 * of an ATTiny2313 (using transistors), while additional daughter board can be
 * attached to the system to generate a better POV effect even at lower speed.
 * Those extra boards each contain a 74HC595 shift register to control their
 * LEDs.
 *
 * A simple reed contact is triggered by a magnet fixed to the bicycle frame:
 * By measuring the time between these passes, the controller can estimate the
 * position of the LED spoke at any time and enable the appropiate LEDs.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <math.h>

/*
 * A full revolution is divided into 256 (2^8) segments,
 * ranging from 0 to 255 (CYCLE_POS_MAX)
 */
#define CYCLE_POS_MAX UINT8_MAX
#define CYCLE_POS_CNT (CYCLE_POS_MAX+1)

#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

#define ELEMS(x) (sizeof(x)/sizeof((x)[0]))

#define LED_DDR DDRD
#define LED_PORT PORTD
const static uint8_t LED[] = {
	PD0,
	PD1,
	PD2,
	PD3,
	PD4,
	PD5,
};

/*
 *  Pins controlling the shift register
 *  of the daughter boards
 */
#define SR_PORT PORTB
#define SR_DDR DDRB
#define SR_DATA PB1
#define SR_CLOCK PB2
#define SR_LATCH PB3

/*
 *  Reed contact triggered every turn
 *  by magnet at bicycle frame
 */
#define REED_PORT PORTB
#define REED_DDR DDRB
#define REED_PIN PB0

const static struct {
	uint8_t offset;
	uint8_t shift;
} daughters[] = {
	/*
	 * The main board, not using a shift register
	 */
	{   0, 0 },
	/*
	 * A single daugher board installed ~ 180°
	 * next to the main device
	 */
	{ 138, 1 },
};

/*
 *  Clock storing the duration of the last wheel rotation
 *  and the current number of ticks
 */
enum clockstate {
	INVALID,
	PENDING, // first contact has been made, waiting for second one
	VALID
};

static volatile struct {
	enum clockstate state;
	unsigned long last_duration;
	unsigned long current;
} clock = {INVALID, 1, 1};

static void init(void) {
	for (int i=0; i<ELEMS(LED); i++) {
		set_output(LED_DDR, LED[i]);
	}
	LED_PORT = ~0;
	set_input(REED_DDR, REED_PIN);
	set_output(SR_DDR, SR_DATA);
	set_output(SR_DDR, SR_CLOCK);
	set_output(SR_DDR, SR_LATCH);

	OCR1A = 2;
	TCCR1A = 0x00;
	// WGM1=4, prescale at 1024
	TCCR1B = (0 << WGM13)|(1 << WGM12)|(1 << CS12)|(0 << CS11)|(1 << CS10);
	//Set bit 6 in TIMSK to enable Timer 1 compare interrupt
	TIMSK |= (1 << OCIE1A);
	sei();
}

/*
 *  Shift data to the daughter boards.
 */
static void shift_out(uint8_t data) {
	/*
	 *  QA is not connected, while
	 *  QB controls the innermost LED
	 */
	for (int i = 0; i < 8; i++) {
		if ((data<<1) & (1<<i)) {
			SR_PORT |= 1<<SR_DATA;
		} else {
			SR_PORT &= ~(1<<SR_DATA);
		}
		SR_PORT |= (1<<SR_CLOCK);
		SR_PORT &= ~(1<<SR_CLOCK);
	}
}

/*
 *  Make daughter boards display new data.
 */
static void trigger_latch(void) {
	SR_PORT |= (1<<SR_LATCH);
	SR_PORT &= ~(1<<SR_LATCH);
}


/*
 *  Called whenever a full revoolution is completed:
 *  Save the duration of the last roundtrip and
 *  reset the clock counter.
 */
static void cycle_finished(void) {
	switch(clock.state) {
		case INVALID:
			clock.state = PENDING;
			break;
		case PENDING:
			clock.state = VALID;
		case VALID:
			clock.last_duration = clock.current;
	}
	clock.current = 0;
}

/*
 *  Put the system to sleep after turning all lights off.
 */
static void slumber(void) {
	// disable timer interrupt
	TIMSK &= ~(1 << OCIE1A);
	// make sure we are not interrupted anymore
	_delay_ms(10);
	// turn off the lights
	LED_PORT = ~0;
	shift_out(~0);
	trigger_latch();
	// enable PCINT0
	PCMSK |= (1<<PCINT0);
	GIMSK |= (1<<PCIE);
	// invalidate clock
	clock.state = INVALID;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	sleep_mode();
	// restore interrupt configuration and reset clock
	GIMSK &= ~(1<<PCIE);
	TIMSK |= 1 << OCIE1A;
}

static volatile uint8_t trigger_state = 0;

SIGNAL(SIG_TIMER1_COMPA) {
	// Check the reed contact
	uint8_t temp = PINB & (1<<PB0);
	if ( temp && !trigger_state && clock.current > 20) {
		cycle_finished();
	} else {
		clock.current++;
	}
	trigger_state = temp;
	/*
	 * If we haven't seen a full revolution after 600
	 * ticks, the bike is probably standing - time to
	 * power down.
	 */
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

/*
 *  Estimate the current position in the wheel cycle;
 *  If a full cycle _should_ already have been completed, the funtion
 *  returns the maximum value.
 */
#define min(a,b) ((a)>(b)?(b):(a))
static uint8_t cycle_position(void) {
	return min( CYCLE_POS_MAX, (CYCLE_POS_MAX*clock.current/clock.last_duration));
}

/*
 *  We can divide the wheel into several spokes which are
 *  "standing still". The width of the spokes is controlled
 *  by the percentage value of W_SPOKES
 *
 *  To completey fill the wheel, set N_SPOKES to 1
 *  and W_SPOKES to 100.0.
 */
#define N_SPOKES 3
#define SEGMENT_WIDTH (CYCLE_POS_CNT/N_SPOKES)
#define W_SPOKES 50.0

/*
 * A global offset, depending on the position of your frame magnet.
 */
#define GLOBAL_OFFSET ((int)(-0.26*CYCLE_POS_CNT))

/*
 *  Calculate the LED content at a specific position
 */
static uint8_t get_content(uint8_t pos) {
	// Check whether we are in an area to be painted
	if (pos%(SEGMENT_WIDTH) < (W_SPOKES/100.0)*(SEGMENT_WIDTH)) {
		// the position within our virtual spoke
		double s_percent = 1.0*(pos%SEGMENT_WIDTH)/((W_SPOKES/100.0)*(SEGMENT_WIDTH));
		uint8_t row = (int)(s_percent*IMG_ROWS);
		return arrow[row];
	} else {
		// emptyness
		return 0;
	}
}

int main(void) {
	init();
	while(1) {
		// only display something with a valid clock content
		uint8_t on = (clock.state == VALID);
		uint8_t p = on ? (cycle_position() + GLOBAL_OFFSET)%(CYCLE_POS_CNT) : 0;
		// handle any daughter boards
		for (uint8_t i=0; i<ELEMS(daughters); i++) {
			uint8_t content = on ? get_content((p+daughters[i].offset)%CYCLE_POS_CNT) : 0;
			// invert the bitmask since LED are activated on LOW ports.
			if (daughters[i].shift) {
				shift_out(~content);
			} else {
				// directly connected
				LED_PORT = ~content;
			}
		}
		// activate the daughter board LEDs
		trigger_latch();
	}
	return 0;
}
