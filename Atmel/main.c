/*
 * ATtiny841_StepperTesting.c
 *
 * Created: 11/17/2015 10:50:51 AM
 * Author : A. Uomoto
 
READING FUSES
Reading the fuses for an unmodified ATtiny841:

	avrdude -c usbtiny -p attiny841 -U lfuse:r:-:h

gives:

	lfuse: 0x42
	hfuse: 0xdf
	efuse: 0xff

The default lfuse value in the datasheet (p220) is 0x62, not 0x42.
Bit 5, which is not used, is programmed (value = 0) on my device
but the datasheet says it's not (value = 1).

SET CLOCK SPEED TO 8 MHz
The factory default divides the internal clock by 8, resulting in
a 1 MHz clock speed. I changed this so that the chip runs at 8 MHz
(no clock division) by changing lfuse to 0xc2.

	lfuse: 0xc2
	hfuse: 0xdf
	efuse: 0xff

SET CLOCK SPEED TO 14.7456 MHz
To use a crystal, change the CKSEL[3:0] bits (p 220 & p 26) to
111X (bottom 4 bits of lfuse) so lfuse becomes 0xce:

	avrdude -c usbtiny -p attiny841 -U lfuse:w:0xce:m

	lfuse: 0xce
	hfuse: 0xdf
	efuse: 0xff

PINS
	The ATtiny841 has 12 pins (but we use one for RESET).
	My ATtiny841 basic board has a header that mates with
	an FTDI serial I/O connection (J1).

PIN CONFLICTS
	PA4 is SCK on the ISP
	PA5 is MISO on the ISP
	PA6 is MOSI on the ISP
	PB2 has an LED
	PB3 is the RESET pin

 */ 

#define	F_CPU	14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define LED_DDR		DDRB
#define LED_PORT	PORTB
#define LED_PIN		PORTB2

#define DIR_DDR		DDRA
#define DIR_PORT	PORTA
#define DIR_PIN		PORTA0

#define STEP_DDR	DDRA
#define STEP_PORT	PORTA
#define STEP_PIN	PORTA3

#define SLEEP_DDR	DDRA
#define SLEEP_PORT	PORTA
#define SLEEP_PIN	PORTA7

#define FLIMIT_DDR	DDRA
#define FLIMIT_PORT	PORTA
#define FLIMIT_PCI	PCINT4
#define FLIMIT_PUE	PINA4
#define FLIMIT_PIN	PINA4

#define HOME_DDR	DDRA
#define HOME_PORT	PORTA
#define HOME_PCI	PCINT5
#define HOME_PUE	PINA5
#define HOME_PIN	PINA5

#define RLIMIT_DDR	DDRA
#define RLIMIT_PORT	PORTA
#define RLIMIT_PCI	PCINT6
#define RLIMIT_PUE	PINA6
#define RLIMIT_PIN	PINA6

#define LIMITMASK	0b01110000
#define FORWARD		1
#define REVERSE		0

// Serial I/O
#define BAUDRATE	9600
#define MYUBRR		((F_CPU / 16 / BAUDRATE) - 1)
#define TX0READY	(UCSR0A & (1 << UDRE0))	// The UDRE0 bit in register UCSR0A is set when the TX buffer is clear
#define TX1READY	(UCSR1A & (1 << UDRE1))
#define CHARSENT	(UCSR0A & (1<<RXC0))	// The RXC0 bit in register UCSR0A is set when a char is available

#define PRINTLN(STRING)	serial0SendStr(strcpy_P(strbuf, STRING))	// strcpy_P is for moving PROGMEM strings

// Function Prototypes
void go(void);
void flashLED(void);
void initialize(void);
void printCmdList(void);
void printStatus(void);
void serial0SendByte(uint8_t);
void serial0SendCRLF(void);
void serial0SendNum(uint16_t);
uint8_t serial0RecvByte(void);
int16_t serial0RecvNum(void);
void serial0SendSignedNum(int16_t);
void serial0SendStr(char *str);
void stepMotor(void);
void stopMotor(void);

// Globals
volatile uint8_t direction;			// Motor direction
volatile uint16_t f_timer;			// Timer frequency
volatile uint16_t stepsPerSec;		// Steps per second
volatile uint16_t ticksPerStep;		// Clock ticks per step
volatile uint16_t nsteps;			// Number of steps to move (will become steps Requested)
volatile uint8_t oldpina;			// Old value of PINA register
volatile uint8_t newpina;			// New value of PINA register
volatile uint8_t bufpina;			// Temporary
volatile int16_t position;			// Current position in steps
volatile int16_t home;				// Home position
volatile uint16_t stepsDone;		// Count current steps
volatile uint16_t stepsRequested;	// Compare with this to stop motion

char strbuf[48];

const char str00[] PROGMEM = "\n\rCommands\n\r";
const char str01[] PROGMEM = "irection: forward or reverse <f|r>: ";
const char str02[] PROGMEM = "\tdirection = ";
const char str03[] PROGMEM = "\td - motor direction, set to (f)orward or (r)everse\n\r";
const char str04[] PROGMEM = "\n\rStatus\n\r";
const char str05[] PROGMEM = "Stepper Motor Testing 2015-11-26\n\r";
const char str06[] PROGMEM = "\t<spacebar> - Stop motor motion\n\r";
const char str07[] PROGMEM = "\tS - print status\n\r";
const char str08[] PROGMEM = "teps per second: ";
const char str09[] PROGMEM = "\tsteps per second = ";
const char str10[] PROGMEM = "\tticks (17.36 us each) per step = ";
const char str11[] PROGMEM = "\ts - steps per second (from 1 to 400)\n\r";
const char str12[] PROGMEM = "steps: ";
const char str13[] PROGMEM = "\tnsteps requested = ";
const char str14[] PROGMEM = "\tsteps done = ";
const char str15[] PROGMEM = "\r\nError: steps per second is 0; no motion";
const char str16[] PROGMEM = "urrent adjust -- set max current now, hit any key to exit";
const char str17[] PROGMEM = "\tg - go (start motion)\n\r";
const char str18[] PROGMEM = "\tn - number of steps\n\r";
const char str19[] PROGMEM = " - unknown command";
const char str20[] PROGMEM = "\tC - Current measurement mode\n\r";
const char str21[] PROGMEM = "\tposition = ";
const char str22[] PROGMEM = "\thome = ";

int main(void)
{

	char cmd, temp;

    initialize();

    for (;;) {
		if (CHARSENT) {
			cmd = serial0RecvByte();
			serial0SendByte(cmd);
			switch (cmd) {

				case ('\r'):
					serial0SendCRLF();
					break;

				case (' '):
					stopMotor();
					serial0SendCRLF();
					break;

				case ('C'):
					PRINTLN(str16);
					SLEEP_PORT |= _BV(SLEEP_PIN);
					while (!CHARSENT) {
					}
					temp = UDR0;
					SLEEP_PORT &= ~_BV(SLEEP_PIN);
					serial0SendCRLF();
					break;

				case ('d'):
					PRINTLN(str01);
					temp = serial0RecvByte();
					serial0SendByte(temp);
					if (temp == 'f') {
						direction = FORWARD;
						DIR_PORT |= _BV(DIR_PIN);
						LED_PORT |= _BV(LED_PIN);
					} else if (temp == 'r') {
						direction = REVERSE;
						DIR_PORT &= ~_BV(DIR_PIN);
						LED_PORT &= ~_BV(LED_PIN);
					} else {
						serial0SendByte('?');
					}
					serial0SendCRLF();
					break;

				case ('g'):
					if (stepsPerSec) {
						go();
					} else {
						PRINTLN(str15);
					}
					serial0SendCRLF();
					break;

				case ('n'):
					PRINTLN(str12);
					nsteps = serial0RecvNum();
					stepsDone = 0;
					break;

				case ('s'):
					PRINTLN(str08);
					stepsPerSec = serial0RecvNum();
					ticksPerStep = 57600 / stepsPerSec;
					OCR1A = ticksPerStep;
					break;

				case ('S'):
					printStatus();
				break;

				default:
					PRINTLN(str19);
					printCmdList();
					break;
			}
			serial0SendByte('>');
		}
	}
}

void flashLED(void)
{
	
	LED_PORT ^= _BV(LED_PIN);
	_delay_us(300);
	_delay_ms(1);
	LED_PORT ^= _BV(LED_PIN);
	_delay_us(300);
	_delay_ms(5);
	stepsDone++;

}

void go(void)
{

	// Turn off counter
	TCCR1B = 0b00000000;
	// Turn off interrupts
	cli();

	// Set up motion parameters
	stepsRequested = nsteps;
	stepsDone = 0;
	OCR1A = ticksPerStep;

	// Apply home position current
	SLEEP_PORT |= _BV(SLEEP_PIN);
	_delay_ms(10);

	// Enable interrupts
	sei();

	// Turn on counter
	TCCR1B = 0b00001100;				// Start timer in CTC mode & clock prescaler set to 256

}

void initialize(void)
{

	// Set these pins as output
	LED_DDR |= _BV(LED_PIN);
	DIR_DDR |= _BV(DIR_PIN);
	STEP_DDR |= _BV(STEP_PIN);
	SLEEP_DDR |= _BV(SLEEP_PIN);

	// Start with forward motion
	direction = FORWARD;
	DIR_PORT |= _BV(DIR_PIN);
	LED_PORT |= _BV(LED_PIN);

	// Start in sleep mode
	SLEEP_PORT &= ~_BV(SLEEP_PIN);

	// Setup USART serial0
	UBRR0H = (uint8_t) (MYUBRR >> 8);	// Baud rate
	UBRR0L = (uint8_t) MYUBRR;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);				// 8 bits, no parity, one stop bit

	// Counter1 setup
	TIMSK1 = 0b00000010;				// Output compare A match interrupt (p 119)
	TIMSK1 = (1 << OCIE1A);				// Enable CTC interrupt
	f_timer = (uint32_t) F_CPU / 256;	// Note hardwired prescaler value
	
	GIMSK = _BV(PCIE0);											// Pin Change Interrupt Enable 0
	PCMSK0 = _BV(FLIMIT_PCI) | _BV(HOME_PCI) | _BV(RLIMIT_PCI);	// Pin Change Interrupt mask
	PUEA = _BV(FLIMIT_PUE) | _BV(HOME_PUE) | _BV(RLIMIT_PUE);	// Pullups on Pin Change Interrupt pins

	stepsPerSec = 0;
	ticksPerStep = f_timer / stepsPerSec;
	stepsRequested = 0;
	stepsDone = 0;
	nsteps = 0;
	position = 0;
	home = 0;
	oldpina = PINA & LIMITMASK;

	PRINTLN(str05);
	serial0SendByte('>');

}

void printCmdList(void)
{

	PRINTLN(str00);
	PRINTLN(str20);
	PRINTLN(str17);
	PRINTLN(str03);
	PRINTLN(str18);
	PRINTLN(str11);
	PRINTLN(str07);
	PRINTLN(str06);
	serial0SendCRLF();

}

void printStatus(void)
{

	char c;

	// "Status:"
	PRINTLN(str04);

	// "position"
	PRINTLN(str21);
	serial0SendSignedNum(position);
	serial0SendCRLF();
	
	// "home"
	PRINTLN(str22);
	serial0SendSignedNum(home);
	serial0SendCRLF();

	// "nsteps requested = "
	PRINTLN(str13);
	serial0SendNum(nsteps);
	serial0SendCRLF();

	// "steps per second = "
	PRINTLN(str09);
	serial0SendNum(stepsPerSec);
	serial0SendCRLF();

	// "direction = "
	PRINTLN(str02);
	if (direction == FORWARD) {
		c = 'f';
	} else if (direction == REVERSE) {
		c = 'r';
	} else {
		c = '?';
	}
	serial0SendByte(c);
	serial0SendCRLF();

	// "ticks per step = "
	PRINTLN(str10);
	serial0SendNum(ticksPerStep);
	serial0SendCRLF();

	// "steps done = "
	PRINTLN(str14);
	serial0SendNum(stepsDone);
	serial0SendCRLF();
	serial0SendCRLF();

}

uint8_t serial0RecvByte(void)
{

	while (!CHARSENT) {
		asm("nop");
	}
	return(UDR0);
}

int16_t serial0RecvNum(void)
{
	
	char strBuf[7];
	uint8_t i;
	
	i = 0;
	for (;;) {
		while (!CHARSENT) {
			asm("nop");
		}
		strBuf[i] = UDR0;
		UDR0 = strBuf[i];
		if (strBuf[i] == '\r') {
			strbuf[i] = '\0';
			serial0SendByte('\n');
			break;
		}
		if (i == 5) {
			if ((strBuf[0] != '+') && (strBuf[0] != '-')) {
				serial0SendCRLF();
				serial0SendByte('?');
				serial0SendCRLF();
				return(0);
			}
		}
		if (i == 6) {
			serial0SendCRLF();
			serial0SendByte('?');
			serial0SendCRLF();
			return(0);
		}
		i++;
	}
	return(atoi(strBuf));
}

void serial0SendByte(uint8_t c)
{

	while (!TX0READY) {
		asm("nop");
	}
	UDR0 = c;

}

void serial0SendCRLF(void)
{

	serial0SendStr("\n\r");

}

void serial0SendNum(uint16_t number)
{
	
	char strBuf[7];

	serial0SendStr(ltoa((int32_t) number, strBuf, 10));

}

void serial0SendSignedNum(int16_t number)
{

	char strBuf[7];

	serial0SendStr(ltoa((int32_t) number, strBuf, 10));

}

void serial0SendStr(char *str)
{
	
	uint8_t i;

	i = 0;
	while (str[i]) {
		serial0SendByte(str[i++]);
	}

}

void stepMotor(void)
{
	
	STEP_PORT |= _BV(STEP_PIN);
	_delay_us(10);
	STEP_PORT &= ~_BV(STEP_PIN);
	_delay_us(10);
	stepsDone++;
	
}

void stopMotor(void)
{
	
	cli();
	SLEEP_PORT &= ~_BV(SLEEP_PIN);
	// Turn off counter
	TCCR1B = 0b00000000;
	stepsRequested = 0;
	
}

ISR(PCINT0_vect)
{

	newpina = PINA & LIMITMASK;
	bufpina = newpina ^ oldpina;

	if (bufpina == _BV(FLIMIT_PIN)) {
		if (!(newpina & _BV(FLIMIT_PIN))) {
			stopMotor();
		}
	} else if (bufpina == _BV(RLIMIT_PIN)) {
		if (!(newpina & _BV(RLIMIT_PIN))) {
			stopMotor();
		}
	} else if (bufpina == _BV(HOME_PIN)) {
		if (newpina & _BV(HOME_PIN)) {
			home = position;
		}
	}

	oldpina = newpina;

/*
	if (!(PINA & _BV(FLIMIT_PIN))) {
		stopMotor();
	} n else if (!(PINA & _BV(RLIMIT_PIN))) {
		stopMotor();
	} else if (PINA & _BV(HOME_PIN)) {
		home = position;
	}
*/
}

ISR(TIMER1_COMPA_vect)
{

	if (stepsDone < stepsRequested) {
		stepMotor();
		if (direction) {
			position++;
		} else {
			position--;
		}
	} else {
		stopMotor();
	}
}
/*==============================================================================

TIMERS

Let's suppose we want a maximum speed of 400 steps per second and a minimum
speed of 1 step per second. Using a 14.745600 MHz crystal, the step interval
range is 6.94444 ms to 1 second. If we choose a clock prescaler of 256, the
number of counts between steps for 400 steps per second is 144 and the number
of counts between steps for 1 step per second is 57600 (out of 65535 max).
The length of a tick with a 256 prescaler is 17.361111 us.

We will use a 1 ms pulse (_delay_ms(1)) to trigger the Allegro A4988 driver.


==============================================================================*/