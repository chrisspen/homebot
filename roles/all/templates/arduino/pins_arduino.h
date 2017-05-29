#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// ATMEL ATMEGA1284 on Hobbytronics UNO*Pro
//
//
// 2014-01-24 - Modified for Software Serial Library to work
//
//   ATMEGA1284 Pin		Arduino Mapping	   Features
//	   PA0                   D21  A7
//    PA1                   D20  A6
//	   PA2                   D19  A5
//    PA3                   D18  A4   
//	   PA4                   D17  A3			
//    PA5                   D16  A2    
//	   PA6                   D15  A1 
//    PA7                   D14  A0
//
//	   PB0                   D4
//    PB1                   D5
//	   PB2                   D2
//    PB3                   D3               PWM
//	   PB4                   D10					PWM  SS
//    PB5                   D11                   MOSI
//	   PB6                   D12              PWM  MISO
//    PB7                   D13              PWM  SCK
//
//	   PC0                   D22                   SCL
//    PC1                   D23                   SDA
//	   PC2                   D24              
//    PC3                   D25
//	   PC4                   D26
//    PC5                   D27
//	   PC6                   D28 
//    PC7                   D29
//
//	   PD0                   D0                    RX
//    PD1                   D1                    TX
//	   PD2                   D6                    RX1
//    PD3                   D7                    TX1
//	   PD4                   D30              PWM
//    PD5                   D8               PWM
//	   PD6                   D9               PWM
//    PD7                   D31              PWM
//

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t SDA = 23;
static const uint8_t SCL = 22;
static const uint8_t LED = 13;

static const uint8_t A0 = 0;
static const uint8_t A1 = 1;
static const uint8_t A2 = 2;
static const uint8_t A3 = 3;
static const uint8_t A4 = 4;
static const uint8_t A5 = 5;
static const uint8_t A6 = 6;
static const uint8_t A7 = 7;

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8

// Macros
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 14 : -1)
#define digitalPinToAnalogPin(p)    ((p) >= 14 && (p) <= 21 ? (p) - 14 : -1 )
#define analogPinToChannel(p)	      ((p) < NUM_ANALOG_INPUTS ? NUM_ANALOG_INPUTS - (p) - 1: -1 )
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 8 || (p) == 9 || (p) == 10 || (p) == 12 || (p) == 13 || (p) == 30 || (p) == 31 )

#define ifpin(p,what,ifnot)	    (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (what) : (ifnot))

extern const uint8_t digital_pin_to_pcint[NUM_DIGITAL_PINS];

#define digitalPinToPCICR(p)    ifpin(p,(&PCICR),(uint8_t *)0)
#define digitalPinToPCICRbit(p) ifpin(p,(digital_pin_to_pcint[p] >> 3),0)
#define digitalPinToPCMSK(p)    (((p) <= 1) ? (&PCMSK3) : (((p) <= 5) ? (&PCMSK1) : (((p) <= 9) ? (&PCMSK3) : (((p) <= 13) ? (&PCMSK1) : (((p) <= 21) ? (&PCMSK0) : (((p) <= 29) ? (&PCMSK2) : (((p) <= 31) ? (&PCMSK3) : ((uint8_t *)0))))))))
#define digitalPinToPCMSKbit(p) ifpin(p,digital_pin_to_pcint[p] & 0x7,0)

#define digitalPinToInterrupt(p)  ((p) == 6 ? 0 : ((p) == 7 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))

#ifdef ARDUINO_MAIN

#define PA 1
#define PB 2
#define PC 3
#define PD 4

const uint8_t digital_pin_to_pcint[NUM_DIGITAL_PINS] =
{
  24, // D0  PD0
  25, // D1  PD1 1
  10, // D2  PB2
  11, // D3  PB3
  8,  // D4  PB0
  9,  // D5  PB1 5
  26, // D6  PD2
  27, // D7  PD3  
  29, // D8  PD5
  30, // D9  PD6 9
  12, // D10 PB4
  13, // D11 PB5
  14, // D12 PB6
  15, // D13 PB7 13
  7,  // D14 PA7
  6,  // D15 PA6
  5,  // D16 PA5
  4,  // D17 PA4
  3,  // D18 PA3
  2,  // D19 PA2
  1,  // D20 PA1
  0,  // D21 PA0 21
  16, // D22 PC0
  17, // D23 PC1
  18, // D24 PC2
  19, // D25 PC3
  20, // D26 PC4
  21, // D27 PC5
  22, // D28 PC6
  23, // D29 PC7 29
  28, // D30 PD4
  31  // D31 PD7 31
};


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
  PD, // D0
  PD, // D1
  PB, // D2
  PB, // D3  
  PB, // D4
  PB, // D5
  PD, // D6
  PD, // D7
  PD, // D8
  PD, // D9
  PB, // D10
  PB, // D11
  PB, // D12
  PB, // D13
  PA, // D14
  PA, // D15
  PA, // D16
  PA, // D17
  PA, // D18
  PA, // D19
  PA, // D20
  PA, // D21
  PC, // D22
  PC, // D23
  PC, // D24
  PC, // D25
  PC, // D26
  PC, // D27
  PC, // D28
  PC, // D29
  PD, // D30
  PD, // D31
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0), // D0  PD0
  _BV(1), // D1  PD1
  _BV(2), // D2  PB2
  _BV(3), // D3  PB3
  _BV(0), // D4  PB0
  _BV(1), // D5  PB1
  _BV(2), // D6  PD2
  _BV(3), // D7  PD3  
  _BV(5), // D8  PD5
  _BV(6), // D9  PD6
  _BV(4), // D10 PB4
  _BV(5), // D11 PB5
  _BV(6), // D12 PB6
  _BV(7), // D13 PB7
  _BV(7), // D14 PA7 (A0)
  _BV(6), // D15 PA6 (A1)
  _BV(5), // D16 PA5 (A2)
  _BV(4), // D17 PA4 (A3)
  _BV(3), // D18 PA3 (A4)
  _BV(2), // D19 PA2 (A5)
  _BV(1), // D20 PA1 (A6)
  _BV(0), // D21 PA0 (A7)
  _BV(0), // D22 PC0
  _BV(1), // D23 PC1
  _BV(2), // D24 PC2
  _BV(3), // D25 PC3
  _BV(4), // D26 PC4
  _BV(5), // D27 PC5
  _BV(6), // D28 PC6
  _BV(7), // D29 PC7
  _BV(4), // D30 PD4
  _BV(7), // D31 PD7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER, // D0  PD0
  NOT_ON_TIMER, // D1  PD1
  NOT_ON_TIMER, // D3  PB2
  TIMER0A,      // D3  PB3
  NOT_ON_TIMER, // D4  PB0
  NOT_ON_TIMER, // D5  PB1
  NOT_ON_TIMER, // D6  PD2
  NOT_ON_TIMER, // D7  PD3  
  TIMER1A,      // D8  PD5
  TIMER2B,      // D9  PD6
  TIMER0B,      // D10 PB4
  NOT_ON_TIMER, // D11 PB5
  TIMER3A,      // D12 PB6
  TIMER3B,      // D13 PB7
  NOT_ON_TIMER, // D14 PA0
  NOT_ON_TIMER, // D15 PA1
  NOT_ON_TIMER, // D16 PA2
  NOT_ON_TIMER, // D17 PA3
  NOT_ON_TIMER, // D18 PA4
  NOT_ON_TIMER, // D19 PA5
  NOT_ON_TIMER, // D20 PA6
  NOT_ON_TIMER, // D21 PA7
  NOT_ON_TIMER, // D22 PC0
  NOT_ON_TIMER, // D23 PC1
  NOT_ON_TIMER, // D24 PC2
  NOT_ON_TIMER, // D25 PC3
  NOT_ON_TIMER, // D26 PC4
  NOT_ON_TIMER, // D27 PC5
  NOT_ON_TIMER, // D28 PC6
  NOT_ON_TIMER, // D29 PC7
  TIMER1B,      // D30 PD4
  TIMER2A,      // D31 PD7
};


#endif // ARDUINO_MAIN

#endif // Pins_Arduino_h
// vim:ai:cin:sts=2 sw=2 ft=cpp
