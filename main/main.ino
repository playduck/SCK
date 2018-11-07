#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "LedControl.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define SDA 7
#define SCL 9
#define RTC_ADDR 0x68

const uint8_t b[2] = {0, 1};

enum state_t	{
	sleep,
	display,
	config,
	test,
	save
};

LedControl lc = LedControl(5,3,2);

uint8_t volatile state = test;
uint8_t volatile timeout = 0;
uint8_t volatile t[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t volatile active = 1;

void setup()	{
	cli();
	MCUSR = 0;
	//wdt_disable();

	sbi(PRR, 3);
	sbi(PRR, 0);

	pinMode(0, INPUT_PULLUP);
	pinMode(1, INPUT_PULLUP);

	sbi(PCMSK0, 1);
	sbi(PCMSK0, 0);
	sbi(SREG, 7);
	sbi(GIMSK, 4);

	lc.shutdown(0,false);
	lc.setIntensity(0,0);
	lc.setScanLimit(0,4);
	lc.clearDisplay(0);
	
	pinMode(SDA, INPUT_PULLUP);
	pinMode(SCL, INPUT_PULLUP);
	Wire.begin();

	Wire.beginTransmission(RTC_ADDR);
	Wire.write(0x0E);
	Wire.write(0x00);
	Wire.write(0x00);
	Wire.endTransmission();

	lc.clearDisplay(0);

	sei();
}

void loop()	{
	switch(state)	{
		case sleep:	{
			hibernate();
			break;
		}
		case display:	{
			getRtc();
			uint16_t bar = (0xFFFF >> round(0.271 * (float)t[0])) ^ 0xFFFF; // 16/59
			setBank(0, t[2]);
			setBank(1, t[1] | (timeout % 6 == 0 ? 0x80 : 0));
			setBank(2, (bar & 0xFF00) >> 8);
			setBank(3, (bar & 0x00FF));
			lc.shutdown(0,false);
			lc.setIntensity(0,0);
			lc.setScanLimit(0,4);
			break;
		}
		case config:	{
			setBank(0, 0xC0 | active<<1);
			setBank(1, t[1] | (active == 1 ? 0x80 : 0));
			setBank(2, t[2] | (active == 2 ? 0x80 : 0));
			setBank(3, t[3] | (active == 3 ? 0x80 : 0));
			break;
		}
		case test:	{
			lc.shutdown(0,false);
			lc.setIntensity(0,0);
			lc.setScanLimit(0,4);
			setBank(0, 0xFF);
			setBank(1, 0xFF);
			setBank(2, 0xFF);
			setBank(3, 0xFF);
			break;
		}
		case save: {
			cli();
			setBank(2, 0xFF);
			setBank(3, 0xFF);
			lc.shutdown(0,false);
			setRtc();
			timeout = 0;
			state = sleep;
			sei();
			break;
		}
	}
	timeout++;
	if(timeout >= 32 && state != config && state != test)	{
		timeout = 0;
		state = sleep;
	}
	//delayMicroseconds(200);
}

ISR(PCINT0_vect)	{
	cli();
	timeout = 0;
	
	uint8_t f = digitalRead(0) + (digitalRead(1) * 2);	//memento internal pullup
	if(state == config)	{
		switch(f)	{
			case 0: state = save; break;
			case 1: {
				if(active >= 3)	{
					active = 1;
				}	else	{
					active++;
				}
				break;
			}
			case 2:	{
				t[active]++;
				// switch(active)	{
				// 	case 1: t[active] = t[active]%60; break;
				// 	case 2: t[active] = t[active]%24; break;
				// 	case 3: t[active] = t[active]%8; break;
				// }
				break;
			}
		}
	}	else if(state == test)	{
		switch(f)	{
			case 0: state = config; break;
			case 1: state = sleep; break;
			case 2: state = config; break;
		}
	}	else	{
		switch(f)	{
			case 0: state = config; break;	//both
			case 1: state = test; break;	//left
			case 2: state = display; break;	//right
		}
	} 
	sei();
}

void getRtc()	{
	resetRtcPointer();

	Wire.requestFrom(RTC_ADDR, 4);
	t[0] = bcd2dec((Wire.read() & 0x7F));
	t[1] = bcd2dec((Wire.read() & 0x7F));
	t[2] = bcd2dec((Wire.read() & 0x3F));
	t[3] = bcd2dec((Wire.read() & 0x07));

//	resetRtcPointer();
}

void setRtc()	{
	resetRtcPointer();

	Wire.beginTransmission(RTC_ADDR);
	Wire.write(0x00);	//reg pointer
	Wire.write(0x00);	//sec reg
	Wire.write(dec2bcd(t[1]) & 0b01111111);
	Wire.write(dec2bcd(t[2]) & 0b00111111);
	Wire.write(dec2bcd(t[3]) & 0b00000111);
	Wire.endTransmission();

//	resetRtcPointer();
}

void resetRtcPointer()	{
	Wire.beginTransmission(RTC_ADDR);
	Wire.write(0x00);
	Wire.endTransmission();
}

void hibernate()	{

	cli();

	lc.clearDisplay(0);
	lc.shutdown(0,true);
	timeout = 0;

	// Power down mode
	sbi(MCUCR, 4);
	cbi(MCUCR, 3);
		
	// Disable BOD during deep sleep
	// // sbi(MCUCR, 7);
	// // sbi(MCUCR, 2);
	// // asm("nop");
	// // cbi(MCUCR, 2);
	sleep_bod_disable();
//	sbi(PRR, 2);
	sei();

	sleep_mode();

//	cbi(PRR, 2);

}

void setBank(uint8_t bank, uint8_t e)	{
	
	e = reverse(e);
	e = (e<<1) & 0xFF | ( (e & (1<<7) )>>7); // :)

	for(uint8_t i = 1; i < 8; i++)	{
		lc.setLed(0,bank,i,e & (1 << i) );
	}
	lc.setLed(0,bank,0,e & 0x01);
}

uint8_t reverse(uint8_t b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

inline uint8_t dec2bcd(uint8_t num)	{
	return ((num/10 * 16) + (num % 10));
}

inline uint8_t bcd2dec(uint8_t num)	{
	return ((num/16 * 10) + (num % 16));
}
