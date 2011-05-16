#define F_CPU 16000000UL
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>


unsigned char hour=0;
unsigned char minute=0;
unsigned char second=0;
unsigned char mode=0;


void AllOff() {
	DDRD = 0x00;	// INPUT
	PORTD = 0;
	DDRB = 0x00;	// INPUT
	PORTB = 0;
}


void Light(unsigned char led) {
	unsigned char dgnd;
	unsigned char dpos;
	unsigned char dgndbitmask;
	unsigned char dposbitmask;
	unsigned char b1,b2;

	dgnd=led>>3;
	dpos=led%8;
	if (dgnd<=dpos) dpos++;
	dgndbitmask=1<<dgnd;
	dposbitmask=1<<dpos;
	b1=0;
	b2=0;
	if (dposbitmask==0) {b1=1; b2=1;}
	if (dgndbitmask==0) {b1=1; b2=0;}

	DDRD=dgndbitmask+dposbitmask;
	PORTD=dposbitmask;
	DDRB=b1;
	PORTB=b2;
}





void BrightnessTest() {
	unsigned char led;
	
	for (;;) {


		AllOff();
		for (int i=0; i<50; i++) {
			for (led=0; led<72; led++) {
				Light(led);
				_delay_ms(1);
			}
		}


		AllOff();
		DDRD=0xFF;
		for (int i=0; i<500; i++) {
			PORTD=0b11111110;
			_delay_ms(1);
			PORTD=0b11111101;
			_delay_ms(1);
			PORTD=0b11111011;
			_delay_ms(1);
			PORTD=0b11110111;
			_delay_ms(1);
			PORTD=0b11101111;
			_delay_ms(1);
			PORTD=0b11011111;
			_delay_ms(1);
			PORTD=0b10111111;
			_delay_ms(1);
			PORTD=0b01111111;
			_delay_ms(1);
		}
	}
}




void GetTime() {
	second++;
	if (second>59) {
		second=0;
		minute++;
		if (minute>59) {
			minute=0;
			hour++;
			if (hour>23) {
				hour=0;
			}
		}
	}
}


void ReverseHMS() {
	unsigned char led;

	for (led=0; led<72; led++) {
		if (led==second || led==minute || led==(60+(hour>>1))) {
			_delay_ms(1);
		} else {
			Light(led);
			_delay_ms(1);
		}
	}
}


void StandardHMS() {
	Light(60+(hour>>1));
	_delay_ms(1);
	Light(minute);
	_delay_ms(1);
	Light(second);
	_delay_ms(1);
}



int main (void) {
	hour=0;
	minute=0;
	second=0;
	mode=0;

	//BrightnessTest();


	for (;;) {

		for (int i=0; i<25; i++) {
			if (mode==0) StandardHMS();
			if (mode==1) ReverseHMS();
		}

		GetTime();


	}


	return 1;
}
