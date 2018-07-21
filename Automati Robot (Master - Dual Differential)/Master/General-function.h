/*
 * timeperiod.h
 *
 * Created: 6/17/2018 1:24:31 AM
 *  Author: Subash Timilsina
 */ 




#ifndef GENERAL_FUNCTION_H_
#define GENERAL_FUNCTION_H_

#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds (a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER2_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
// the whole number of milliseconds per timer2 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER2_OVERFLOW / 1000)
// the fractional number of milliseconds per timer2 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER2_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)




volatile unsigned long timer2_millis = 0;
volatile unsigned long timer2_fract = 0;


unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;
	
	// disable interrupts while we read timer2_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer2_millis)
	cli();
	m = timer2_millis;
	SREG = oldSREG;
	
	return m;
}



void initialise_timeperiod()
{
	TCCR2B |= (1<<CS22);
	TIMSK2 |= (1<<TOIE2);
	TCNT2 = 0; 
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 


SIGNAL(TIMER2_OVF_vect) {
	timer2_millis += 1;
	timer2_fract += 3;
	if (timer2_fract >= 125) {
		timer2_fract -= 125;
		timer2_millis += 1;
	}
}


#endif /* TIMEPERIOD_H_ */