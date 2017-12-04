/*
 * encoder.cpp
 *
 * Created: 10/18/2017 12:47:40 PM
 *  Author: abheesh
 */ 


#include "encoder.h"
#include <avr/io.h>
#include <avr/interrupt.h>

extern encoder e1,e2,e3,e4;

void encoder::Init_encoder_interrupt()
{
	sei();
	EICRA = 0b11111111;
	EICRB = 0b00000011;
	EIMSK |= (1<<INT1) | (1<<INT2) | (1<<INT3) | (1<<INT4);
}

void encoder::Init_timer()
{
	sei();
	TCCR0B |= (1<<CS02) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
}
int encoder::getrpm()
{
	return (((speed/16)*1000)*float(60/13));
	//return (speed * float(249/23));
}


ISR(INT1_vect)
{
	if((bit_is_set(PINA,PA1)))
	{
		e1.inc_count();
	}
	else
	e1.dcr_count();

}
ISR(INT2_vect)
{
	if((bit_is_set(PINA,PA2)))
	{
		e2.inc_count();
	}
	else
	e2.dcr_count();
}
ISR(INT3_vect)
{
	if((bit_is_set(PINA,PA3)))
	{
		e3.inc_count();
	}
	else
	e3.dcr_count();
}
ISR(INT4_vect)
{
	if((bit_is_set(PINA,PA4)))
	{
		e4.inc_count();
	}
	else
	e4.dcr_count();
}
ISR(TIMER0_OVF_vect)
{
	e1.setspeed();
	e2.setspeed();
	e3.setspeed();
	e4.setspeed();
	
}