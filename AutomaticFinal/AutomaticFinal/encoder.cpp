/*
 * encoder.cpp
 *
 * Created: 10/18/2017 12:47:40 PM
 *  Author: abheesh
 */ 


#include "encoder.h"
#include <avr/io.h>
#include <avr/interrupt.h>



void encoder::Init_encoder_interrupt()
{
	sei();
	EICRA = 0b11111111;
	EIMSK |= (1<<INT0)|(1<<INT1) | (1<<INT2) | (1<<INT3);
}

void encoder::Init_timer()
{
	sei();
	TCCR0B |= (1<<CS02) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
} 

