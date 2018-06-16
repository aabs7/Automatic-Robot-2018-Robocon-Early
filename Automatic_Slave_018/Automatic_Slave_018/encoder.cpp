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
extern bool PidUpdateFlagMotor;
extern bool getdata;

char motorcount;
char datacount;


void encoder::Init_encoder_interrupt()
{
	sei();
	EICRA = 0b11111111;
	EICRB = 0b00001111;
	EIMSK |=  (1<<INT2) | (1<<INT3) | (1<<INT4) | (1<<INT5) ;
}

void encoder::Init_timer()	//FOR SPEED TUNING OF 4 MOTORS
{
	sei();
	TCCR0B |= (1<<CS02) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
}
float encoder::getdistance()
	{
	distance = (3.1415 * encoderdiameter * count_encoder)/(encoderPPR) ; 
	return distance;
}

ISR(INT4_vect)	//for motor1
{
	if((bit_is_set(PINA,PA4)))
	{
		e1.inc_count();
	}
	else
	e1.dcr_count();

}
ISR(INT5_vect)	//for motor2
{
	if((bit_is_set(PINA,PA1)))
	{
		e2.inc_count();
	}
	else
	e2.dcr_count();
}
ISR(INT2_vect)	//for motor3
{
	if((bit_is_set(PINA,PA2)))
	{
		e3.inc_count();
	}
	else
		e3.dcr_count();
}
ISR(INT3_vect)	//for motor4
{
	if((bit_is_set(PINA,PA3)))
	{
		e4.inc_count();
	}
	else
	e4.dcr_count();
}

ISR(TIMER0_OVF_vect)
{
	datacount++;
	
		PidUpdateFlagMotor = true;
		
 	if(datacount == 2){
 		//getdata = true;
 		datacount = 0;
 	}
	
	e1.setspeed();
	e2.setspeed();
	e3.setspeed();
	e4.setspeed();
	
}