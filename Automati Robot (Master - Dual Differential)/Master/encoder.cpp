/*
 * encoder.cpp
 *
 * Created: 10/18/2017 12:47:40 PM
 *  Author: abheesh
 */ 


#include "encoder.h"

#include <avr/io.h>
#include <avr/interrupt.h>


extern encoder encoderX,encoderY;

volatile bool PidUpdateFlagLinetrackerBack = true;
volatile bool PidUpdateFlagLinetrackerFront = true;
volatile bool PidUpdateFlagCompass = true;
volatile bool PidUpdateFlagDriveX = true;
volatile bool PidUpdateFlagDriveY = true;


void encoder::Init_encoder_interrupt()
{
	sei();
	EICRA = 0b00110000;
	EICRB = 0b00001100;
	EIMSK |=  (1<<INT2) |(1<<INT5);
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

ISR(INT2_vect)	//for x -axis encoder
{
	if((bit_is_set(PINA,PA0)))
	{
		encoderX.inc_count();
	}
	else
		encoderX.dcr_count();
}

ISR(INT5_vect)	//for y-axis encoder
{
	if((bit_is_set(PINA,PA5)))
	{
		encoderY.inc_count();
	}
	else
		encoderY.dcr_count();
}

ISR(TIMER0_OVF_vect)
{
	PidUpdateFlagCompass = true;
	PidUpdateFlagLinetrackerBack = true;
	PidUpdateFlagLinetrackerFront = true;
 
	PidUpdateFlagDriveX = true;
	PidUpdateFlagDriveY = true;
	
}