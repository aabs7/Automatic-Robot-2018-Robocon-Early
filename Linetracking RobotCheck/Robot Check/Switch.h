/*
 * Switch.h
 *
 * Created: 1/4/2018 10:50:26 PM
 *  Author: Prakash Chaudhary
 */ 

#include "Headers.h"
#include "drive.h"

#ifndef SWITCH_H_
#define SWITCH_H_

#define SW1 F,2
#define SW2 F,3
#define SW3 F,4
#define SW4 F,5
#define SW5 F,6
#define SW6 F,7

volatile bool Flag_Proportional_Gain_Increase = true;
volatile bool Flag_Proportional_Gain_Decrease = true;
volatile bool Flag_Differential_Gain_Increase = true;
volatile bool Flag_Differential_Gain_Decrease = true;
volatile bool Flag_Integral_Gain_Increase = true;
volatile bool Flag_Integral_Gain_Decrease = true;

void Update_Gain(void)
{
	if (!READ(SW1) && Flag_Proportional_Gain_Decrease)
	{
		//UART0TransmitString("Switch1 clicked\r\n");
		dcrkpp();
		Flag_Proportional_Gain_Decrease = false;
		
	}
	
	if (!READ(SW2) && Flag_Integral_Gain_Decrease)
	{
		//UART0TransmitString("Switch2 clicked\r\n");
		dcrkii();
		Flag_Integral_Gain_Decrease = false;
	}
	if (!READ(SW3) && Flag_Differential_Gain_Decrease)
	{
		//UART0TransmitString("Switch3 cllicked \r\n");
		dcrkdd();
		Flag_Differential_Gain_Decrease = false;
	}
	if (!READ(SW4) && Flag_Differential_Gain_Increase)
	{
		//UART0TransmitString("Switch 4 clicked \r\n");
		incrkdd();
		Flag_Differential_Gain_Increase = false;
	}
	if (!READ(SW5) && Flag_Integral_Gain_Increase)
	{
		//UART0TransmitString("Switch 5 clicked\r\n");
		incrkii();
		Flag_Integral_Gain_Increase = false;
	}
	if (!READ(SW6) && Flag_Proportional_Gain_Increase)
	{
		//UART0TransmitString("Switch 6 clicked \r\n");
		incrkpp();
		Flag_Proportional_Gain_Increase = false;
	}
	
	if (READ(SW1) && READ(SW2) && READ(SW3) && READ(SW4) && READ(SW5) && READ(SW6))
	{
		Flag_Proportional_Gain_Increase = true;
		Flag_Proportional_Gain_Decrease = true;
		Flag_Differential_Gain_Increase = true;
		Flag_Differential_Gain_Decrease = true;
		Flag_Integral_Gain_Increase = true;
		Flag_Integral_Gain_Decrease = true;
		//UART0TransmitString("Reset");
	}
}




#endif /* SWITCH_H_ */