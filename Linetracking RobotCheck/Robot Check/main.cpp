/*
 * Robot Check.cpp
 *
 * Created: 11/21/2017 4:33:56 PM
 * Author : abheesh
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Motor.h"
#include "drive.h"
#include "encoder.h"
#include "uart.h"
#include "pid.h"
#include "Linetracker.h"
#include "Switch.h"
/////////////	Objects //////////////////
encoder encoderX,encoderY,e1,e2,e3,e4;
Motor m1(1) , m2(2) , m3(3) , m4(4);
Linetracker l1(0);
extern bodyPid encoderx,encodery;
/////////////////////////////////////////
char rcvdata;
char detData = 'f';
bool newdatacome = true;

int main(void)
{
	//INPUT(SW1);
	//INPUT(SW2);
	//INPUT(SW3);
	//INPUT(SW4);
	//INPUT(SW5);
	//INPUT(SW6);
	sei();
	initUART2();
	UART2TransmitString("Initializing all robot... \r\n");
	l1.initialise();
	//l1.Calibrate();
    while (1) 
    {	
		
		//UART0TransmitData(l1.Get_Sensors_Data());
		//UART0TransmitString("\n");
		rcvdata = UART2Receive();
		if(newdatacome)
		{
			movx(10000,Front);
		}
		else if(!newdatacome)
		{
			movx(10000,Back);
		}
		if(rcvdata == 'p')	{incrkpp();}
		else if(rcvdata == 'o')	{dcrkpp();}
		else if(rcvdata == 'i')	{incrkii();}
		else if(rcvdata == 'd')	{incrkdd();}
		else if(rcvdata == 'u')	{dcrkii();}
		else if(rcvdata == 's')	{dcrkdd();}
		else if(rcvdata == 'r'){newdatacome = false;}
		computePid();
		if(l1.Get_Sensors_Data() == 255){
			stopDrive();
		}
		//if(l1.Get_Sensors_Data() == 255){
		//stopDrive();
		//return 0;
		//}
		//if(where == inTZ3)	return 0;
		
		//UART2TransmitData(encoderX.getdistance());
		//UART2TransmitString("\t");
		//UART2TransmitData(encoderx.SETPOINT);
		//UART2TransmitString("\t");
		UART2TransmitData(kpp*100);
		UART2TransmitString(" e-2\t\t");
		UART2TransmitData(kii * 100000);
		UART2TransmitString(" e-5\t\t");
		UART2TransmitData(kdd * 1000);
		//UART0TransmitData();
		UART2TransmitString(" e-2\t\t");
		//UART0TransmitData(velocity_motor[3]);
		//UART0TransmitString("\t");
		UART2TransmitString("\r\n");
		//UART3TransmitString("\r\n");
    }
}

//void throwingzone1()
//{
	//if(flag)
	//{
		//movx(4500,Back);
		//computePid();
		//if(abs(encoderX.getdistance()) >= 4500){
			//flag = false;
			//stopDrive();
			//movy(3000,Front);
			//_delay_ms(2000);
			//UART0TransmitString("it stoopped \n");
			//encoderX.distance = 0;
			//encoderX.resetCount();
			//encoderY.distance = 0;
			//encoderY.resetCount();
			//
		//}
	//}
	//else
	//{
		//movy(3000,Back);
		//computePid();
		//if(abs(encoderY.getdistance()) >= 3000)
		//{
			//stopDrive();
		//}
	//}
//}

