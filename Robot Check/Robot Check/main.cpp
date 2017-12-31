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
#include "zonedrive.h"
/////////////	Objects //////////////////
encoder encoderX,encoderY,e1,e2,e3,e4;
Motor m1(1) , m2(2) , m3(3) , m4(4);
extern bodyPid encoderx,encodery;
/////////////////////////////////////////
char rcvdata;
int main(void)
{
	INPUT(SHUTTLECOCKPIN);
	INPUT(ZONEPIN);
	SET(SHUTTLECOCKPIN);
	SET(ZONEPIN);
	sei();
	initUART0();
	initUART2();
	initUART3();
    while (1) 
    {	
		rcvdata = UART3Receive();
		if(rcvdata == 'p')	incrkpp();
		else if(rcvdata == 'o')	dcrkpp();
		else if(rcvdata == 'i')	incrkii();
		else if(rcvdata == 'd')	incrkdd();
		else if(rcvdata == 'u')	dcrkii();
		else if(rcvdata == 's')	dcrkdd();
		gorockthegamefield();
		computePid();
		if(where == inTZ3)	return 0;
		
		UART2TransmitData(encoderX.getdistance());
		UART2TransmitString("\t");
		UART2TransmitData(encoderx.SETPOINT);
		//UART2TransmitString("\t");
		UART3TransmitData(kpp*100);
		UART3TransmitString("\t");
		UART3TransmitData(kii * 100);
		UART3TransmitString("\t");
		UART3TransmitData(kdd * 100);
		//UART0TransmitData();
		UART3TransmitString("\t");
		//UART0TransmitData(velocity_motor[3]);
		//UART0TransmitString("\t");
		UART2TransmitString("\r\n");
		UART3TransmitString("\r\n");
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

