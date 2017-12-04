/*
 * Robot Check.cpp
 *
 * Created: 11/21/2017 4:33:56 PM
 * Author : abheesh
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "uart.h"
#include "drive.h"
#include "encoder.h"
#include "pid.h"

char rcvdata;


encoder e1,e2,e3,e4;
Motor m1(1) , m2(2) , m3(3) , m4(4);



int main(void)
{
	sei();
	initUART0();
	initUART2();
	initUART3();
    while (1) 
    {
		
		SETPOINT1 = -4;
		SETPOINT2 = -4;
		SETPOINT3 = -4;
		SETPOINT4 = -4;
		
		//if(rcvdata == 'w')
			//mov(0);
		//if(rcvdata == 'a')
			//mov(90);
		//if(rcvdata == 's')
			//mov(180);
		//if(rcvdata == 'd')
			//mov(270);
		//else if (rcvdata == 'f')
		//{
			//UART2TransmitString("stop signal \r\n");
			//SETPOINT1 = 0;
			//SETPOINT2 = 0;
			//SETPOINT3 = 0;
			//SETPOINT4 = 0;
		//}
		//
		//
			//if(rcvdata == '1')
				//incrkp();
			//if(rcvdata == '2')
				//dcrkp();
			//if (rcvdata == '3')
				//incrki();
			//if(rcvdata == '4')
				//dcrki();
			//if(rcvdata == '5')
				//incrkd();
			//if (rcvdata == '6')
				//dcrkd();
	//
		//UART2TransmitString("kp = ");UART2TransmitData(kp*100);UART2TransmitString("\t");		//for mobile bluetooth
		//UART2TransmitString("ki = ");UART2TransmitData(ki*100);UART2TransmitString("\t");
		//UART2TransmitString("kd = ");UART2TransmitData(kd*100);UART2TransmitString("\t");
		//UART2TransmitString("\r\n");
		//
		//UART3TransmitData(e2.getspeed());										//for serial plotter
		//UART3TransmitString(" ");
		//UART3TransmitData(SETPOINT2);
		//UART3TransmitString("\r\n");	
		computePid();
		
		/////////////////////////////////////////////CHECK ENCODER///////////////////////////////
		UART0TransmitData(e1.getspeed());UART0TransmitString("\t");
		UART0TransmitData(e2.getspeed());UART0TransmitString("\t");
		UART0TransmitData(e3.getspeed());UART0TransmitString("\t");
		UART0TransmitData(e4.getspeed());UART0TransmitString("\t");
		UART0TransmitString("\r\n");
		/////////////////////////////////////////////////////////////////////////////////////////
    }
}



