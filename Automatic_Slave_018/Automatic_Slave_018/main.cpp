/*
 * I2C Communication (Slave).cpp
 *
 * Created: 6/8/2018 10:58:21 AM
 * Author : Prakash Chaudhary
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "TWI.h"
#include "uart.h"
#include "Motor.h"
#include "encoder.h"
#include "pid.h"

Motor m1(1),m2(2),m3(3),m4(4);
encoder e1,e2,e3,e4;
extern pid ma,mb,mc,md;
	
///Tuning parameter//
/*
	first time when installed suspension without communicaiton
	motor1 -> 2.6, 0 , 3
	motor2 -> 2.2, 0 , 4
	motor3 -> 1.9, 0 , 6
	motor4 -> 1.95, 0 , 6
*/

int main(void)
{
 	PCICR |= (1<<PCIE0);
 	PCMSK0 |= (1<<PCINT5);
	ma.setpid(2.6,0,3);
	mb.setpid(2.2,0,4);
	mc.setpid(1.9,0,6);
	md.setpid(1.95,0,6);
	
	initUART0();
	initUART2();

	sei();

	
	while (1)
	{
		UART0TransmitData(e1.getspeed());
		UART0TransmitString("\r\n");
 		if(abs(rcvdata[0]) <= 4){
	 		ma.setpid(2.6,0,3);
	 		mb.setpid(2.2,0,4);
	 		mc.setpid(1.9,0,6);
	 		md.setpid(1.95,0,6);
 		}
 		else{
	 		ma.setpid(1.1,0,3);
	 		mb.setpid(1.1,0,3);
	 		mc.setpid(0.9,0,3);
	 		md.setpid(0.9,0,3);
 		}
		computePid();	
	}
}
 ISR(PCINT0_vect)
 {
	stopDrive();
}



////////////////////To tune motor/////////////////////////////
//char data = 0;
//
//void doall(void){
//
//UART3TransmitData(ma.kp * 100);
//UART3TransmitString(" ");
//UART3TransmitData(ma.ki * 100);
//UART3TransmitString(" ");
//UART3TransmitData(ma.kd * 2);
//UART3TransmitString(" ");
//
//UART3TransmitData(mb.kp * 100);
//UART3TransmitString(" ");
//UART3TransmitData(mb.ki * 100);
//UART3TransmitString(" ");
//UART3TransmitData(mb.kd * 2);
//UART3TransmitString(" ");
//
//UART3TransmitData(mc.kp * 100);
//UART3TransmitString(" ");
//UART3TransmitData(mc.ki * 100);
//UART3TransmitString(" ");
//UART3TransmitData(mc.kd * 2);
//UART3TransmitString(" ");
//
//UART3TransmitData(md.kp * 100);
//UART3TransmitString(" ");
//UART3TransmitData(md.ki * 100);
//UART3TransmitString(" ");
//UART3TransmitData(md.kd * 2);
//
//UART3TransmitString("\r\n");
//}
///////////To tune Motor/////////////////////
// 		data = UART3Receive();
// 		
// 		if(data == 'p'){
// 			ma.incrKP();
// 			doall();
// 		}
// 		else if(data == 'o'){
// 			mb.incrKP();
// 			doall();
// 			
// 		}
// 		else if(data == 'i'){
// 			
// 			mc.incrKP();
// 			doall();
// 		}
// 		else if(data == 'u'){
// 			md.incrKP();
// 			doall();
// 		}
// 		else if(data == 'l'){
// 			ma.dcrKP();
// 			doall();
// 		}
// 		else if(data == 'k'){
// 			mb.dcrKP();
// 			doall();
// 		}
// 		else if(data == 'j'){
// 			mc.dcrKP();
// 			doall();
// 		}
// 		else if(data == 'h'){md.dcrKP();doall();}
// 		
// 		else if(data == 'q'){ma.incrKD();doall();}
// 		else if(data == 'w'){mb.incrKD();doall();}
// 		else if(data == 'e'){mc.incrKD();doall();}
// 		else if(data == 'r'){md.incrKD();doall();}
// 		
// 		else if(data == 'a'){ma.dcrKD();doall();}
// 		else if(data == 's'){mb.dcrKD();doall();}
// 		else if(data == 'd'){mc.dcrKD();doall();}
// 		else if(data == 'f'){md.dcrKD();doall();}
// 		
// 		else if(data == 'z'){ma.incrKI();doall();}
// 		else if(data == 'x'){mb.incrKI();doall();}
// 		else if(data == 'c'){mc.incrKI();doall();}
// 		else if(data == 'v'){md.incrKI();doall();}
// 		
// 		else if(data == ','){ma.dcrKI();doall();}
// 		else if(data == '/'){mb.dcrKI();doall();}
// 		else if(data == ' '){mc.dcrKI();doall();}
// 		else if(data == '.'){md.dcrKI();doall();}
// 		
// 		//	}
// 		//////////////////////////////////////////////////////////////////
// 		
// 		/////////To see graph of response////////////////////////////////
// 		
// 		UART0TransmitData(e1.getspeed());
// 		UART0TransmitString("\t");
// 		UART0TransmitData(e2.getspeed());
// 		UART0TransmitString("\t");
// 		UART0TransmitData(e3.getspeed());
// 		UART0TransmitString("\t");
// 		UART0TransmitData(e4.getspeed());
// 		UART0TransmitString("\t");
// 		UART0TransmitData(rcvdata[3]);
// 		UART0TransmitString("\t");
// 		UART0TransmitData(-rcvdata[3]);
// 		UART0TransmitString("\r\n");
// 		
// 		////////////////////////////////////////////////////////////////
