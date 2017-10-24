/*
 * AutomaticFinal.cpp
 *
 * Created: 10/22/2017 4:19:45 PM
 * Author : abheesh
 */ 

#include <avr/io.h>
#include "encoder.h"
#include "Motor.h"
#include <avr/interrupt.h>
#include "pid.h"
#include "uart.h"

Motor m1(1),m2(2),m3(3),m4(4);
encoder e1,e2,e3,e4;
extern pid ma,mb,mc,md;

char rcvdata ;

int main(void)
{
    sei();
	initUART0();
	//m1.SetOcrValue(20);
	//m2.SetOcrValue(20);
	//m3.SetOcrValue(20);
	//m4.SetOcrValue(20);
    while (1) 
    {
		//movy();
		rcvdata = UART0Receive();
		if (rcvdata == 'd')
			ma.incrKP();
		else if(rcvdata == 'w')
			ma.incrKD();
		else if (rcvdata == 'a')
			ma.dcrKP();
		else if(rcvdata == 's')
			ma.dcrKD();
		if(rcvdata == 'f')
		{SETPOINT += 3 ;}
		if (rcvdata == 'g')
			SETPOINT -= 3;
		setTuningsM1();
		//setTuningsM2();
		//setTuningsM3();
		//setTuningsM4();
		//setTuningsM2();
		
		UART0TransmitString("motor1: kp= ");UART0TransmitData(ma.kp *100);UART0TransmitString(" ki= ");UART0TransmitData(ma.ki*100);UART0TransmitString(" kd=");UART0TransmitData(ma.kd*100);
		UART0TransmitString("\t\t");
		//UART0TransmitString("motor2: kp= ");UART0TransmitData(mb.kp);UART0TransmitString("\tki= ");UART0TransmitData(mb.ki);UART0TransmitString(" kd=");UART0TransmitData(mb.kd);
		//UART0TransmitString("\t\t");
		//UART0TransmitString("motor3: kp= ");UART0TransmitData(mc.kp);UART0TransmitString("\tki= ");UART0TransmitData(mc.ki);UART0TransmitString(" kd=");UART0TransmitData(mc.kd);
		//UART0TransmitString("\t\t");
		//UART0TransmitString("motor4: kp= ");UART0TransmitData(md.kp);UART0TransmitString("\tki= ");UART0TransmitData(md.ki);UART0TransmitString(" kd=");UART0TransmitData(md.kd);
		UART0TransmitString("\t\t");
		UART0TransmitData(SETPOINT);
		//UART0TransmitString("\r\n");
		//
		UART0TransmitString("\tmotor1speed= ");UART0TransmitData(e1.speed);
		//UART0TransmitString("\tmotor2speed= ");UART0TransmitData(e2.speed);
		//UART0TransmitString("\tmotor3speed= ");UART0TransmitData(e3.speed);
		//UART0TransmitString("\tmotor4speed= ");UART0TransmitData(e4.speed);
		UART0TransmitString("\r\n");
		
	
		
    }
}


ISR(INT0_vect)
{
	e1.count_encoder++;
}
ISR(INT3_vect)
{
	e2.count_encoder++;
}
ISR(INT1_vect)
{
	e3.count_encoder++;
}
ISR(INT2_vect)
{
	e4.count_encoder++;
}
ISR(TIMER0_OVF_vect)
{
	e1.speed = e1.count_encoder;
	e2.speed = e2.count_encoder;
	e3.speed = e3.count_encoder;
	e4.speed = e4.count_encoder;
	e1.count_encoder = 0;
	e2.count_encoder = 0;
	e3.count_encoder = 0;
	e4.count_encoder = 0;
	
}

