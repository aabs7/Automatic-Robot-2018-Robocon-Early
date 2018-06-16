/*
 * Master.cpp
 *
 * Created: 6/8/2018 5:24:41 PM
 * Author : Prakash Chaudhary
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "TWI.h"
#include "uart.h"
#include "encoder.h"
#include "drive.h"
#include "qmccompass.h"


////////////////objects of classes///////////
encoder encoderX,encoderY;
char rcvdata;
///////////////////////////////////////////
int main(void)
{
	DDRK = 0xff;
	PORTK &= ~(1<<PK1); 
	uart0_init(UART_BAUD_SELECT(9600,F_CPU));
	uart2_init(UART_BAUD_SELECT(38400,F_CPU));
	//uart3_init(UART_BAUD_SELECT(9600,F_CPU));
	initializeAll();
	
	sei();
	
    while (1) 
    {
		movx(1500,Front);
// 		uart0_putint(bufferMotorSpeed[0]);
// 		uart0_puts("\t");
// 		uart0_putint(bufferMotorSpeed[1]);
// 		uart0_puts("\t");
// 		uart0_putint(bufferMotorSpeed[2]);
// 		uart0_puts("\t");
// 		uart0_putint(bufferMotorSpeed[3]);
// 		uart0_puts("\t");
// 		uart0_putint(velocity_robot[0]);
// 		uart0_puts("\r\n");
    }
	
}


////////For calibration of compass//////////
//  			if(calibrate){
// 				 velocity_robot[0] = 0;
// 				 velocity_robot[1] = 0;
// 				 velocity_robot[2] = 60;
// 				 calculatevel();
// 				 calibrate_compass();
// 				 calibrate = false;
// 			 }
// 			 else{
// 				 velocity_robot[0] = 0;
// 				 velocity_robot[1] = 0;
// 				 velocity_robot[2] = 0;
// 				 calculatevel();
// 				 uart0_putint(X_offset);
// 				 uart0_puts("\t");
// 				 uart0_putint(Y_offset);
// 				 uart0_puts("\n");
// 			 }


