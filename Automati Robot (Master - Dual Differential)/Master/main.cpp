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
#include "zonedrive.h"
#include "headers.h"
#include "gy88.h"

////////////////objects of classes///////////
encoder encoderX,encoderY;
////////////////////////////////////////////

int main(void)
{
	////////////SET COMMUNICATION PINS AS INPUT AND PULL UP////////
	INPUT(SHUTTLECOCKPIN);
	INPUT(ZONEPIN);
 	SET(SHUTTLECOCKPIN);
 	SET(ZONEPIN);
	///////PULL DOWN RACK PIN 
	INPUT(RACKPIN);
	CLEAR(RACKPIN);
	
	///Turn internal pullup for limit switch pin
	INPUT(RIGHT_LIMIT_SW);
	INPUT(LEFT_LIMIT_SW);
	SET(RIGHT_LIMIT_SW);
	SET(LEFT_LIMIT_SW);
	///SET PK1 AS OUTPUT TO SEND SIGNAL TO SLAVE TO BRAKE MOTOR 
	DDRK |= (1<<PK0);
	PORTK &= ~(1<<PK0); 
	/// INITIALIZE ALL THE UART
	uart0_init(UART_BAUD_SELECT(9600,F_CPU));
	uart2_init(UART_BAUD_SELECT(38400,F_CPU));
	uart3_init(UART_BAUD_SELECT(38400,F_CPU));
	//INITIALIZE EVERYTHING ELSE
	initializeAll();
	
// 	task1 = task2 = task3 = task4 = task5 = task6 = task7 =true;
// 	ManualInFrontOfLZ2 = false;
// 	where = inLZ2;
// 	robotState = notmoving;
	//ShuttleCockGiven = ShuttleCockArmGone = true;
	char rcvdata = 't';
	sei();
    while (1) 
    {
		
// 		rcvdata = uart0_getc();
// 		uart3_putc(rcvdata);
	//	uart0_putc(rcvdata);

 		gorockthegamefield();
// 		
// 		reactConditionOfLineLeftFront();
// 		reactConditionOfLineLeftBack();
// 		
// 		
 		calculatevel();
	}
}


///////////////copy this code and run.//////////////
////get linetracker data
//lineTrackerData = getLineTrackerYdata();
////get compass data
//compass_Angle = get_Angle();
////Check the junction of Y linetracker
//checkJunctionOfY();
////call the gameplay function
//changeCompassSetpoint();
//gorockthegamefield();
////check for line left condition and react to it
//reactConditionOfLineLeft();
////calculate velocity of each motor and send to slave
//calculatevel();
////set previous line tracker data
//previousLinetrackerData = linetracker_data;
////////not below this line///////////////////////


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


