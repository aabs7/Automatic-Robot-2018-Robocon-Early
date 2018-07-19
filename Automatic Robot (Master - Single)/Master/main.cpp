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

#define STARTPIN	F,0	

////////////////objects of classes///////////
encoder encoderX,encoderY;
////////////////////////////////////////////
bool junctionY;
bool junctionX;

unsigned int linetrackerDataFront;
unsigned int linetrackerDataBack;
unsigned int previousLinetrackerDataFront;
unsigned int previousLinetrackerDataBack;

void checkJunctionOfY(){
	
	if(bit_is_set(PINK,PK7)){
		junctionY = true;
	}
	else{
		junctionY = false;
	}
	
}
void checkJunctionOfX(){
	if(bit_is_set(PINB,PB4)){
		junctionX = true;
	}
	else{
		junctionX = false;
	}
}

void reactConditionOfLineLeftFront(){
	//////////Find the condition when line is left in front linetracker////////
		linetrackerDataFront = getLineTrackerYFrontData();
		if(linetrackerDataFront == 0 && previousLinetrackerDataFront == 10)
		{
			uart0_puts("f leftedge left \r\n");
			ltYFront.leftedgeleft = true;
			ltYFront.rightedgeleft = false;
			lineMeet = false;
		}
		else if(linetrackerDataFront == 0 && previousLinetrackerDataFront == 80)
		{
			uart0_puts("f rightedge left \r\n");
			ltYFront.rightedgeleft = true;
			ltYFront.leftedgeleft = false;
			lineMeet = false;
		}
		/////////////////////////////////////////////////////
		
		////If line left by linetracker find when line is meet
		if(ltYFront.rightedgeleft && linetrackerDataFront == 80){
			uart0_puts("f linemeet from left \r\n");
			lineMeet = true;
			ltYFront.rightedgeleft = false;
			ltYFront.leftedgeleft = false;
		}
		else if(ltYFront.leftedgeleft && linetrackerDataFront == 10){
			uart0_puts("f linemeet from right \r\n");
			lineMeet = true;
			ltYFront.leftedgeleft = false;
			ltYFront.rightedgeleft = false;
		}
		///////////////////////////////////////////////////////
		if(movingyfront){
			////////// if edge is left and junction is detected //////
			if(ltYFront.leftedgeleft){
					velocity_robot[0] = -30;
					velocity_robot[1] = 10;
					calculateCompassPID();
			}
			else if(ltYFront.rightedgeleft ){
					velocity_robot[0] = 30;
					velocity_robot[1] = 10;
					calculateCompassPID();
			}
	}
	previousLinetrackerDataFront = linetrackerDataFront;
}

void reactConditionOfLineLeftBack(){
	//////////Find the condition when line is left////////
	checkJunctionOfY();
	linetrackerDataBack = getLineTrackerYBackData();
		if(linetrackerDataBack == 0 && previousLinetrackerDataBack == 10 && !junctionY)
		{
			uart0_puts("b leftedge left \r\n");
			ltYBack.leftedgeleft = true;
			ltYBack.rightedgeleft = false;
			lineMeet = false;
		}
		else if(linetrackerDataBack == 0 && previousLinetrackerDataBack == 80 && !junctionY)
		{
			uart0_puts("b rightedge left \r\n");
			ltYBack.rightedgeleft = true;
			ltYBack.leftedgeleft = false;
			lineMeet = false;
		}
		/////////////////////////////////////////////////////
		
		////If line left by linetracker find when line is meet
		if(ltYBack.rightedgeleft && linetrackerDataBack == 80 && !junctionY){
			uart0_puts("b linemeet from left \r\n");
			lineMeet = true;
			ltYBack.rightedgeleft = false;
			ltYBack.leftedgeleft = false;
		}
		else if(ltYBack.leftedgeleft && linetrackerDataBack == 10 && !junctionY){
			uart0_puts("b linemeet from right \r\n");
			lineMeet = true;
			ltYBack.leftedgeleft = false;
			ltYBack.rightedgeleft = false;
		}
		///////////////////////////////////////////////////////
		
		if(movingyback){
			////////// if edge is left and junction is detected //////
			if((ltYBack.leftedgeleft || ltYBack.rightedgeleft) && junctionY){
					velocity_robot[0] = 0;
					velocity_robot[1] = -30;
					calculateCompassPID();
			}
			/////////////////////////////////////////////////////////
			else if(ltYBack.leftedgeleft){
					velocity_robot[0] = -30;
					velocity_robot[1] = -10;
					calculateCompassPID();
			}
			else if(ltYBack.rightedgeleft ){
					velocity_robot[0] = 30;
					velocity_robot[1] = -10;
					calculateCompassPID();
			}
	}
	previousLinetrackerDataBack = linetrackerDataBack;
}


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
 	uart3_init(UART_BAUD_SELECT(9600,F_CPU));
 	//INITIALIZE EVERYTHING ELSE
 	initializeAll();
 	sei();
	DDRB |= (1<<PB7);
	DDRK = 0XFF;
	DDRL = 0x00;
	
	DDRF = 0X00;
	PORTF = 0xff;
	
	//while(READ(STARTPIN));
    while (1) 
    {
		
		
		PORTK = 0xff;
//  			PORTB |= (1<<PB7);
 			_delay_ms(2000);
// 			PORTB = 0x00;
 			PORTK = 0X00;
 			_delay_ms(2000);
// 			if (bit_is_set(PINL,6))
// 				PORTK = 0xFF;
// 			else
// 				PORTK = 0x00;	
//    		gorockthegamefield();
//    		reactConditionOfLineLeftFront();
//    		reactConditionOfLineLeftBack();
//    		calculatevel();
//  		uart0_putint(encoderX.getdistance());
//  		uart0_puts("\t");
//  		uart0_putint(encoderY.getdistance());
//  		uart0_puts("\r\n");
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


