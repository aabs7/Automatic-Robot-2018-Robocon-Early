/*
 * drive.h
 *
 * Created: 10/23/2017 3:16:43 PM
 *  Author: abheesh
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#include "motor.h"
#include "uart.h"
#include <math.h>


#define PI 3.14159265

float velocity_motor[3];
int velocity_robot[3];
float coupling_matrix[4][3] = {{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};  //0.707
double value = PI / 180;

int SETPOINT1;
int SETPOINT2;
int SETPOINT3;
int SETPOINT4;


void calculatevel()
{
	
	for(int i=0;i<4;i++)
	{
		velocity_motor[i] = 0;
		for(int j=0;j<3;j++)
		{
			velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
			
		}
	}
	//UART0TransmitString("velocity robot1 = ");UART0TransmitData(SETPOINT1);
	//UART0TransmitString("\tvelocity robot2 =");UART0TransmitData(SETPOINT2);
	//UART0TransmitString("\tvelocity robot3 =");UART0TransmitData(SETPOINT3);
	//UART0TransmitString("\tvelocity robot4 =");UART0TransmitData(SETPOINT4);
	//UART0TransmitString("\r\n");
	SETPOINT1 = (velocity_motor[0] * 23)/249;
	SETPOINT2 = (velocity_motor[1] * 23)/249;
	SETPOINT3 = (velocity_motor[2] * 23)/249;
	SETPOINT4 = (velocity_motor[3] * 23)/249 ;
	
}



void mov(int degree)
{
	velocity_robot[0] = (50 * float(cos(degree * value)));
	velocity_robot[1] = (50 * float(sin(degree * value)));
	velocity_robot[2] = 0;
	calculatevel();

	
}
#endif /* DRIVE_H_ */