/*
 * drive.h
 *
 * Created: 10/23/2017 3:16:43 PM
 *  Author: abheesh
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#define PI		3.14159265

#include "motor.h"
#include "uart.h"
#include "encoder.h"
#include "Flags.h"
#include "Linetracker.h"
#include <math.h>

extern encoder encoderY, encoderX;
extern Motor m1,m2,m3,m4;
extern Linetracker l1;
bool PidUpdateFlagLinetracker = true;

////////////////////////////////////////////////////////////////////////////////
int SETPOINT1;
int SETPOINT2;
int SETPOINT3;
int SETPOINT4;
float velocity_motor[4];
int velocity_robot[3];
float coupling_matrix[4][3] = {{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};  //0.707
double value = PI / 180;
////////////////////////////////////////////////////////////////////////////////

int distanceX;
int distanceY;
int currentDistance;
int difference;
bool fullspeed = false;

////////////////////////////////////////////////////////////////////////////////
enum direction{
	X_Axis,
	Y_Axis,
	Front,
	Back
};
////////////////////////////////////////////////////////////////////////////////
double kpp = 0.4, kii = 0.00016 , kdd = 0.064;

//kdd = 0.054 for 40 ocr
//kdd = 0.064 for 80 ocr

struct bodyPid{ 
	int input,error,prevInput,output;
	double Iterm;
	int SETPOINT;	
};
bodyPid lt;

/////////////////////////////////////////////////////////

void ramp(int axis,int direction)
{
	if(axis == X_Axis){	
		currentDistance = abs(encoderX.getdistance());
		difference = distanceX - currentDistance;
		if(difference > 0){
			if(currentDistance < 1000)		//ramp up
			{
				velocity_robot[0] = 30 + 0.12*currentDistance;
			}
		
			else if(currentDistance >= 1000)		//same speed
			{
				velocity_robot[0] = 150;
				fullspeed = true;
			}
		
			if(fullspeed && difference < 1500)//ramp down
			{
				velocity_robot[0] =  0.1*difference ;
				if(velocity_robot[0] < 40)
					velocity_robot[0] = 40;
			}
			if(direction == Front)	velocity_robot[0] = velocity_robot[0];
			else					velocity_robot[0] = -velocity_robot[0];
		}
	}
	else if(axis == Y_Axis){ 
		currentDistance = abs(encoderY.getdistance());
		difference = distanceY - currentDistance;
		if(difference > 0){
			if(currentDistance < 1000)		//ramp up
			{
				velocity_robot[1] = 30 + 0.12*currentDistance;	//0.15 because in 800mm it reach 150 ocr
			}
		
			else if(currentDistance > 1000)		//same speed
			{
				velocity_robot[1] = 150;
				fullspeed = true;
			}
		
			if(fullspeed && difference < 1500)//ramp down
			{
				velocity_robot[1] =  0.1*difference; //0.1 because in 1500mm it reach 0 ocr
				if(velocity_robot[1] < 40)
					velocity_robot[1] = 40;
			}
			if(direction == Front)	velocity_robot[1] = velocity_robot[1];
			else					velocity_robot[1] = -velocity_robot[1];
			
		}
	}
	
}

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
	SETPOINT1 = (velocity_motor[0] * 23)/249;	  
	SETPOINT2 = (velocity_motor[1] * 23)/249;	  
	SETPOINT3 = (velocity_motor[2] * 23)/249;	  
	SETPOINT4 = (velocity_motor[3] * 23)/249 ;  
}		//use matrix to find setpoint

void movx(int distance,int direction)
{

		distanceX = distance;
		if(BodyPidFlag && PidUpdateFlagLinetracker){
			lt.SETPOINT = 35;
			lt.input = l1.Get_Sensors_Data();
			lt.error = lt.SETPOINT - lt.input;
			lt.Iterm += kii * lt.error;
			lt.output = kpp * lt.error + lt.Iterm -kdd *(lt.input - lt.prevInput);
	
			velocity_robot[1] = lt.output;
			PidUpdateFlagLinetracker = false;	
						//for reverse y compensation, -ve sign
			//if(abs(velocity_robot[1]) > 40 ){
					//if(velocity_robot[1] < 0)	velocity_robot[1] = -40;
					//else if(velocity_robot[1] > 0)	velocity_robot[1] = 40;
				//}
		}
		if(!BodyPidFlag)
			velocity_robot[1] = 0;
		//ramp(X_Axis,direction);
		velocity_robot[0] = 80;
		velocity_robot[2] = 0;
		calculatevel();	
	
}

//void movy(int distance,int direction)
//{
		//distanceY = distance;
		//if(BodyPidFlag){
			//encoderx.SETPOINT = 0;
			//encoderx.input = encoderX.getdistance();
			//encoderx.error = encoderx.SETPOINT - encoderx.input;
			//encoderx.Iterm += kii * encoderx.error;
			//encoderx.output = kpp * encoderx.error + encoderx.Iterm -kdd*(encoderx.input - encoderx.prevInput);
			//velocity_robot[0] = encoderx.output;			//for reverse x compensation, +ve sign
			//if(abs(velocity_robot[0]) > 40){
				//if(velocity_robot[0] < 0)	velocity_robot[0] = -40;
				//else if (velocity_robot[0] > 0)	velocity_robot[0] = 40;
			//}
		//}
		//else
			//velocity_robot[0] = 0;
		//ramp(Y_Axis,direction);
		//velocity_robot[2] = 0;
		//calculatevel();
		//
//}

inline void incrkpp(){kpp += 0.01;}
inline void dcrkpp(){kpp -= 0.01;}
inline void incrkii(){kii += 0.00001;}
inline void dcrkii(){kii -= 0.00001;}
inline void incrkdd(){kdd += 0.0005;}
inline void dcrkdd(){kdd -= 0.0005;}
inline float getkpp(void){return kpp;}
inline float getkii(void){return kii;}
inline float getkdd(void){return kdd;}


#endif /* DRIVE_H_ */