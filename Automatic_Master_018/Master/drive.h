/*
 * drive.h
 *
 * Created: 10/23/2017 3:16:43 PM
 *  Author: abheesh
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#define PI		3.14159265
#define START_BYTE	127

#include "uart.h"
#include "encoder.h"
#include "Flags.h"
#include "qmccompass.h"
#include <util/delay.h>
#include <math.h>


extern encoder encoderY, encoderX;

////////////////////////////////////////////////////////////////////////////////
  extern bool PidUpdateFlagCompass;
  extern bool PidUpdateFlagDriveX;
  extern bool PidUpdateFlagLinetracker;
  extern bool PidUpdateFlagDriveY;
//////////////////// For Motor variables //////////////////////
signed char bufferMotorSpeed[4] = {0,0,0,0};
int velocity_motor[4];
int velocity_robot[3];
float coupling_matrix[4][3] = {{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};  //0.707
double value = PI / 180;
/////////////////////////// Other variables //////////////////////////////////

int Compass_angle;
int distanceX;
int distanceY;
int currentDistance;
int difference;
bool fullspeed = false;

int speed = 60;
int previousLineTrackerData = 35;
bool junctionX=false;
bool junctionY = false;
bool stopflag = false;
unsigned int count = 0;

//////////////////////// For Linetracker//////////////////////////////////
static uint8_t linestate = 0;
static int linetracker_data = 0;
static double totalSum = 0;
static double totalLine = 0;
static int lineBit[8];
static int weight[8] = {10,20,30,40,50,60,70,80};

////////////////////////////////////////////////////////////////////////////////
enum direction{
	X_Axis,
	Y_Axis,
	Front,
	Back
};
////////////////////////////////////////////////////////////////////////////////


struct bodyPid{ 
	bodyPid():FirstData(true){};
	int input,error,prevInput,output;
	bool leftedgeleft = false;
	bool rightedgeleft = false;
	double Iterm;
	int SETPOINT;	
	bool FirstData = true;
	int Max_output;
	int Min_output;
	double kp, ki, kd;
	
	void setPid(float p, float i, float d){
		kp = p;
		ki = i;
		kd = d;
	}
	void Set_Max_Min_Output (int Max_dum, int Min_dum)
	{
		Max_output = Max_dum;
		Min_output = Min_dum;
	}
	inline void incrkp(){kp += 0.1;}
	inline void dcrkp(){kp -= 0.1;}
	inline void incrki(){ki += 0.01;}
	inline void dcrki(){ki -= 0.01;}
	inline void incrkd(){kd += 0.5;}
	inline void dcrkd(){kd -= 0.5;}
	inline void incrSetpoint(){SETPOINT += 5;}
	inline void dcrSetpoint(){SETPOINT -= 5;}
	inline int getSETPOINT(){return SETPOINT;}
	inline float getkp(void){return kp;}
	inline float getki(void){return ki;}
	inline float getkd(void){return kd;}
	
};
bodyPid ltX,ltY,compass,driveX,driveY;


/////////////////////////////////////////////////////////
void calculateCompassPID(void);
void calculatevel();
int filterLineTrackerData(int);
void calculateLineTrackerYPid();
void movx();
void movy();
void movYForwardSlow();
void initializeAll();
void sendDataToSlave();
void BrakeMotor();
int getLineTrackerYdata();
////////////////////////////////////////////////////

void BrakeMotor(){
	PORTK ^= (1<<PK1);
}

void sendDataToSlave(void){
//	uart0_puts("a");
// 	I2C_Start(0x20);
// 	I2C_Write_byte_array(bufferMotorSpeed,4);
// 	I2C_Stop();
uart2_putc(START_BYTE);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[0]);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[1]);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[2]);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[3]);
/*_delay_ms(1);*/
}

int getLineTrackerYdata(void){
	for(int i = 0; i <= 7; i++){
		if(bit_is_set(PINC,i)){
			lineBit[i] = 1;
			linestate |= (1<<i);
		}
		else{
			lineBit[i] = 0;
		}
		totalSum += weight[i]*lineBit[i];
		totalLine += lineBit[i];
	}
	linetracker_data = totalSum/totalLine;
	totalSum = 0;
	totalLine = 0;
	return linetracker_data;
}


void calculateCompassPID(void)
{
	if(PidUpdateFlagCompass && compassPID)
	{
		
		compass.input =get_Angle();
		
		
		//uart0_putint(compass.input);
// 		if (compass.FirstData)
// 		{
// 			compass.SETPOINT = compass.input;
// 			compass.FirstData = false;
// 		}
		
		compass.error = compass.SETPOINT	-	compass.input;

		if (compass.error > 180)
		{
			compass.error = compass.error - 360;
		}
		else if (compass.error < -180)
		{
			compass.error = compass.error + 360;
		}
	
		compass.Iterm += compass.ki*compass.error;

		if (abs(compass.Iterm) > 0.1*compass.Max_output)
		{
			if(compass.Iterm > 0)
				compass.Iterm = 0.1*compass.Max_output;
			else
				compass.Iterm = -0.1*compass.Max_output;
		}
		compass.output = compass.kp*compass.error	-	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;

		compass.prevInput = compass.input;
		//uart0_puts("\tprevInput= ");
		//uart0_putint(compass.prevInput);
		
		if (abs(compass.output) > compass.Max_output)
		{
			compass.output = (compass.output > compass.Max_output) ?	compass.Max_output : -compass.Max_output;
		}

		velocity_robot[2] = -compass.output;
		
		PidUpdateFlagCompass = false;
	}
	
	if(!compassPID){
		velocity_robot[2] = 0;
	}
}


void calculatevel()	//use matrix to find setpoint of individual motor and store in bufferMotorSpeed and send to slave
{
	for(int i=0;i<4;i++)
	{
		velocity_motor[i] = 0;
		for(int j=0;j<3;j++)
		{
			velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
			
		}
	}
	bufferMotorSpeed[0] = ((velocity_motor[0]) * 46)/249;	  
	bufferMotorSpeed[1] = ((velocity_motor[1]) * 46)/249;	  
	bufferMotorSpeed[2] = ((velocity_motor[2]) * 46)/249;	  
	bufferMotorSpeed[3] = ((velocity_motor[3]) * 46)/249 ;
	
	sendDataToSlave();  
}		


int filterLineTrackerData(int newData)
{
	int data;
	
	if(previousLineTrackerData == 70 && newData >= 255)
		data = 255;
	else if(previousLineTrackerData == 0 && newData >= 255)
		data = 255;
	else if(previousLineTrackerData >= 0 && previousLineTrackerData <= 70 && abs(newData) > 70)
		data = previousLineTrackerData;
	else
		data = newData;
	previousLineTrackerData = data;
	
	return data;
	
}


void calculateLineTrackerYPid()
{
 	if(ltY.FirstData && getLineTrackerYdata() != 0){
 		ltY.prevInput = getLineTrackerYdata();
 		ltY.FirstData = false;
 	}
	else if(PidUpdateFlagLinetracker && linetrackerPID){
		ltY.input = getLineTrackerYdata();
		if(ltY.input == 0 && ltY.prevInput < 45)
		{
			//uart0_puts("leftedgeleft\n");
			ltY.leftedgeleft = true;
			ltY.rightedgeleft = false;
		}
		else if(ltY.input == 0 && ltY.prevInput > 45)
		{
			//uart0_puts("right edge left\n");
			ltY.rightedgeleft = true;
			ltY.leftedgeleft = false;
		}
		if(ltY.input != 0){
			//uart0_puts("calculating\n");
			ltY.error = ltY.SETPOINT - ltY.input;
			if((ltY.error) == 0)
			{
				ltY.Iterm = 0;
			}
			if(ltY.error == 0)
				ltY.prevInput = ltY.input;
			ltY.Iterm += ltY.ki * ltY.error;
			if(abs(ltY.Iterm) > 10){
				if(ltY.Iterm > 0)	ltY.Iterm = 10;
				else if(ltY.Iterm < 0)	ltY.Iterm = -10;
			}
			ltY.output = ltY.kp * ltY.error + ltY.Iterm - ltY.kd *(ltY.input - ltY.prevInput);
			ltY.prevInput = ltY.input;
			if (abs(ltY.output) > 80)
			{
				if (ltY.output > 0){ltY.output = 80;}
				else{ltY.output = -80;}
			}
			velocity_robot[0] = -ltY.output;
		}
		else if (ltY.rightedgeleft){
			velocity_robot[0] = 40;
			velocity_robot[1] = 0;
		}
		else if(ltY.leftedgeleft){
			velocity_robot[0] = -40;
			velocity_robot[1] = 0;
		}
		PidUpdateFlagLinetracker = false;
		
	}
	if(!linetrackerPID)
		velocity_robot[0] = 0;
	
}



void initializeAll()
{
	compass.Set_Max_Min_Output(40,0);
	
	compass.SETPOINT = 7;
	ltX.SETPOINT = 35;
	ltY.SETPOINT = 45;
	compass.setPid(2.1,0.04,32);	//5.5, 0, 500
	ltX.setPid(4.0,0.003,488);			//1.7, 0, 17
	ltY.setPid(0.68,0.05,348);
	driveX.setPid(0.15,0,1.5);		
	driveY.setPid(0.15,0,1.5);
	init_QMC5883L();
	//drive - 0.15, 0.00, 1.5
	//lt.setpid = 4.0, 0.003, 450
	//compass.setpid = 8.9 , 488, 0.008
}

void movx(int distance_setpoint, int direction){
	//compass.setPid(2.1,0.04,32);
	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveX)
	{
		driveX.input = distanceX;
		PidUpdateFlagDriveX = false;
		if(distanceX > 200){
			driveX.error = driveX.SETPOINT - driveX.input;
			driveX.Iterm += driveX.ki * driveX.error;
			if(abs(driveX.Iterm) > 30){
				if(driveX.Iterm > 0)	driveX.Iterm = 30;
				if(driveX.Iterm < 0)	driveX.Iterm = -30;
			}
			if((driveX.error) > 0){
				driveX.output = driveX.kp * driveX.error + driveX.Iterm - driveX.kd*(driveX.input - driveX.prevInput);
			}
			else{
				driveX.output = 0;
			}
			driveX.prevInput = driveX.input;
			//////////////////////////////////////////////////////
			if(abs(driveX.output) > 100){
				if(driveX.output >0)	driveX.output = 100;
				else						driveX.output = -100;
			}
// 			if(abs(driveX.output) < 30){
// 				if(driveX.output >= 0)	driveX.output = 30;
// 				else					driveX.output = -30;
// 			}
			//////////////////////////////////////////////////////
			velocity_robot[0] = driveX.output;
		}
		else{
				velocity_robot[0] = 20 + 0.4*distanceX;
		}
		if(direction == Front){
			velocity_robot[0] = velocity_robot[0];
		}
		else if(direction == Back){
			velocity_robot[0] = -abs(velocity_robot[0]);
		}
	
	}
	velocity_robot[1] = 0;
	velocity_robot[2] = 0;
	//calculateCompassPID();
	calculatevel();
}

void movy(int distance_setpoint, int direction){
	//compass.setPid(2.1,0.04,32);
	distanceY = abs(encoderY.getdistance());
	driveY.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveY)
	{
			PidUpdateFlagDriveY = false;
			if(distanceY > 200)
			{
				driveY.input = distanceY;
				driveY.error = driveY.SETPOINT - driveY.input;
				driveY.Iterm += driveY.ki * driveY.error;
				if(abs(driveY.Iterm) > 30){
					if(driveY.Iterm > 0)	driveY.Iterm = 30;
					if(driveY.Iterm < 0)	driveY.Iterm = -30;
				}
				if(driveY.error > 0){
					driveY.output = driveY.kp * driveY.error + driveY.Iterm - driveY.kd*(driveY.input - driveY.prevInput);
				}
				else{
					driveY.output = 0;
				}
				driveY.prevInput = driveY.input;
				////////////////////////////////////////////////////////////
				if(abs(driveY.output) > 100){
					if(driveY.output >100)	driveY.output = 100;
					else						driveY.output = -100;
				}
				if(abs(driveY.output) < 20){
					if(driveY.output >= 0)	driveY.output = 20;
					else					driveY.output = -20;
				}
				/////////////////////////////////////////////////////////
				velocity_robot[1] = driveY.output;
			}
			else
			{
				velocity_robot[1] = 40 + (distanceY*0.3);
			}
			if(direction == Front){
				velocity_robot[1] = velocity_robot[1];
			}
			else if(direction == Back){
				velocity_robot[1] = -abs(velocity_robot[1]);
			}
		
	}
	//velocity_robot[0] = 0;
	calculateLineTrackerYPid();
	velocity_robot[2] = 0;
	//calculateCompassPID();
	calculatevel();
}

void movYForwardSlow(){
	//compass.setPid(2.1,0.04,32);
	velocity_robot[1] = 30;
	velocity_robot[0] = 0;
	velocity_robot[2] = 0;
	//calculateLineTrackerYPid();
	//calculateCompassPID();
	calculatevel();
}

void holdposition(){
	compass.setPid(5.5,0,500);
	calculateCompassPID();
	calculatevel();
}


#endif /* DRIVE_H_ */