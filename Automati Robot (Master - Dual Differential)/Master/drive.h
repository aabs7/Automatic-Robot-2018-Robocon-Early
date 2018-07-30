/*
 * drive.h
 *
 * Created: 10/23/2017 3:16:43 PM
 *  Author: Abheesh Khanal, Prakash Chaudhary
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#define PI		3.14159265
#define START_BYTE	127


#define STARTZONEtoCORNER	200
#define CORNERtoLZ1			40
#define LZ1toTZ1			100
#define	TZ1toLZ1			100
#define LZ1toLZ2			100
#define LZ2toTZ2			100
#define	TZ2toLZ2			100
#define LZ2toTZ3			150

#include "uart.h"
#include "headers.h"
#include "encoder.h"
#include "Flags.h"
#include "gy88.h"
#include "retry.h"
#include <util/delay.h>
#include <math.h>

extern uint8_t change;
extern encoder encoderY, encoderX;

unsigned long startTime;


/*****************************Limit switch pins*************************/
#define RIGHT_LIMIT_SW H,3
#define LEFT_LIMIT_SW E,3
unsigned long time_of_limit_switches_pressed = 0;
bool first_data_time_of_limit_switches_pressed = true;
/********************************************************************/
uint16_t stable_data_count = 0;
unsigned long millis_time_then = 0;

////////////////////////////////////////////////////////////////////////////////
extern bool PidUpdateFlagCompass;
extern bool PidUpdateFlagDriveX;
extern bool PidUpdateFlagLinetrackerFront;
extern bool PidUpdateFlagLinetrackerBack;
extern bool PidUpdateFlagDriveY;
//////////////////// For Motor variables //////////////////////
signed char bufferMotorSpeed[4] = {0,0,0,0};
int velocity_motor[4];
int velocity_robot[3];
float coupling_matrix[4][3] = {{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};  //0.707
double value = PI / 180;
/////////////////////////// Other variables //////////////////////////////////

int distanceX;
int distanceY;
uint16_t initialCompassAngle;


bool startingAtFront = true;
bool inverseKinematicsTrue = true;
bool givenReverseThrust = false;
bool pressRobot = false;

bool lineMeet = true;
bool movingxfront = false;
bool movingxback = false;
bool movingyfront = false;
bool movingyback = false;
//////////////////////// For Linetracker//////////////////////////////////
static uint8_t linestate = 0;
static uint8_t linetracker_data = 0;
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
	inline void incrSetpoint(){SETPOINT += change;}
	inline void dcrSetpoint(){SETPOINT -= change;}
	inline int getSETPOINT(){return SETPOINT;}
	inline float getkp(void){return kp;}
	inline float getki(void){return ki;}
	inline float getkd(void){return kd;}
	
};
bodyPid ltYFront,ltYBack,compass,driveX,driveY;


/////////////////////////////////////////////////////////
void calculateCompassPID(void);
void calculatevel();
void movx();
void movYForwardSlow();
void initializeAll();
void sendDataToSlave();
void BrakeMotor();

inline void linetrackerXjunctionWatch();
inline void lintrackerYjunctionWatch();

inline void linetrackerXjunctionWatchOff();
inline void linetrackerYjunctionWatchOff();

////////////////////////////////////////////////////

//16th july - Dual Differential Drive
int8_t _direction_matrix[5][4] = {{-1, -1, 1, 1}, {1, 1, -1, -1}, {1, -1, -1, 1}, {-1, 1, 1, -1}, {0, 0, 0, 0}};
uint8_t _axis = 7, _direction = 7;
bool _front_linetracker_left_edge_left = false, _front_linetracker_right_edge_left = false;
bool _back_linetracker_left_edge_left = false, _back_linetracker_right_edge_left = false;
uint8_t _previous_data_of_front_linetracker = 35, _previous_data_of_back_linetracker = 45;
bodyPid FrontLinetrackerY_, BackLinetrackerY_;

uint8_t Get_Front_LinetrackerY_Data(void);
uint8_t Get_Back_LinetrackerY_Data(void);
void Calculate_Front_LinetrackerY_Pid(void);
void Calculate_Back_LinetrackerY_Pid(void);
void Calculate_Velocity(void);
void Calculate_Motor_Differential_Velocity_With_Center_Pivot(int d_speed);
void Calculate_Motor_Differential_Velocity_With_Wheel_Pivot(int d_speed);
void Move_Xaxis(int distance_setpoint, int direction, unsigned int speed);
void Move_Yaxis(int distance_setpoint, int direction, unsigned int speed);
void MovY_Slow(int distance_setpoint, int direction, unsigned int speed);
void Hold_Position(void);
/////////////////////////////////////


void initializeAll()
{
	
	compass.Set_Max_Min_Output(40,0);
	compass.setPid(5.5,0,500);//2,0,31);//4,0.09,18);	//5.5, 0, 500 , 2.1,0.04,32
	
	driveX.setPid(0.15,0,0.9);
	driveY.setPid(0.15,0,1);

	FrontLinetrackerY_.SETPOINT =45;
	BackLinetrackerY_.SETPOINT = 45;
	FrontLinetrackerY_.setPid(1.2,0,16);
	BackLinetrackerY_.setPid(1.2,0,16);
	
	initGY88();
	startTime = millis();
	//uart0_puts("down loop \r\n");
	while((millis() - startTime) < 500){	//take 100 ms to set setpoint of compass
		initialCompassAngle = getYawGY88();
		//uart0_puts("1st \r\n");
		compass.FirstData = false;
		compass.SETPOINT = initialCompassAngle;
	}
	checkRobotMotion();
	
}

inline void linetrackerXjunctionWatch(void){
	sei();
	PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT4);
}
inline void linetrackerYjunctionWatch(void){
	sei();
	PCICR |= (1<<PCIE2);
	PCMSK2 |= (1<<PCINT23);
}
inline void linetrackerXjunctionWatchOff(void){
	PCMSK0 &= ~(1<<PCINT4);
}
inline void linetrackerYjunctionWatchOff(void){
	PCMSK2 &= ~(1<<PCINT23);
}

void BrakeMotor(){
	PORTK ^= (1<<PK0);
	movingxfront = false;
	movingxback = false;
	movingyfront = false;
	movingyback = false;
}

void sendDataToSlave(void){
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

bool Goto_Fence_And_Detect(void)
{
	movingyfront = false;
	if (READ(RIGHT_LIMIT_SW) && !READ(LEFT_LIMIT_SW))
	{
		inverseKinematicsTrue = false;
		velocity_motor[0] = 30;
		velocity_motor[1] = 0;
		velocity_motor[2] = 0;
		velocity_motor[3] = -20;
		time_of_limit_switches_pressed = 0;
		first_data_time_of_limit_switches_pressed = true;
	}
	else if (READ(LEFT_LIMIT_SW) && !READ(RIGHT_LIMIT_SW))
	{
		inverseKinematicsTrue = false;
		velocity_motor[0] = 20;
		velocity_motor[1] = 0;
		velocity_motor[2] = 0;
		velocity_motor[3] = -30;
		time_of_limit_switches_pressed = 0;
		first_data_time_of_limit_switches_pressed = true;
	}
	else if (READ(LEFT_LIMIT_SW) && READ(RIGHT_LIMIT_SW))
	{
		inverseKinematicsTrue = true;
		velocity_robot[0] = -40;
		velocity_robot[1] = 0;
		velocity_robot[2] = 0;
		time_of_limit_switches_pressed = 0;
		first_data_time_of_limit_switches_pressed = true;
	}
	
	if (!READ(LEFT_LIMIT_SW) && !READ(RIGHT_LIMIT_SW))
	{
		inverseKinematicsTrue = false;
		if(!pressRobot){
			velocity_motor[0] = 0;
			velocity_motor[1] = 0;
			velocity_motor[2] = 0;
			velocity_motor[3] = 0;
			if (first_data_time_of_limit_switches_pressed)
			{
				time_of_limit_switches_pressed = millis();
				first_data_time_of_limit_switches_pressed = false;
			}
			if (millis() - time_of_limit_switches_pressed > 1)
			{
				return 1;
			}
		}
		else{
			velocity_motor[0] = 15;
			velocity_motor[1] = 0;
			velocity_motor[2] = 0;
			velocity_motor[3] = -15;
		}
	}

	return 0;
}

void calculateCompassPID(void)
{
	if(PidUpdateFlagCompass && compassPID)
	{
		
		compass.input = getYawGY88();
		
		
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
		
		if (abs(compass.error) > 1)
		{
			compass.output = compass.kp*compass.error	-	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;
		}
		else
		{
			compass.Iterm = 0;
			compass.output = 0;
		}
			
		compass.prevInput = compass.input;
		
		if (abs(compass.output) > compass.Max_output)
		{
			compass.output = (compass.output > compass.Max_output) ?	compass.Max_output : -compass.Max_output;
		}

		velocity_robot[2] = compass.output;
		
		PidUpdateFlagCompass = false;
	}
	
	if(!compassPID){
		velocity_robot[2] = 0;
	}
}


void calculatevel()	//use matrix to find setpoint of individual motor and store in bufferMotorSpeed and send to slave
{
	if(inverseKinematicsTrue){
		for(int i=0;i<4;i++)
		{
			velocity_motor[i] = 0;
			for(int j=0;j<3;j++)
			{
				velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
				
			}
		}
	}
	bufferMotorSpeed[0] = ((velocity_motor[0]) * 23)/249;	  
	bufferMotorSpeed[1] = ((velocity_motor[1]) * 23)/249;	  
	bufferMotorSpeed[2] = ((velocity_motor[2]) * 23)/249;	  
	bufferMotorSpeed[3] = ((velocity_motor[3]) * 23)/249 ;
	
	sendDataToSlave();  
}		


void movx(int distance_setpoint, int direction, unsigned int speed){
	//compass.setPid(2.1,0.04,32);
	inverseKinematicsTrue = true;
	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveX)
	{
		movingyfront = false;
		movingyback = false;
		driveX.input = distanceX;
		PidUpdateFlagDriveX = false;
		if(distanceX >= 1000){
			driveX.error = driveX.SETPOINT - driveX.input;
			driveX.Iterm += driveX.ki * driveX.error;
			if(driveX.FirstData){
				driveX.prevInput = driveX.input;
				driveX.FirstData = false;
			}
			if(abs(driveX.Iterm) > 10){
				if(driveX.Iterm > 0)	driveX.Iterm = 10;
				if(driveX.Iterm < 0)	driveX.Iterm = -10;
			}
			if((driveX.error) > 0){
				driveX.output = driveX.kp * driveX.error + driveX.Iterm - driveX.kd*(driveX.input - driveX.prevInput);
			}
			else{
				driveX.output = 0;
			}
			driveX.prevInput = driveX.input;
			//////////////////////////////////////////////////////
			if(abs(driveX.output) > speed){
				if(driveX.output >0)	driveX.output = speed;
				else						driveX.output = -speed;
			}
 			if(abs(driveX.output) < 20){
 				if(movingxfront)		driveX.output = 20;
 				else if(movingxback)	driveX.output = -20;
 			}
			//////////////////////////////////////////////////////
			velocity_robot[0] = driveX.output;
		}
		else{
			if(startingAtFront){	//if starting from front, use this function to ramp up
				velocity_robot[0] = 60 + 0.1*distanceX;
			}
			else{					//if going from loading zone 1 to loading zone 2 use this to ramp up
				velocity_robot[0] = 60 + 0.04 * distanceX;
			}
		}
		if(direction == Front){
			velocity_robot[0] = velocity_robot[0];
			movingxfront = true;
			movingxback = false;
			movingyback = false;
			movingyfront = false;
		}
		else if(direction == Back){
			velocity_robot[0] = -abs(velocity_robot[0]);
			movingxfront = false;
			movingxback = true;
			movingyfront = false;
			movingyback = false;
		}
	
	}
	if(startingAtFront){			//if starting from start zone then push the fence
		velocity_robot[1] = -20;
		velocity_robot[2] = 0;
	}
	else{
		velocity_robot[1] = 0;
		calculateCompassPID();
	}
}

void movYForwardSlow(unsigned int speed){
	
	FrontLinetrackerY_.setPid(0.6,0,16);
	BackLinetrackerY_.setPid(0.6,0,16);
	
	_axis = Y_Axis;
	_direction = Front;
	Calculate_Motor_Differential_Velocity_With_Center_Pivot(speed);
}

void movDegree(int degree)
{
	inverseKinematicsTrue = true;
	int speed = 60;
	int difference = 4700 - abs(encoderX.getdistance());
	
	if(abs(encoderX.getdistance()) < 1000){
		speed = 60 + 0.09 * abs(encoderX.getdistance());
	}
	else{
		speed = 150;
	}
	
	if(abs(encoderX.getdistance()) > 4000){
		speed = difference * 0.1;
		if(difference < 0){
			speed = 30;
		}
		if(speed < 30){
			speed = 30;
		}
	}

	velocity_robot[0] = (speed * float(cos(degree * DEG_TO_RAD)));
	velocity_robot[1] = (speed * float(sin(degree * DEG_TO_RAD)));
	velocity_robot[2] = 0;
	//calculateCompassPID();
	
}

// //16th july dual differential
	
uint8_t Get_Front_LinetrackerY_Data(void)
{
	uint8_t data = uart2_getc();
	
	if (data > 100  && _previous_data_of_front_linetracker != 80 && _previous_data_of_front_linetracker != 10 )
	{
		data = _previous_data_of_front_linetracker;
	}
	else
	{
		if (_previous_data_of_front_linetracker == 10 && data == 0)
		{
			_front_linetracker_left_edge_left = false;
		}
		
		if (_previous_data_of_front_linetracker == 80 && data == 70)
		{
			_front_linetracker_right_edge_left = false;
		}
		
		if (_previous_data_of_front_linetracker == 10 && data > 100)
		{
			data = 0;
			_previous_data_of_front_linetracker = 10;
			_front_linetracker_left_edge_left = true;
			
		}
		else if (_previous_data_of_front_linetracker == 80 && data > 100)
		{
			data = 90;
			_previous_data_of_front_linetracker = 80;
			_front_linetracker_right_edge_left = true;
		}
		else if (!_front_linetracker_left_edge_left && !_front_linetracker_right_edge_left)
		{
			data = data + 10;
			_previous_data_of_front_linetracker = data;
		}
		else
		{
			data = data + 10;
		}
		
	}
	
	return(data);
}

uint8_t Get_Back_LinetrackerY_Data(void)
{
		for(uint8_t i = 0; i <= 7; i++){
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
		
		if (_previous_data_of_back_linetracker == 10 && linetracker_data == 10)
		{
			_back_linetracker_left_edge_left = false;			
		}
		
		if (_previous_data_of_back_linetracker == 80 && linetracker_data == 80)
		{
			_back_linetracker_right_edge_left = false;
		}
		
		if (_previous_data_of_back_linetracker == 10 && linetracker_data == 0)
		{
			linetracker_data = 0;
			_previous_data_of_back_linetracker = 10;
			_back_linetracker_left_edge_left = true;
			
		}
		else if (_previous_data_of_back_linetracker == 80 && linetracker_data == 0)
		{
			linetracker_data = 90;
			_previous_data_of_back_linetracker = 80;
			_back_linetracker_right_edge_left = true;
		}
		else if (!_back_linetracker_left_edge_left && !_back_linetracker_right_edge_left)
		{
			_previous_data_of_back_linetracker = linetracker_data;
		}
		return linetracker_data;
}

void Calculate_Front_LinetrackerY_Pid(void)
{
	if(FrontLinetrackerY_.FirstData && Get_Front_LinetrackerY_Data() != 0){
		FrontLinetrackerY_.prevInput = Get_Front_LinetrackerY_Data();
		FrontLinetrackerY_.FirstData = false;
	}
	else if(PidUpdateFlagLinetrackerBack && linetrackerPID){
			FrontLinetrackerY_.input = Get_Front_LinetrackerY_Data();
			FrontLinetrackerY_.error = FrontLinetrackerY_.SETPOINT - FrontLinetrackerY_.input;
			
			FrontLinetrackerY_.Iterm += FrontLinetrackerY_.ki * FrontLinetrackerY_.error;	

			if(abs(FrontLinetrackerY_.Iterm) > 5){
				if(FrontLinetrackerY_.Iterm > 0)	FrontLinetrackerY_.Iterm = 5;
				else if(FrontLinetrackerY_.Iterm < 0)	FrontLinetrackerY_.Iterm = -5;
			}
			if (FrontLinetrackerY_.error == 0)
			{
				FrontLinetrackerY_.Iterm = 0;
				FrontLinetrackerY_.output = 0;
			}
			else
			{
				FrontLinetrackerY_.output = FrontLinetrackerY_.kp * FrontLinetrackerY_.error + FrontLinetrackerY_.Iterm - FrontLinetrackerY_.kd *(FrontLinetrackerY_.input - FrontLinetrackerY_.prevInput);
			}
			
			FrontLinetrackerY_.prevInput = FrontLinetrackerY_.input;
			
			if (abs(FrontLinetrackerY_.output) > 80)
			{
				if (FrontLinetrackerY_.output > 0){FrontLinetrackerY_.output = 80;}
				else{FrontLinetrackerY_.output = -80;}
			}

		PidUpdateFlagLinetrackerFront = false;

	}
	if(!linetrackerPID)
	FrontLinetrackerY_.output = 0;

}

void Calculate_Back_LinetrackerY_Pid(void)
{
	if(BackLinetrackerY_.FirstData && Get_Back_LinetrackerY_Data() != 0){
		BackLinetrackerY_.prevInput = Get_Back_LinetrackerY_Data();
		BackLinetrackerY_.FirstData = false;
	}

	if(PidUpdateFlagLinetrackerBack && linetrackerPID){
			BackLinetrackerY_.input = Get_Back_LinetrackerY_Data();
		
			BackLinetrackerY_.error = BackLinetrackerY_.SETPOINT - BackLinetrackerY_.input;
			
			BackLinetrackerY_.Iterm += BackLinetrackerY_.ki * BackLinetrackerY_.error;
			
			if(abs(BackLinetrackerY_.Iterm) > 5){
				if(BackLinetrackerY_.Iterm > 0)	BackLinetrackerY_.Iterm = 5;
				else if(BackLinetrackerY_.Iterm < 0)	BackLinetrackerY_.Iterm = -5;
			}
			
			if (BackLinetrackerY_.error == 0)
			{
				BackLinetrackerY_.Iterm = 0;
				BackLinetrackerY_.output = 0;
			}
			else
			{
				BackLinetrackerY_.output = BackLinetrackerY_.kp * BackLinetrackerY_.error + BackLinetrackerY_.Iterm - BackLinetrackerY_.kd *(BackLinetrackerY_.input - BackLinetrackerY_.prevInput);
			}
			
			BackLinetrackerY_.prevInput = BackLinetrackerY_.input;
			
			if (abs(BackLinetrackerY_.output) > 80)
			{
				if (BackLinetrackerY_.output > 0){BackLinetrackerY_.output = 80;}
				else{BackLinetrackerY_.output = -80;}
			}

		PidUpdateFlagLinetrackerBack = false;

	}

	if(!linetrackerPID)
	BackLinetrackerY_.output = 0;

}

void Calculate_Velocity(void)
{
	uint8_t ii = 4;
	
	if (_axis == X_Axis)
	{
		if (_direction == Front){ ii = 0; }
		else if (_direction == Back) { ii = 1; }
	}
	else if (_axis == Y_Axis)
	{
		if (_direction == Front){ ii = 2; }
		else if (_direction == Back) { ii = 3; }
	}
	
	for (uint8_t jj = 0; jj < 4; jj++)
	{
		velocity_motor[jj] = velocity_motor[jj] * _direction_matrix[ii][jj];
	}
}

void Move_Xaxis(int distance_setpoint, int direction, unsigned int speed){
	// 	//compass.setPid(2.1,0.04,32);
	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveX)
	{
		movingyfront = false;
		movingyback = false;
		driveX.input = distanceX;
		PidUpdateFlagDriveX = false;
		if(distanceX >= 1000){
			driveX.error = driveX.SETPOINT - driveX.input;
			driveX.Iterm += driveX.ki * driveX.error;
			if(driveX.FirstData){
				driveX.prevInput = driveX.input;
				driveX.FirstData = false;
			}
			if(abs(driveX.Iterm) > 10){
				if(driveX.Iterm > 0)	driveX.Iterm = 10;
				if(driveX.Iterm < 0)	driveX.Iterm = -10;
			}
			if((driveX.error) > 0){
				driveX.output = driveX.kp * driveX.error + driveX.Iterm - driveX.kd*(driveX.input - driveX.prevInput);
			}
			else{
				driveX.output = 0;
			}
			driveX.prevInput = driveX.input;
			//////////////////////////////////////////////////////
			if(abs(driveX.output) > speed){
				if(driveX.output >0)	driveX.output = speed;
				else						driveX.output = -speed;
			}
			
			if(abs(driveX.output) < 20){
				if(movingxfront)		driveX.output = 20;
				else if(movingxback)	driveX.output = -20;
			}
		}
		////////////////////////////////////////////////////
		else{
			if(startingAtFront){	//if starting from front, use this function to ramp up
				velocity_robot[0] = 60 + 0.1*distanceX;
			}
			else{					//if going from loading zone 1 to loading zone 2 use this to ramp up
				velocity_robot[0] = 60 + 0.04 * distanceX;
			}
		}
	}
	//
	_axis = X_Axis;
	
	if(direction == Front){
		movingxfront = true;
		movingxback = false;
		movingyback = false;
		movingyfront = false;
		_direction = Front;
	}
	else if(direction == Back){
		movingxfront = false;
		movingxback = true;
		movingyfront = false;
		movingyback = false;
		_direction = Back;
	}

	//	}

	Calculate_Motor_Differential_Velocity_With_Center_Pivot(driveX.output);
}
//
void Move_Yaxis(int distance_setpoint, int direction ,unsigned int speed)
{
	FrontLinetrackerY_.setPid(1.2,0,16);
	FrontLinetrackerY_.setPid(1.2,0,16);
	
	distanceY = abs(encoderY.getdistance());
	driveY.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveY)
	{
		PidUpdateFlagDriveY = false;
		if(distanceY >= 600)
		{
			driveY.input = distanceY;
			driveY.error = driveY.SETPOINT - driveY.input;
			driveY.Iterm += driveY.ki * driveY.error;
			if(driveY.FirstData){
				driveY.prevInput = driveY.input;
				driveY.FirstData = false;
			}
			if(abs(driveY.Iterm) > 10){
				if(driveY.Iterm > 0)	driveY.Iterm = 10;
				if(driveY.Iterm < 0)	driveY.Iterm = -10;
			}
			if(driveY.error > 0){
				driveY.output = driveY.kp * driveY.error + driveY.Iterm - driveY.kd*(driveY.input - driveY.prevInput);
			}
			else{
				driveY.output = 0;
			}
			driveY.prevInput = driveY.input;
			////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////
		}
		else
		{
			driveY.output = 60 + (distanceY*0.15);
		}
		
		
		if(abs(driveY.output) >= speed){
			driveY.output = speed;
			//if(movingyfront)		driveY.output = speed;
			//else if(movingyback)	driveY.output = -speed;
		}
		if(abs(driveY.output) < 20){
		driveY.output = 20;
		}
		_axis = Y_Axis;
		if(direction == Front){
			movingyfront = true;
			movingyback = false;
			movingxfront = false;
			movingxback = false;
			_direction = Front;
		}
		else if(direction == Back){
			movingyback = true;
			movingyfront = false;
			movingxfront = false;
			movingxback  = false;
			_direction = Back;
		}
		
	}
	Calculate_Motor_Differential_Velocity_With_Center_Pivot(driveY.output);
	//Calculate_Motor_Differential_Velocity_With_Wheel_Pivot(driveY.output);
}


void Calculate_Motor_Differential_Velocity_With_Center_Pivot(int d_speed)
{
	inverseKinematicsTrue = false;
	Calculate_Front_LinetrackerY_Pid();
	Calculate_Back_LinetrackerY_Pid();
	
	if (_direction == Front)
	{
		velocity_motor[0] = d_speed + FrontLinetrackerY_.output;
		velocity_motor[1] = d_speed - FrontLinetrackerY_.output;
		velocity_motor[2] = d_speed + BackLinetrackerY_.output;
		velocity_motor[3] = d_speed - BackLinetrackerY_.output;
	}
	else
	{
		velocity_motor[0] = d_speed - FrontLinetrackerY_.output;
		velocity_motor[1] = d_speed + FrontLinetrackerY_.output;
		velocity_motor[2] = d_speed - BackLinetrackerY_.output;
		velocity_motor[3] = d_speed + BackLinetrackerY_.output;
	}
	
	Calculate_Velocity();
}

void Calculate_Motor_Differential_Velocity_With_Wheel_Pivot(int d_speed)
{
	inverseKinematicsTrue = false;
	Calculate_Front_LinetrackerY_Pid();
	Calculate_Back_LinetrackerY_Pid();
	
	if (_direction == Front)
	{
		if (FrontLinetrackerY_.output > 0)
		{
			velocity_motor[0] = d_speed + FrontLinetrackerY_.output;
			velocity_motor[1] = d_speed;
			velocity_motor[2] = d_speed  + BackLinetrackerY_.output;
			velocity_motor[3] = d_speed;
		}
		else
		{
			velocity_motor[0] = d_speed;
			velocity_motor[1] = d_speed - FrontLinetrackerY_.output;
			velocity_motor[2] = d_speed;
			velocity_motor[3] = d_speed - BackLinetrackerY_.output;
		}
	}
	else if (_direction == Back)
	{
		if (FrontLinetrackerY_.output > 0)
		{
			velocity_motor[0] = d_speed;
			velocity_motor[1] = d_speed + FrontLinetrackerY_.output;
			velocity_motor[2] = d_speed;
			velocity_motor[3] = d_speed + BackLinetrackerY_.output;
		}
		else
		{
			velocity_motor[0] = d_speed - FrontLinetrackerY_.output;
			velocity_motor[1] = d_speed;
			velocity_motor[2] = d_speed - BackLinetrackerY_.output;
			velocity_motor[3] = d_speed;
		}
	}
	
	Calculate_Velocity();
	
}

void MovY_Slow(int distance_setpoint, int direction, unsigned int speed)
{
	distanceY = abs(encoderY.getdistance());
	
	_axis = Y_Axis;
	
	if(direction == Front){
		movingyfront = true;
		movingyback = false;
		movingxfront = false;
		movingxback = false;
		_direction = Front;
	}
	else if(direction == Back){
		movingyback = true;
		movingyfront = false;
		movingxfront = false;
		movingxback  = false;
		_direction = Back;
	}
	
	if (distanceY > 500)
	{
		Calculate_Motor_Differential_Velocity_With_Center_Pivot(20);
	}
	else
	{
		Calculate_Motor_Differential_Velocity_With_Center_Pivot(speed);
	}
}


void Hold_Position(void)
{
	_axis = Y_Axis;
	_direction = Back;
	
	
 	FrontLinetrackerY_.setPid(1.8,0,10);
 	BackLinetrackerY_.setPid(1.8,0,10);
	
	Calculate_Motor_Differential_Velocity_With_Center_Pivot(0);
}

#endif /* DRIVE_H_ */