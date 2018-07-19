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


#define STARTZONEtoCORNER	200
#define CORNERtoLZ1			30
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
#include <util/delay.h>
#include <math.h>

extern uint8_t change;
extern encoder encoderY, encoderX;

unsigned long startTime;


/*****************************Limit switch pins*************************/
#define RIGHT_LIMIT_SW F,1
#define LEFT_LIMIT_SW F,2
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

bool lineMeet = true;
bool movingxfront = false;
bool movingxback = false;
bool movingyfront = false;
bool movingyback = false;
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
int filterLineTrackerData(int);
void calculateLineTrackerYPid();
void movx();
void movy();
void movYForwardSlow();
void initializeAll();
void sendDataToSlave();
void BrakeMotor();
int getLineTrackerYBackData();
int getLineTrackerYFrontData();

inline void linetrackerXjunctionWatch();
inline void lintrackerYjunctionWatch();

inline void linetrackerXjunctionWatchOff();
inline void linetrackerYjunctionWatchOff();

void holdposition();
void ramp(int, int);
////////////////////////////////////////////////////

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



int getLineTrackerYBackData(void){
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

int getLineTrackerYFrontData(void){
	int data = uart2_getc();
	if(data == 255){
		data = 0;
		return(data);
	}
	else 
		return (data + 10);
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

bool Stable_Robot(void)
{
	holdposition();
	uint16_t _get_angle = 0;
	//uart0_puts
	//uart0_putint(millis());
	//uart0_puts("\t");
	//uart0_putint(millis_time_then);
	if ((millis() - millis_time_then) > 1)
	{
		_get_angle = getYawGY88();
		//uart3_putint(_get_angle);
		//uart3_puts("\r\n");
		if(_get_angle<=(compass.SETPOINT+1) && _get_angle>=(compass.SETPOINT-1))
		{
			stable_data_count++;
			//uart3_putint(stable_data_count);
			//uart3_putc('\t');
		}
		else{
			stable_data_count = 0;
		}
		if (stable_data_count == 100)
		{
			//uart3_puts("Stable data \r\n");
			stable_data_count = 0;
			return 1;
		}
		//else return 0;
		millis_time_then = millis();
	}
	
	return 0;
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
		velocity_motor[0] = 10;
		velocity_motor[1] = 0;
		velocity_motor[2] = 0;
		velocity_motor[3] = -10;
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
		
		if (abs(compass.error) < 4 && abs(compass.error) > 0)
		{
			compass.output = compass.kp*compass.error	-	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;
		}
		else if(abs(compass.error) >= 4){
			compass.output = compass.kp * compass.kp * compass.error -	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;
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



void calculateLineTrackerYFrontPid()
{
 	if(ltYFront.FirstData && getLineTrackerYFrontData() != 0){
 		ltYFront.prevInput = getLineTrackerYFrontData();
 		ltYFront.FirstData = false;
 	}
	else if(PidUpdateFlagLinetrackerFront && linetrackerPID){
		ltYFront.input = getLineTrackerYFrontData();
		//if linetracker input is not zero///
		if(ltYFront.input != 0 && lineMeet){
			//uart0_puts("calculating\n");
			ltYFront.error = ltYFront.SETPOINT - ltYFront.input;
			if((ltYFront.error) == 0)
			{
				ltYFront.Iterm = 0;
			}
			if(ltYFront.error == 0)
				ltYFront.prevInput = ltYFront.input;
			ltYFront.Iterm += ltYFront.ki * ltYFront.error;
			if(abs(ltYFront.Iterm) > 10){
				if(ltYFront.Iterm > 0)	ltYFront.Iterm = 10;
				else if(ltYFront.Iterm < 0)	ltYFront.Iterm = -10;
			}
			ltYFront.output = ltYFront.kp * ltYFront.error + ltYFront.Iterm - ltYFront.kd *(ltYFront.input - ltYFront.prevInput);
			ltYFront.prevInput = ltYFront.input;
			if (abs(ltYFront.output) > 40)
			{
				if (ltYFront.output > 0){ltYFront.output = 40;}
				else{ltYFront.output = -40;}
			}
			velocity_robot[0] = -ltYFront.output;
		}

		PidUpdateFlagLinetrackerFront = false;
		
	}
	if(!linetrackerPID)
		velocity_robot[0] = 0;
	
}

void calculateLineTrackerYBackPid()
{
	if(ltYBack.FirstData && getLineTrackerYBackData() != 0){
		ltYBack.prevInput = getLineTrackerYBackData();
		ltYBack.FirstData = false;
	}
	else if(PidUpdateFlagLinetrackerBack && linetrackerPID){
		ltYBack.input = getLineTrackerYBackData();
		//if linetracker input is not zero///
		if(ltYBack.input != 0 && lineMeet){
			//uart0_puts("calculating\n");
			ltYBack.error = ltYBack.SETPOINT - ltYBack.input;
			if((ltYBack.error) == 0)
			{
				ltYBack.Iterm = 0;
			}
			if(ltYBack.error == 0)
			ltYBack.prevInput = ltYBack.input;
			ltYBack.Iterm += ltYBack.ki * ltYBack.error;
			if(abs(ltYBack.Iterm) > 10){
				if(ltYBack.Iterm > 0)	ltYBack.Iterm = 10;
				else if(ltYBack.Iterm < 0)	ltYBack.Iterm = -10;
			}
			ltYBack.output = ltYBack.kp * ltYBack.error + ltYBack.Iterm - ltYBack.kd *(ltYBack.input - ltYBack.prevInput);
			ltYBack.prevInput = ltYBack.input;
			if (abs(ltYBack.output) > 40)
			{
				if (ltYBack.output > 0){ltYBack.output = 40;}
				else{ltYBack.output = -40;}
			}
			velocity_robot[0] = -ltYBack.output;
		}

		PidUpdateFlagLinetrackerBack = false;
		
	}
	if(!linetrackerPID)
		velocity_robot[0] = 0;
	
}



void initializeAll()
{
	
	compass.Set_Max_Min_Output(40,0);	
	
	ltYFront.SETPOINT = 45;
	ltYBack.SETPOINT = 45;
	compass.setPid(5.5,0,500);//2,0,31);//4,0.09,18);	//5.5, 0, 500 , 2.1,0.04,32
	ltYFront.setPid(0.58,0.05,370);//0.58,0.05,370);
	ltYBack.setPid(0.58,0.05,370);
	driveX.setPid(0.15,0,0.9);		
	driveY.setPid(0.15,0,1);

 	//initGY88();
 	startTime = millis();
 	//uart0_puts("down loop \r\n");
//  	while((millis() - startTime) < 500){	//take 100 ms to set setpoint of compass
//   		initialCompassAngle = getYawGY88();
//  		//uart0_puts("1st \r\n");
//   		compass.FirstData = false;
//   		compass.SETPOINT = initialCompassAngle;
//   	}
	
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
		velocity_robot[1] = -10;
	}
	else{
		velocity_robot[1] = 0;
	}
	//velocity_robot[2] = 0;
	calculateCompassPID();
}

void movy(int distance_setpoint, int direction ,unsigned int speed){
	//compass.setPid(2.1,0.04,32);
	inverseKinematicsTrue = true;
	distanceY = abs(encoderY.getdistance());
	driveY.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveY)
	{
			PidUpdateFlagDriveY = false;
			if(distanceY >= 200)
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
				if(abs(driveY.output) >= speed){
					if(movingyfront)		driveY.output = speed;
					else if(movingyback)	driveY.output = -speed;
				}
   				if(abs(driveY.output) < 30){
   					if(movingyfront)		driveY.output = 30;
   					else if(movingyback)	driveY.output = -30;
   				}
				/////////////////////////////////////////////////////////
				velocity_robot[1] = driveY.output;
			}
			else
			{
				velocity_robot[1] = 60 + (distanceY*0.45);
			}
			if(direction == Front){
				movingyfront = true;
				movingyback = false;
				movingxfront = false;
				movingxback = false;
				velocity_robot[1] = velocity_robot[1];
				calculateLineTrackerYFrontPid();
			}
			else if(direction == Back){
				movingyback = true;
				movingyfront = false;
				movingxfront = false;
				movingxback  = false;
				velocity_robot[1] = -abs(velocity_robot[1]);
				calculateLineTrackerYBackPid();
			}
		
	}

	calculateCompassPID();
}

void movxForwardSlow(unsigned int speed){
	inverseKinematicsTrue = true;
	movingxfront = true;
	movingyback = false;
	movingyfront = false;
	movingxback = false;
	velocity_robot[0] = speed;
	velocity_robot[1] = 0;
	calculateCompassPID();
}

void movYForwardSlow(unsigned int speed){
	inverseKinematicsTrue = true;
	movingxfront = false;
	movingxback = false;
	movingyfront = true;
	movingyback = false;
	compass.setPid(2,0,31);
	velocity_robot[1] = speed;
	velocity_robot[0] = 0;
	calculateLineTrackerYBackPid();
	calculateCompassPID();
}

void movYCornerToLoadingZone1(unsigned int speed){
	inverseKinematicsTrue = true;
	movingxfront = false;
	movingxback = false;
	movingyfront = true;
	movingyback = false;
	if(!givenReverseThrust){
		velocity_robot[1] = speed;
		if(abs(encoderY.getdistance())>= 500){
			unsigned long nowTime = millis();
			velocity_robot[1] = -40;
			velocity_robot[0] = 0;
			calculatevel();
			givenReverseThrust = true;
			while((millis() - nowTime) <= 100);	//give reverse thrust in motor for 100ms
		}
	}
	else{
		velocity_robot[1] = 30;
	}
	calculateLineTrackerYBackPid();
	//calculateLineTrackerYFrontPid();
	velocity_robot[2] = 0;
}

void holdposition(){
	inverseKinematicsTrue = true;
	velocity_robot[0]  = 0;
	velocity_robot[1] = 0;
	//velocity_robot[2] = 0;
	compass.setPid(4.2,0.24,32);	//5.1,0,31
	calculateCompassPID();
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
	calculateCompassPID();
	
}

#endif /* DRIVE_H_ */
