/*
 * zonedrive.h
 *
 * Created: 12/20/2017 5:38:00 PM
 *  Author: abheesh
 */ 


#ifndef ZONEDRIVE_H_
#define ZONEDRIVE_H_

#include <util/delay.h>
#include "drive.h"
#include "pid.h"
#include "encoder.h"
#include "headers.h"

#define SHUTTLECOCK_STATUSPORT		PINF
#define SHUTTLECOCK_STATUSPIN		PINF0	
#define ZONE_STATUSPORT				PINF
#define ZONE_STATUSPIN				PINF1

#define SHUTTLECOCKPIN				F,0
#define ZONEPIN						F,1	


/////////////////////////////////////////
bool ShuttleCockGiven = false;
bool GoThrowingZone1 = false;
bool GoThrowingZone2 = false;
bool GoThrowingZone3 = false;
/////////////////////////////////////////
bool backtoLZ1 = false;
bool backtoLZ2 = false;
////////////////////////////////////////
enum{
	inStart_point,
	inTZ1,
	inTZ2,
	inTZ3,
	inLZ1,
	inLZ2,
	moving,
	notmoving
	};

struct coordinates{
	int x;
	int y;
	};

const struct coordinates Throwingzone1 = {4500,3000};
const struct coordinates Throwingzone2 = {6500,3000};
const struct coordinates Throwingzone3 = {6500,6260};
struct coordinates currentPosition;

unsigned int where = inStart_point;
unsigned int robotState = notmoving;
void updateZoneflag();

void gorockthegamefield(void)
{
	if((where == inLZ1 || where == inLZ2) && (robotState == notmoving))
		updateZoneflag();
	//////////////AT START GO TO LOADING ZONE 1/////////////////////////
	if(where == inStart_point){
		movx(Throwingzone1.x,Front);
		robotState = moving;
		if(abs(encoderX.getdistance()) >= Throwingzone1.x){
			stopDrive();
			robotState = notmoving;
			where = inLZ1;
			encoderX.resetCount();
			encoderY.resetCount();
		}
	}
	///////////// REACHED LOADING ZONE 1///////////////////////////////
	if(ShuttleCockGiven)
	{
		if(GoThrowingZone1)
		{
			movy(Throwingzone1.y,Front);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= Throwingzone1.y){
				stopDrive();
				robotState = notmoving;
				backtoLZ1 = true;
				where = inTZ1;
				encoderX.resetCount();
				encoderY.resetCount();
				GoThrowingZone1 = false;
				_delay_ms(500);
			}
		}
		if(backtoLZ1 && where == inTZ1)
		{
			movy(Throwingzone1.y,Back);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= Throwingzone1.y){
				stopDrive();
				robotState = notmoving;
				ShuttleCockGiven = false;
				backtoLZ1 = false;
				where = inLZ1;
				encoderX.resetCount();
				encoderY.resetCount();
			}
		}
		if(GoThrowingZone2)
		{
			if(where == inLZ1){
				movx((Throwingzone2.x - Throwingzone1.x),Front);
				robotState = moving;
				if(abs(encoderX.getdistance()) >= (Throwingzone2.x - Throwingzone1.x)){
					stopDrive();
					where = inLZ2;
					encoderX.resetCount();
					encoderY.resetCount();
				}
			}
			else if(where == inLZ2)
			{
				movy(Throwingzone2.y,Front);
				robotState = moving;
				if(abs(encoderY.getdistance()) >= Throwingzone2.y){
					stopDrive();
					robotState = notmoving;
					where = inTZ2;
					backtoLZ2 = true;
					encoderX.resetCount();
					encoderY.resetCount();
					GoThrowingZone2 = false;
					_delay_ms(500);
				}
			}
		}
		if(backtoLZ2 && where == inTZ2)
		{
			movy(Throwingzone2.y,Back);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= Throwingzone2.y){
				stopDrive();
				robotState = notmoving;
				where = inLZ2;
				encoderX.resetCount();
				encoderY.resetCount();
				ShuttleCockGiven = false;
			}
		}
	
		 if(GoThrowingZone3)
		 {
			movy(Throwingzone3.y,Front);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= Throwingzone3.y){
				stopDrive();
				robotState = notmoving;
				where = inTZ3;
				encoderX.resetCount();
				encoderY.resetCount();
				_delay_ms(500);
				//GoThrowingZone3 = false;
				//return 0;
			}
		}
	}
}

void updateZoneflag(void)
{
	INPUT(SHUTTLECOCKPIN);
	INPUT(ZONEPIN);
	SET(SHUTTLECOCKPIN);
	SET(ZONEPIN);
	
	////if low on shuttlecock pin then shuttlecock received///////////////
	///i.e if obstacle on shuttlecock IR ////////////////////////////////	
	if((SHUTTLECOCK_STATUSPORT & (1<<SHUTTLECOCK_STATUSPIN)) && (where == inLZ2 || where == inLZ1) ){
		ShuttleCockGiven = false;
		}	//shuttlecock given
	else{
		ShuttleCockGiven = true;
	}
	
	/////if low on zone pin then go to the next zone//////////////////////
	if((ZONE_STATUSPORT & (1<<ZONE_STATUSPIN)) && (where == inLZ1) ){			//if low on zone signal go next throwingzone2	
		GoThrowingZone1 = true;
		GoThrowingZone2 = false;
	}
	else if((!(ZONE_STATUSPORT & (1<<ZONE_STATUSPIN))) && (where == inLZ1)){	//if high on zone signal repeat throwingzone1
		GoThrowingZone1 = false;
		GoThrowingZone2 = true;
	}
	else if((ZONE_STATUSPORT & (1<<ZONE_STATUSPIN)) && (where == inLZ2)){		//if low on zone signal next go throwing zone 3
		GoThrowingZone2 = true;
		GoThrowingZone3 = false;
	}
	else if((!(ZONE_STATUSPORT & (1<<ZONE_STATUSPIN))) && (where == inLZ2)){
		GoThrowingZone2 = false;
		GoThrowingZone3 = true;
	}
	
	
}

#endif /* ZONEDRIVE_H_ */
	