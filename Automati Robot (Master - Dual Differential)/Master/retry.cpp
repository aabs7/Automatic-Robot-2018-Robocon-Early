/*
 * retry.cpp
 *
 * Created: 7/20/2018 12:51:08 AM
 *  Author: Abheesh Khanal
 */ 
 #include "retry.h"
 
 bool mainSwitchOn			= false;
 bool directlyLZ2			= false;
 bool directlyTZ3			= false;
 bool LZ2ForTZ3				= false;
 bool alwaysTZ2				= false;
 bool alwaysTZ1				= false;
 bool normalGame			= false;

 
void checkRobotMotion(){
	//Wait while main switch is not clicked//
 	while(READ(M_MAIN_SWITCH));
 	////after main switch is clicked check the state of retry conditions//
 	if(!READ(M_DIRECTLY_LZ2))			{directlyLZ2 = true;}
 	else if(!READ(M_DIRECTLY_TZ3))		{directlyTZ3 = true;}
 	else if(!READ(M_LZ2FORTZ3))			{LZ2ForTZ3   = true;}
 	else if(!READ(M_ALWAYSTZ2))			{alwaysTZ2   = true;}
 	else if(!READ(M_ALWAYSTZ1))			{alwaysTZ1   = true;}
 	else								{normalGame = true;}
}
