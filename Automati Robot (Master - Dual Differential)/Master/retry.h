/*
 * retry.h
 *
 * Created: 7/20/2018 12:17:51 AM
 *  Author: Abheesh Khanal
 */ 


#ifndef RETRY_H_
#define RETRY_H_

#include "headers.h"

//pins to check initial condition of robot
#define M_MAIN_SWITCH			F,0
#define M_DIRECTLY_LZ2			F,1
#define M_DIRECTLY_TZ3			F,2
#define M_LZ2FORTZ3				F,3
#define M_ALWAYSTZ2				F,4
#define M_ALWAYSTZ1				F,5

//function to check initial condition of robot
void checkRobotMotion(void);

#endif /* RETRY_H_ */