/*
 * pid.h
 *
 * Created: 10/22/2017 5:38:07 PM
 *  Author: abheesh
 */ 


#ifndef PID_H_
#define PID_H_

#include "Motor.h"
#include "encoder.h"
#include "uart.h"

int SETPOINT = 10;

extern Motor m1,m2,m3,m4;
extern encoder e1,e2,e3,e4;

struct pid
{
	double kp = 1  , ki , kd = 0.05 ;
	int input , output;
	int error;
	double Iterm;
	int previnput;
	int MOTOR_OCR_VALUE = 20 ;
	
	inline void incrKP(void){kp += 0.05;}
	inline void incrKD(void){kd += 0.05;}
	inline void incrKI(void){ki += 0.05;}
	inline void dcrKP(void){kp -= 0.05;}
	inline void dcrKI(void){ki -= 0.05;}
	inline void dcrKD(void){kd -= 0.05;}
	inline void reset(void){kp = 0;ki=0;kd=0;}
	inline float getkp(void){return kp;}
	inline float getkd(void){return kd;}
	inline float getki(void){return ki;}
	void setpid(int KP, int KI, int KD)
	{
		kp = KP;
		ki = KI;
		kd = KD;
	}

};

pid ma,mb,mc,md;



void setTuningsM1(void)
{
	ma.input = e1.speed;
	if(ma.input != 0){
	ma.error = SETPOINT - ma.input;
	
	ma.Iterm += ma.ki * ma.error;
	
	ma.output = ma.kp * ma.error + ma.Iterm - ma.kd * (ma.input - ma.previnput) ;
	
	ma.previnput = ma.input;
	ma.MOTOR_OCR_VALUE = ma.MOTOR_OCR_VALUE + ma.output;
	
	if (ma.MOTOR_OCR_VALUE > ICR_TOP)
		ma.MOTOR_OCR_VALUE = ICR_TOP;
	}
	else 
		ma.MOTOR_OCR_VALUE = 20;
	m1.SetOcrValue(ma.MOTOR_OCR_VALUE);
}
void setTuningsM2(void)
{
	mb.input = e2.speed;
	if(mb.input != 0){
	mb.error = SETPOINT - mb.input;
	
	mb.Iterm += mb.ki * mb.error;
	
	mb.output = mb.kp * mb.error + mb.Iterm - mb.kd * (mb.input - mb.previnput) ;
	
	mb.previnput = mb.input;
	mb.MOTOR_OCR_VALUE = mb.MOTOR_OCR_VALUE + mb.output;
	
	if (mb.MOTOR_OCR_VALUE > ICR_TOP)
	mb.MOTOR_OCR_VALUE = ICR_TOP;
	}
	else
		mb.MOTOR_OCR_VALUE = 20;
	m2.SetOcrValue(mb.MOTOR_OCR_VALUE);
}

void setTuningsM3(void)
{
	mc.input = e3.speed;
	if(mc.input!=0){
	mc.error = SETPOINT - mc.input;
	
	mc.Iterm += mc.ki * mc.error;
	
	mc.output = mc.kp * mc.error + mc.Iterm - mc.kd * (mc.input - mc.previnput) ;
	
	mc.previnput = mc.input;
	mc.MOTOR_OCR_VALUE = mc.MOTOR_OCR_VALUE + mc.output;
	
	if (mc.MOTOR_OCR_VALUE > ICR_TOP)
	mc.MOTOR_OCR_VALUE = ICR_TOP;
	}
	else
	mc.MOTOR_OCR_VALUE = 20;
	m3.SetOcrValue(mc.MOTOR_OCR_VALUE);

}
void setTuningsM4(void)
{
	
	md.input = e4.speed;
	if(md.input != 0){
	md.error = SETPOINT - md.input;
	
	md.Iterm += md.ki * md.error;
	
	md.output = md.kp * md.error + md.Iterm - md.kd * (md.input - md.previnput) ;
	
	md.previnput = md.input;
	md.MOTOR_OCR_VALUE = md.MOTOR_OCR_VALUE + md.output;

	if (md.MOTOR_OCR_VALUE > ICR_TOP)
		md.MOTOR_OCR_VALUE = ICR_TOP;
	}
	else
	md.MOTOR_OCR_VALUE = 20;
	m4.SetOcrValue(md.MOTOR_OCR_VALUE);
}




#endif /* PID_H_ */