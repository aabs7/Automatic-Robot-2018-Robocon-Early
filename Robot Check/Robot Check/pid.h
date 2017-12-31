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
#include "drive.h"
#include "Flags.h"

/////////////////////////////////////////////////
extern Motor m1,m2,m3,m4;
extern encoder e1,e2,e3,e4;
////////////////////////////////////////////////
double kp = 0.89  , ki , kd = 8.2 ; //0.89 -> p and 8.2 -> d

struct pid
{
	int input , output;
	int error;
	double Iterm;
	int previnput;
	int MOTOR_OCR_VALUE;
	
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
	ma.input = e1.getspeed();
	
		ma.error = SETPOINT1 - ma.input;
	
		ma.Iterm += ki * ma.error;
	
		ma.output = kp * ma.error + ma.Iterm - kd * (ma.input - ma.previnput) ;
	
		ma.previnput = ma.input;
		ma.MOTOR_OCR_VALUE = ma.MOTOR_OCR_VALUE + ma.output;
	
		if (abs(ma.MOTOR_OCR_VALUE) > ICR_TOP){
			if(ma.MOTOR_OCR_VALUE > 0)
				ma.MOTOR_OCR_VALUE = ICR_TOP;
			else 
				ma.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m1.SetOcrValue(ma.MOTOR_OCR_VALUE);
	
}
void setTuningsM2(void)
{
	mb.input = e2.getspeed();
		mb.error = SETPOINT2 - mb.input;

		mb.Iterm += ki * mb.error;
	
		mb.output = kp * mb.error + mb.Iterm - kd * (mb.input - mb.previnput) ;
	
		mb.previnput = mb.input;
		mb.MOTOR_OCR_VALUE = mb.MOTOR_OCR_VALUE + mb.output;
	
		if (abs(mb.MOTOR_OCR_VALUE) > ICR_TOP){
			if(mb.MOTOR_OCR_VALUE > 0)
				mb.MOTOR_OCR_VALUE = ICR_TOP;
			else
				mb.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m2.SetOcrValue(mb.MOTOR_OCR_VALUE);

}

void setTuningsM3(void)
{
	mc.input = e3.getspeed();

		mc.error = SETPOINT3 - mc.input;
		mc.Iterm += ki * mc.error;
	
		mc.output = kp * mc.error + mc.Iterm - kd * (mc.input - mc.previnput) ;
	
		mc.previnput = mc.input;
		mc.MOTOR_OCR_VALUE = mc.MOTOR_OCR_VALUE + mc.output;
	
		if (abs(mc.MOTOR_OCR_VALUE) > ICR_TOP){
			if(mc.MOTOR_OCR_VALUE > 0)
				mc.MOTOR_OCR_VALUE = ICR_TOP;
			else
				mc.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m3.SetOcrValue(mc.MOTOR_OCR_VALUE);
	
}
void setTuningsM4(void)
{
	
	md.input = e4.getspeed();

	
		md.error = SETPOINT4 - md.input;
		md.Iterm += ki * md.error;
	
		md.output = kp * md.error + md.Iterm - kd * (md.input - md.previnput) ;
	
		md.previnput = md.input;
		md.MOTOR_OCR_VALUE = md.MOTOR_OCR_VALUE + md.output;

		if (abs(md.MOTOR_OCR_VALUE) > ICR_TOP){
			if(md.MOTOR_OCR_VALUE > 0)
				md.MOTOR_OCR_VALUE = ICR_TOP;
			else
				md.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m4.SetOcrValue(md.MOTOR_OCR_VALUE);

}

void computePid()
{
	if(MotorPidFlag){
		setTuningsM1();
		setTuningsM2();
		setTuningsM3();
		setTuningsM4();
	}
	else{
		m1.SetOcrValue(velocity_motor[0]);
		m2.SetOcrValue(velocity_motor[1]);
		m3.SetOcrValue(velocity_motor[2]);
		m4.SetOcrValue(velocity_motor[3]);
	}
}

void stopDrive()
{
	m1.StopMotor();
	m2.StopMotor();
	m3.StopMotor();
	m4.StopMotor();
	SETPOINT1 = SETPOINT2 = SETPOINT3 = SETPOINT4 = 0;
	ma.MOTOR_OCR_VALUE = 0;
	mb.MOTOR_OCR_VALUE = 0;
	mc.MOTOR_OCR_VALUE = 0;
	md.MOTOR_OCR_VALUE = 0;
	ma.Iterm = 0;
	mb.Iterm = 0;
	mc.Iterm = 0;
	md.Iterm = 0;
	ma.previnput = 0;
	mb.previnput = 0;
	mc.previnput = 0;
	md.previnput = 0;
	UART0TransmitString("message print\r\n");
}

inline void incrkp(){
	kp += 0.05;}
inline void dcrkp(){
	kp -= 0.05;
}
inline void incrki(){
	ki += 0.005;
}
inline void dcrki(){
	ki -= 0.005;
}
inline void incrkd(){
	kd += 0.05;
}
inline void dcrkd(){
	kd -= 0.05;
}
#endif /* PID_H_ */