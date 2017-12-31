/*
 * encoder.h
 *
 * Created: 10/18/2017 12:47:22 PM
 *  Author: abheesh
 */ 



/********************TO DO / TO SEE***********************
ENCODER X AND ENCODER Y CAN ONLY CALCULATE UPTO 29 m distance due to count_encoder being 'int' data type'
*/////////////////////////////////////////////////////////


#ifndef ENCODER_H_
#define ENCODER_H_


#define encoderdiameter 58
#define encoderPPR		 200


#include <avr/io.h>
#include <avr/interrupt.h>


class encoder
{
	private:
		int count_encoder;
		int speed;
		
		
	public:
		float distance;
		encoder()
		{
			
			count_encoder= 0;
			Init_encoder_interrupt();
			Init_timer();
		}

		void Init_encoder_interrupt();
		void Init_timer();
		inline void inc_count(){count_encoder++;};
		inline void dcr_count(){count_encoder--;};
		inline void resetCount(){count_encoder = 0;};
		void setspeed(){
			speed = count_encoder;
			count_encoder = 0;
			};
		float getdistance();
		float getinstdistance();
		int getcount(){return count_encoder;};
		inline int getspeed(){return speed;};
		
	};


#endif /* ENCODER_H_ */