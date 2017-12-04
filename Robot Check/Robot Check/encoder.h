/*
 * encoder.h
 *
 * Created: 10/18/2017 12:47:22 PM
 *  Author: abheesh
 */ 


#ifndef ENCODER_H_
#define ENCODER_H_

#include <avr/io.h>
#include <avr/interrupt.h>

class encoder
{
	private:
		int count_encoder;
		int speed;
	public:
		
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
		inline int getspeed(){return speed;};
		int getrpm();
		
	};


#endif /* ENCODER_H_ */