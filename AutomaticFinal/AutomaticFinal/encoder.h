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
		
	public:
		int count_encoder;
		
		int speed;
		encoder()
		{
			
			count_encoder= 0;
			Init_encoder_interrupt();
			Init_timer();
		}

		void Init_encoder_interrupt();
		void Init_timer();
		
		
	};


#endif /* ENCODER_H_ */