/*
 * headers.h
 *
 * Created: 9/29/2017 1:49:31 PM
 *  Author: abheesh
 */ 

//This header file includes all the MACRO functions for easy use//

#ifndef HEADERS_H_
#define HEADERS_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define INPUT2(port,pin)	DDR ## port &= ~_BV(pin)
#define OUTPUT2(port,pin)	DDR ## port |= _BV(pin)
#define CLEAR2(port,pin)	PORT ## port &= ~_BV(pin)
#define SET2(port,pin)		PORT ## port |= _BV(pin)

#define INPUT(x)	INPUT2(x)
#define OUTPUT(x)	OUTPUT2(x)
#define CLEAR(x)	CLEAR2(x)
#define SET(x)		SET2(x)


#include <avr/io.h>

#endif /* HEADERS_H_ */
