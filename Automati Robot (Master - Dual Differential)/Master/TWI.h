/*
 * TWI.h
 *
 * Created: 1/30/2016 9:43:30 AM
 *  Author: dhiraj.basnet
 */ 


#ifndef TWI_H_
#define TWI_H_
#define F_CPU 16000000UL

#include <util/twi.h>


#define SCL_CLOCK  100000L
#define I2C_WRITE   0
#define I2C_READ    1

void i2c_init(void);
void i2c_stop(void);
uint8_t i2c_write_array(uint8_t *data, uint8_t count);
unsigned char i2c_start(unsigned char addr);
void i2c_start_wait(unsigned char addr);
unsigned char i2c_rep_start(unsigned char addr);
unsigned char i2c_write(unsigned char data);
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);



#endif /* TWI_H_ */