/*
 * TWI.h
 *
 * Created: 1/30/2016 9:43:30 AM
 *  Author: Prakash Chaudhary, Abhish Khanal
 */ 

/*************************************************************************
 Error Byte that can be used to find which step's function is causing the
 problem in I2C communication between Master Transmitter and SR
 
 Check can be done by viewing the value of _I2C_error_byte
 
 S--> Start Condition Error
 M--> Slave Match Error
 s--> Stop Condition Error
 W--> Write Byte Error
 R--> Read and Send Acknowledgement Error
 r--> Read and Send No Acknowledgement Error
 
*************************************************************************/

#ifndef TWI_H_
#define TWI_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

/* Macros for timer */
#define I2C_TCCRB TCCR2B
#define I2C_CS0 CS20
#define I2C_CS1 CS21
#define I2C_CS2 CS22

#define I2C_TIMSK TIMSK2
#define I2C_TOIE TOIE2

#define I2C_TCNT TCNT2


#define SCL_CLOCK  100000L
#define I2C_WRITE   0
#define I2C_READ    1

void I2C_Initialize_timer(void);
void I2C_Start_timer(void);
void I2C_Stop_timer(char _Error_letter);

void I2C_Initialize(void);
void I2C_Assign_address(unsigned char addr);

uint8_t I2C_Start(unsigned char addr);
uint8_t I2C_Repeated_start(unsigned char addr);
uint8_t I2C_Slave_match(void);
void I2C_Stop(void);

uint8_t I2C_Write_byte(unsigned char data);
uint8_t I2C_Write_byte_array(uint8_t *data, uint8_t count);
uint8_t I2C_Read_and_send_ack(void);
uint8_t I2C_Read_and_send_nack(void);
uint8_t I2C_Read_byte_array(uint8_t *buffer, uint8_t count);

#endif /* TWI_H_ */