/*
 * TWI.cpp
 *
 * Created: 1/30/2016 9:45:20 AM
 *  Author: Prakash Chaudhary, Abhish Khanal
 */ 
#include "TWI.h"

volatile uint8_t _Timer_overflow_count = 0;
volatile bool _Flag_Timer_overflow = false;
volatile char _I2C_error_byte = 'z';
/*************************************************************************
 Initialization of the timer for I2C. Need to be called only once
*************************************************************************/
void I2C_Initialize_timer(void)
{
	I2C_TIMSK |= (1<<I2C_TOIE);
}/* I2C_Initialize_timer */

/*************************************************************************
 Start of timer for I2C. Need to be called for every while loop used
*************************************************************************/
void I2C_Start_timer(void)
{
	I2C_TCCRB |= (1<<I2C_CS0) | (1<<I2C_CS1) | (1<<I2C_CS2);
	I2C_TCNT = 0;
	_Flag_Timer_overflow = false;
}/* I2C_Start_timer */

/*************************************************************************
 Stop of timer for I2C. Need to be called for every while loop used
*************************************************************************/
void I2C_Stop_timer(char _Error_letter)
{
	I2C_TCCRB &= (0<<I2C_CS0) | (0<<I2C_CS1) | (0<<I2C_CS2);
	if (_Flag_Timer_overflow)
	{
		_I2C_error_byte = _Error_letter;
	}
}/* I2C_Stop_timer */

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void I2C_Initialize(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
  
  I2C_Initialize_timer();

}/* I2C_Initialize */

/*************************************************************************
 Assign unique address for every I2C device. Need to be called only once
*************************************************************************/
void I2C_Assign_address(unsigned char addr)
{
	TWAR = addr;
}/* I2C_Assign_address */

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char I2C_Start(unsigned char address)
{
	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	I2C_Start_timer();
	while(!(TWCR & (1<<TWINT)) && !_Flag_Timer_overflow);
	I2C_Stop_timer('S');
	// check value of TWI Status Register. Mask prescaler bits.
	if ( (TW_STATUS != TW_START) && (TW_STATUS != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	I2C_Start_timer();
	while(!(TWCR & (1<<TWINT))	|	!_Flag_Timer_overflow);
	I2C_Stop_timer('n');

	// check value of TWI Status Register. Mask prescaler bits.
	if ( (TW_STATUS != TW_MT_SLA_ACK) && (TW_STATUS != TW_MR_SLA_NACK) ) return 1;

	return 0;

}/* I2C_Start */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 
 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char I2C_Repeated_start(unsigned char address)
{
    return I2C_Start( address );

}/* I2C_Repeated_start */

/*************************************************************************	
  Starts slave I2C and proceeds for slave match 
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
uint8_t I2C_Slave_match(void)
{
	TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWINT);
	
	I2C_Start_timer();
	while(!(TWCR & (1<<TWINT)) && !_Flag_Timer_overflow);
	I2C_Stop_timer('M');
	
	if(TW_STATUS!= TW_SR_SLA_ACK) return 1;

	return 0;
}/* I2C_Slave_match */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void I2C_Stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	I2C_Start_timer();
	while((TWCR & (1<<TWSTO)) && !_Flag_Timer_overflow);
	I2C_Stop_timer('s');

}/* i2c_stop */

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char I2C_Write_byte( unsigned char data )
{
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	I2C_Start_timer();
	while(!(TWCR & (1<<TWINT)) && !_Flag_Timer_overflow);
	I2C_Stop_timer('W');

	// check value of TWI Status Register. Mask prescaler bits
	I2C_Start_timer();
	while( TW_STATUS != TW_MT_DATA_ACK && _Flag_Timer_overflow);// return 1;
	I2C_Stop_timer('W');
	return 0;

}/* I2C_Write_byte */

/*************************************************************************
  Send array of byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
uint8_t I2C_Write_byte_array(uint8_t *data, uint8_t count)
{
	while(count--)
	{
		if (I2C_Write_byte(*data++)!= 0) return count;
	}
	return 0;
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
uint8_t I2C_Read_and_send_ack(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	
	I2C_Start_timer();
	while(!(TWCR & (1<<TWINT)) && !_Flag_Timer_overflow);
	I2C_Stop_timer('R');  
	
	I2C_Start_timer();
	while(TW_STATUS!= TW_SR_DATA_ACK && !_Flag_Timer_overflow);
	I2C_Stop_timer('R'); 
	
	return TWDR;
	

}/* i2c_readAck */

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char I2C_Read_and_send_nack(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	I2C_Start_timer();
	while(!(TWCR & (1<<TWINT)) && !_Flag_Timer_overflow);
	I2C_Stop_timer('r');
	
	I2C_Start_timer();
	while(TW_STATUS!= TW_SR_DATA_NACK && !_Flag_Timer_overflow);
	I2C_Stop_timer('r');
	
    return TWDR;

}/* i2c_readNak */

/*************************************************************************
 Read array of byte from the I2C device, request more data from device 
 and end last byte with no acknowledge signal
 Return:  byte read from I2C device
*************************************************************************/
uint8_t I2C_Read_byte_array(uint8_t *buffer, uint8_t count)
{
	while(count--)
	{
		*buffer++ = I2C_Read_and_send_ack();
		//*buffer++ = (count == 0) ? I2C_Read_and_send_nack() : I2C_Read_and_send_ack();
	}
	
	return 0;
}/* I2C_Read_byte_array */

/*************************************************************************	
  Interrupt Service Routine for timer overflow for I2C timeout
*************************************************************************/
ISR(TIMER2_OVF_vect)
{
	_Timer_overflow_count = (_Timer_overflow_count == 3)	?	0	: (_Timer_overflow_count+1);
	if (_Timer_overflow_count	==	3)
	{
		_Flag_Timer_overflow = true;
	}
}/* ISR */