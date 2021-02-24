/*
Author : VAIBHAV
Contact: vaibhavp627@gmail.com
			https://github.com/vaibhavp369
*/

#include "firebird_avr.h"
#include "agv_ctrl.h"

#define angle_resolution 4.090			//resolution used for angle rotation
#define distance_resolution 5.338		//resolution used for distance traversal

unsigned long int ShaftCountLeft = 0; 	//to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; 	//to keep track of right position encoder			
unsigned int Degrees;

void motors_pin_config(void) 
{
	motors_dir_ddr_reg |= (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) ;			// motor pin as output
	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) );		// stop motor initially
}


void pwm_pin_config(void)
{
	motors_pwm_ddr_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// left and right channel pin as output
	motors_pwm_port_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// turn on left and right channel
}

void position_encoder_pin_config (void)
{
 	position_encoder_ddr_reg  &= ~(1 << left_encoder_pin | 1 << right_encoder_pin);  	//Set the direction of the encoder pins as input
 	position_encoder_port_reg |= (1 << left_encoder_pin | 1 << right_encoder_pin); 		//Enable internal pull-up for encoder pins
}


void timer5_init()
{
	TCCR5B_reg = 0x00;	//Stop
	
	TCNT5H_reg = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L_reg = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	
	OCR5AH_reg = 0x00;	//Output compare register high value for Left Motor
	OCR5AL_reg = 0xFF;	//Output compare register low value for Left Motor
	
	OCR5BH_reg = 0x00;	//Output compare register high value for Right Motor
	OCR5BL_reg = 0xFF;	//Output compare register low value for Right Motor
	
	// Clear on Compare
	TCCR5A_reg |= (1 << COMA1_bit) | (1 << COMB1_bit);
	TCCR5A_reg &= ~( (1 << COMA0_bit) | (1 << COMB0_bit));

	// Configure for FAST PWM
	TCCR5A_reg |= (1 << WGM0_bit);
	TCCR5A_reg &= ~(1 << WGM1_bit);
	TCCR5B_reg |= (1 << WGM2_bit);
	TCCR5B_reg &= ~(1 << WGM3_bit);

	// Set Prescalar to 64
	TCCR5B_reg |= (1 << CS1_bit) | (1 << CS0_bit);
	TCCR5B_reg &= ~(1 << CS2_bit);
}


void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL_reg = left_motor;
	OCR5BL_reg = right_motor;
}

void position_encoder_interrupt_config (void)
{
 	// all interrupts have to be disabled before configuring interrupts
	cli();	// Disable Interrupts Globally
	
	// Turn ON INT4 and INT5 (alternative function of PE4 and PE5 i.e Left and Right Encoder Pin)
	EIMSK_reg |= (1 << interrupt_left_encoder_pin | 1 << interrupt_right_encoder_pin);

	// Falling Edge detection on INT4 and INT5 pins
	EICRB_reg |= (1 << interrupt_ISC_left_bit1 | 1 << interrupt_ISC_right_bit1);
	EICRB_reg &= ~(1 << interrupt_ISC_left_bit0 | 1 << interrupt_ISC_right_bit0);

	sei();	// Enable Interrupts Globally
}

void forward (void) //both wheels forward
{
  	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_LB_pin) );	// Make LB and RB LOW
	motors_dir_port_reg |= (1 << motors_RF_pin) | (1 << motors_LF_pin) ;		// Make LF and RF HIGH
}

void back (void) //both wheels backward
{
  	motors_dir_port_reg &=  ~( (1 << motors_RF_pin) | (1 << motors_LF_pin) );	// Make LF and RF LOW
	motors_dir_port_reg |= ((1 << motors_RB_pin) | (1 << motors_LB_pin)) ;		// Make LB and RB HIGH
}

void left (void) //Left wheel backward, Right wheel forward
{
  	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_LF_pin) );	// Make LF and RB LOW
	motors_dir_port_reg |= (1 << motors_RF_pin) | (1 << motors_LB_pin) ;		// Make LB and RF HIGH
}

void right (void) //Left wheel forward, Right wheel backward
{
  	motors_dir_port_reg &=  ~( (1 << motors_LB_pin) | (1 << motors_RF_pin) );	// Make LB and RF LOW
	motors_dir_port_reg |= (1 << motors_LF_pin) | (1 << motors_RB_pin) ;		// Make LF and RB HIGH
}

void stop (void)
{
  	motors_dir_port_reg &=  ~( (1 << motors_LF_pin) | (1 << motors_RF_pin) | (1 << motors_LB_pin) | (1 << motors_RB_pin));	// Make LF, RF, LB and RB LOW
}

void angle_rotate(unsigned int Degrees)
{
	 float ReqdShaftCount = 0;
	 unsigned long int ReqdShaftCountInt = 0;

	 ReqdShaftCount = (float) Degrees/ angle_resolution; // division by resolution to get shaft count
	 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	 ShaftCountRight = 0; 
	 ShaftCountLeft = 0; 

	 while (1)
	 {
		  if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
			break;
	 }
	 stop(); //Stop robot
}


void linear_distance_mm(unsigned int DistanceInMM)
{
	 float ReqdShaftCount = 0;
	 unsigned long int ReqdShaftCountInt = 0;

	 ReqdShaftCount = DistanceInMM / distance_resolution; // division by resolution to get shaft count
	 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
	 ShaftCountRight = 0;
	 ShaftCountLeft = 0;
	 while(1)
	 {
		  if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
			  break;
	 } 
	 stop(); //Stop robot
}

ISR(INT5_vect)
{
	ShaftCountRight++;
}


ISR(INT4_vect)
{
	ShaftCountLeft++;
}


void forward_mm(unsigned int DistanceInMM)
{
	 velocity(200,212);
	 forward();
	 linear_distance_mm(DistanceInMM);
}

/**
 * @brief      Function to move backward Firebird-V by specified distance
 * @param[in]  DistanceInMM   Distance in mm 0 to 65535
 */
void back_mm(unsigned int DistanceInMM)
{
	velocity(200,212);
	 back();
	 linear_distance_mm(DistanceInMM);
}

/**
 * @brief      Function to rotate Firebird-V left by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void left_degrees(unsigned int Degrees) 
{
	 // 88 pulses for 360 degrees rotation 4.090 degrees per count
	 velocity(200,212);
	 left(); //Turn left
	 angle_rotate(Degrees);
}

/**
 * @brief      Function to rotate Firebird-V right by specified degrees
 * @param[in]  Degrees   Rotation angle 0 to 360
 */
void right_degrees(unsigned int Degrees)
{
	 // 88 pulses for 360 degrees rotation 4.090 degrees per count
	 velocity(200,212);
	 right(); //Turn right
	 angle_rotate(Degrees);
}


void buzzer_init()
{
	buzzer_ddr_reg |= (1<<buzzer_pin);
	buzzer_port_reg &=~(1<<buzzer_pin);
}

void buzzer_on()
{
	buzzer_port_reg |=(1<<buzzer_pin);
}

void buzzer_off()
{
	buzzer_port_reg &=~(1<<buzzer_pin);
}


void adc_init(){
	
	// enable ADC and pre-scalar = 64 (ADEN = 1, ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
	// and clear ADC start conversion bit, auto trigger enable bit, interrupt flag bit and interrupt enable bit
	ADCSRA_reg	|= ( (1 << ADEN_bit) | (1 << ADPS2_bit) | (1 << ADPS1_bit) );
	ADCSRA_reg	&= ~( (1 << ADSC_bit) | (1 << ADATE_bit) | (1 << ADIF_bit) | (1 << ADIE_bit) | (1 << ADPS0_bit) );
	
	// In ADCSRB, disable Analog Comparator Multiplexer, MUX5 bit and ADC Auto Trigger Source bits
	ADCSRB_reg	&= ~( (1 << ACME_bit) | (1 << MUX5_bit) | (1 << ADTS2_bit) | (1 << ADTS1_bit) | (1 << ADTS0_bit) );
	
	// In ADMUX, set the Reference Selection bits to use the AVCC as reference, and disable the channel selection bits MUX[4:0]
	ADMUX_reg	&= ~( (1 << REFS1_bit) | (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );
	ADMUX_reg	|= (1 << REFS0_bit);
	
	// In ADMUX, enable the ADLAR bit for 8-bit ADC result
	ADMUX_reg	|= (1 << ADLAR_bit);
	
	// In ACSR, disable the Analog Comparator by writing 1 to ACD_bit
	ACSR_reg	|= ( 1 << ACD_bit );
}




unsigned char ADC_Conversion(unsigned char channel_num)
{
	unsigned char adc_8bit_data;
	
	// MUX[5:0] bits to select the ADC channel number
	if ( channel_num > 7 )
	{
		ADCSRB_reg |= ( 1 << MUX5_bit );
	}
	channel_num	= channel_num & 0x07;
	ADMUX_reg	= ( ADMUX_reg | channel_num );
	
	// set the ADSC bit in ADCSRA register
	ADCSRA_reg		|= ( 1 << ADSC_bit );
	
	//Wait for ADC conversion to complete
	while( ( ADCSRA_reg & ( 1 << ADIF_bit ) ) == 0x00 );
	
	adc_8bit_data = ADCH_reg;
	
	// clear ADIF bit by writing 1 to it
	ADCSRA_reg		|= ( 1 << ADIF_bit );
	
	// clear the MUX5 bit
	ADCSRB_reg		&= ~( 1 << MUX5_bit );
	
	// clear the MUX[4:0] bits
	ADMUX_reg		&= ~( (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );
	
	return adc_8bit_data;
}



void adc_port_config (void)
{
	adc_sensor_low_ddr_reg		= 0x00;				// set PORTF direction as input
	adc_sensor_low_port_reg		= 0x00;				// set PORTF pins floating
	adc_sensor_high_ddr_reg		= 0x00;				// set PORTK direction as input
	adc_sensor_high_port_reg	= 0x00;				// set PORTK pins floating
}

