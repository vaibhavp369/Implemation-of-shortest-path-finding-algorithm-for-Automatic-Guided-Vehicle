/*
Author : VAIBHAV,SIDDHANT,SHANKAR AND PRASANNA
Contact: vaibhavp627@gmail.com 
		https://github.com/vaibhavp369
*/
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include "lcd.h"
#include "agv_ctrl.h"
#include <avr/interrupt.h>

#define BAUD 9600
#define min_thresh  50
#define FOSC 14745600
#define  IR_SENSE_THRESH 100

unsigned char  Flag;
int move_flag = 0;
unsigned char node_detected;
unsigned char obs_send_flag = 0;
char current_position_x;
char current_position_y;
char current_orientation = 'N';
char next_orientation;
char next_move;
char data;
char path[130];
char fb[3];


unsigned char i=0;

int size = sizeof(path)/sizeof(path[0]);


char *next_position_x = &path[0];
char *next_position_y = &path[1];




void USART_init(unsigned int baud)
{
	cli();
	unsigned int ubrr = FOSC/16/baud-1;
	UBRR2H = (unsigned char) (ubrr >> 8);		// set baud rate
	UBRR2L = (unsigned char) ubrr;				// set baud rate
	UCSR2B = ( (1<< TXEN2) | (1 << RXEN2) | (1 << RXCIE2) );	// enable TX,RX,RX complete interrupt
	UCSR0C = ( (1 << UCSZ21) | (1 << UCSZ20) );  // 8 bit data frame and 1 stop bit
	sei();
}


char USART_Receive(void)
{
	while ( !( UCSR2A & ( 1<< RXC2) ) );
	return (char) UDR2;
}


void USART_Transmit(unsigned char data)
{
	while( !( UCSR2A & (1<< UDRE2) ) );
	UDR2 = data;
	
}

char getNextOrientation(char* current_position_x,char* current_position_y,char** next_position_x,char** next_position_y)
{
	if( **next_position_x == '?' || **next_position_y == '?' )
	{
		*next_position_x = &path[0];
		*next_position_y = &path[1];
		return 'x';
	}
	if(*current_position_x == **next_position_x && *current_position_y <= **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'N';
	}
	else if(*current_position_x <= **next_position_x && *current_position_y == **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'E';
	}
	else if(*current_position_x == **next_position_x && *current_position_y >= **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'S';
	}
	else if(*current_position_x >= **next_position_x && *current_position_y == **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'W';
	}
}

char getNextMove(char* current_orientation,char next_orientation)
{
	if((*current_orientation == 'N' && next_orientation == 'E') || (*current_orientation == 'E' && next_orientation == 'S') ||
	(*current_orientation=='S' && next_orientation == 'W') || (*current_orientation == 'W' && next_orientation == 'N')){
		*current_orientation = next_orientation;
		return 'R';
	}
	else if((*current_orientation == 'N' && next_orientation == 'W') || (*current_orientation == 'W' && next_orientation == 'S') ||
	(*current_orientation=='S' && next_orientation == 'E') || (*current_orientation == 'E' && next_orientation == 'N')){
		*current_orientation = next_orientation;
		return 'L';
	}
	else if((*current_orientation == 'N' && next_orientation == 'N') || (*current_orientation == 'E' && next_orientation == 'E') ||
	(*current_orientation=='S' && next_orientation == 'S') || (*current_orientation == 'W' && next_orientation == 'W')){
		*current_orientation = next_orientation;
		return 'F';
	}
	
	else if((*current_orientation == 'N' && next_orientation == 'S') || (*current_orientation == 'S' && next_orientation == 'N') ||
	(*current_orientation=='W' && next_orientation == 'E') || (*current_orientation == 'E' && next_orientation == 'W'))
	{
		*current_orientation = next_orientation;
		return 'T';
	}
	
	else
	return 'S';
}

void send_fb()
{
	fb[0] = current_position_x;
	fb[1] = current_position_y;
	fb[2] = current_orientation;
	for(i=0;i<3;i++)
	{
		USART_Transmit(fb[i]);
	}
}

ISR(USART2_RX_vect)
{
	data = UDR2;
	switch(data)
	{
		
		case '$':
		current_position_x = USART_Receive();
		current_position_y = USART_Receive();
		for(i=0;i<130;i++)
		{
			path[i] = USART_Receive();
			if(path[i] == '%')
			break;
		}
		next_orientation = current_orientation;
		move_flag = 1;
		break;
		
		case '#':
		fb[0] = current_position_x;
		fb[1] = current_position_y;
		fb[2] = next_orientation;
		send_fb();
		break;
	}
	
	
}



char path_follow(unsigned char L_sen, unsigned char C_sen, unsigned char R_sen)
{
	Flag = 0;
	node_detected = 0;
	if ( (L_sen > min_thresh) && (C_sen > min_thresh) && (R_sen > min_thresh) )		// Node Detected
	{
		velocity(0,0);
		Flag = 1;
		node_detected = 1;
	}
	
	if( (L_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) && move_flag )		// Left
	{
		forward();
		velocity(80,180);
		Flag = 1;
	}

	if( (R_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) && move_flag )       // right
	{
		forward();
		velocity(180,80);
		Flag = 1;
	}
	

	if ( (C_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 )&& move_flag )		// Forward
	{
		forward();
		velocity(200,212);
		Flag =1;
	}
	
	if (node_detected && move_flag)
	{
		
		
		lcd_string(1,1,"CP:");
		lcd_wr_char(1,4,current_position_x);
		lcd_wr_char(1,5,',');
		lcd_wr_char(1,6,current_position_y);
		
		
		lcd_string(1,8,"NP:");
		lcd_wr_char(1,11,*next_position_x);
		lcd_wr_char(1,12,',');
		lcd_wr_char(1,13,*next_position_y);
		
		lcd_string(2,1,"CO:");
		lcd_wr_char(2,4,current_orientation);
		
		next_orientation = getNextOrientation(&current_position_x,&current_position_y,&next_position_x,&next_position_y);
		
		
		
		send_fb();
		
		if (next_orientation == 'x')
		{
			move_flag=0;
			return 'x';
		}
		next_move = getNextMove(&current_orientation,next_orientation);
		current_orientation = next_orientation;
		_delay_ms(250);

		switch(next_move)
		{
			
			case 'F':
			velocity(200,212);
			forward_mm(85);
			break;
			
			case 'R':
			forward_mm(65);
			right_degrees(90);
			break;
			
			
			case 'L':
			forward_mm(65);
			left_degrees(90);
			break;
			
			case 'T':
			forward_mm(65);
			left_degrees(180);
			break;
			
			case 'S':
			lcd_wr_char(1,6,'G');
		}
		
	}
	

}



int main(void)
{
	USART_init(BAUD_RATE);
	unsigned char R_sense,C_sense,L_sense,Center_IR;
	char stop_flag;
	R_sense = 0;
	C_sense = 0;
	L_sense = 0;
	lcd_port_config();
	lcd_init();
	motors_pin_config();
	pwm_pin_config();
	position_encoder_pin_config();
	position_encoder_interrupt_config();
	adc_port_config();
	adc_init();
	timer5_init();
	velocity(200,200);
	buzzer_init();
	while(1)
	{
		L_sense = ADC_Conversion(left_wl_sensor_channel);
		C_sense = ADC_Conversion(center_wl_sensor_channel);
		R_sense = ADC_Conversion(right_wl_sensor_channel);
		Center_IR = ADC_Conversion(ir_prox_3_sensor_channel);
		lcd_numeric_value(2,14,Center_IR,3);
		if(Center_IR > IR_SENSE_THRESH)
			{
				stop_flag = path_follow(L_sense, C_sense, R_sense);
				obs_send_flag = 0;	
			}
		else
			{
				stop();
				if(! (obs_send_flag ))
					{
						send_fb();
						obs_send_flag = 1;
					}
				lcd_wr_char(2,12,'O');
			}
		lcd_numeric_value(1,15,move_flag,1);
	}
}