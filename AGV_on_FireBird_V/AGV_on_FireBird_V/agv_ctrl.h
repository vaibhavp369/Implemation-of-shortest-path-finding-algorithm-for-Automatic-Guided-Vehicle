/*
Author : VAIBHAV
Contact: vaibhavp627@gmail.com
			https://github.com/vaibhavp369
*/
#ifndef AGV_CTRL
#define AGV_CTRL

void motors_pin_config(void); // configure the motor pins

void pwm_pin_config(void); // Configure PWM(enable) pins of motor

void position_encoder_pin_config(void); // Configure encoder pins 

void timer5_init(void); // configure timer5 for velocity control

void velocity(unsigned char left_motor, unsigned char right_motor); // 

void position_encoder_interrupt_config(void); // Function to configure interrupt for encoders

void forward(void); // forward motion 

void back(void); // backward motion

void left(void); // left motion

void right(void); // right motion

void stop(void); //stop motor

void angle_rotate(unsigned int Degrees); // Roatate F.B. by in 0 to 360 Degrees

void linear_distance_mm(unsigned int DistanceInMM); // Linear Motion in desired MM

void forward_mm( unsigned int DistanceInMM); // Go Forward in MM

void back_mm(unsigned int DistanceInMM); // Backward in MM

void left_degrees(unsigned int Degrees); // Rotate left by Degrees

void right_degrees(unsigned int Degrees); // Rotate Right by Degrees

void buzzer_init();

void buzzer_on();

void buzzer_off();

void adc_init();

unsigned char ADC_Conversion(unsigned char channel_num);

void adc_port_config (void);

#endif