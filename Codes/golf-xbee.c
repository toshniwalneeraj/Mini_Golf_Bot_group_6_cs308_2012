/**
Authors:

Group 6:
---------

VAIBHAV GUPTA 09005019 
ARUP KUMAR PAL 09005024
NEERAJ TOSHNIWAL 09005039
SHRIKANT NAGORI 09005040

AVR Studio Version 4.17, Build 666

Date: 18th March 2012

This project is about developing a robot which can play golf in an appropriate arena.

Concepts covered: Image Processing, Wireless Communication through Zigbee and Interrupt Handling.

Note:

1. Make sure that in the configuration options following settings are done for proper operation of the code

Microcontroller: atmega2560 Frequency: 11059200 Optimization: -O0 (For more information read section: Selecting proper optimization options below figure 4.22 in the hardware manual)

2. The matlab code uses a serial port for communicating data through zigbee module. This port number must be changed accordingly(the port used for Zigbee Communication). 

*********************************************************************************/
/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/signal.h>
#include <math.h>
#include "lcd.c"
#define FCPU 11059200ul 	//defined here to make sure that program works properly

unsigned char data;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp,sharp1, distance, adc_reading;
unsigned int value, value2;
float BATT_Voltage, BATT_V;
//unsigned char data;

int scr,scl;

/* Function: INIT_PORTS()
   Initializes the ports for simple locomotion and buzzer
*/
void INIT_PORTS()
{
	DDRA=0x0F;
	PORTA=0x00;				//INITIALIZE
	DDRE=0xCF;
	PORTE=0xFF;	
	DDRL=0x18;
	PORTL=0x18;
	DDRC=0x00; 				// buzzer off
	PORTC =0x00;
	TCCR5B =0x00;
	TCCR5A = 0xA1;
	TCCR5B=0x0B;
}

/* Function: INIT_PORTS_ROTATE()
   Initializes the ports for rotation
  */
void INIT_PORTS_ROTATE()
{
	DDRA=0x0F;
	PORTA=0x00;				//INITIALIZE
	DDRE=0xCF;
	PORTE=0xFF;	
	DDRL=0x18;
	PORTL=0x18;

}

/***** Function To Initialize UART0 *****/
// desired baud rate:9600
// actual baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled

/*
   Function: uart0_init()
   This function initilizes the ports for Interrupt Handling.
*/
void uart0_init(void)
{
 	UCSR0B = 0x00; 			//disable while setting baud rate
 	UCSR0A = 0x00;
 	UCSR0C = 0x06;
 	UBRR0L = 0x47; 			//set baud rate lo
 	UBRR0H = 0x00; 			//set baud rate hi
 	UCSR0B = 0x98;
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
/*
	Function: buzzer_on()
	This function turns on the buzzer.
*/
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}
/*
    Function: buzzer_off()
	This function turns off the buzzer.
*/
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

/*	
	Function: SIGNAL(SIG_USART0_RECV)
	This functions stores the byte sent by matlab into the global variable data.
*/
 
SIGNAL(SIG_USART0_RECV)
{
	buzzer_off(); 
	data = UDR0; 			//making copy of data from UDR0 in data variable	
}

SIGNAL(SIG_USART0_TRANS)
{}

/*
    Function: timer5_init()
	This function initilizes the timer which is used for delaying the system from time to time.
*/
void timer5_init()
{
	TCCR5B = 0x00;
	TCCR5A = 0xA1;
	TCCR5B = 0x0B;
}
/*
	Function: forward()
	This function makes the bot move forward. 
*/
void forward()
{
	PORTA=0x06;
}
/*
	Function: right()
	This function makes the bot turn right. 
*/
void right() 				//function for moving right 
{
	PORTA=0x02;     				// Soft right
}
/*
	Function: left()
	This function makes the bot turn right. 
*/
void left() 				//function for moving left
{
	PORTA=0x05;     				// Hard left
}
/*
	Function: back()
	This function makes the bot move backwards. 
 */
void back() 				//function for moving backward
{
	PORTA=0x09;     				// move back
}
/*
	Function: stop()
	This function makes the bot stop any movement.
 */
void stop() 				//function for moving stop
{
	PORTA=0x00;						// Stop
}

void right_hard() 			//function for moving right
{
	PORTA=0x0A;						// Hard right
}
/*
	Function: velocity(unsigned char t1, unsigned char t2)
	
    Velocity control function of wheels.
    
    Parameters:
    
		t1 - For the left wheel
		t2 - For the right wheel
*/
void velocity (unsigned char t1,unsigned char t2)
{
	OCR5AL = t1;                  
	OCR5BL = t2;
}


//initialization function of left wheel encoder

/* 
    Function: left_position_encoder_interrupt_init()
    This function enables the left position encoder interrupt.
 */

void left_position_encoder_interrupt_init(void)
{
	cli();
	EICRB=EICRB|0x02;
	EIMSK=EIMSK|0x10;
	sei();
}

//initialization function of right wheel encoder

/* 
    Function: left_position_encoder_interrupt_init()
    This function enables the right position encoder interrupt.
 */

void right_position_encoder_interrupt_init(void)
{
	cli();
	EICRB=EICRB|0x08;
	EIMSK=EIMSK|0x20;
	sei();
}

/*Angle rotation function 
When an angle is given the function calculates the number of steps needed for that angle.
Whenever the wheel cuts the shaft encoder, the corresponding ISR is invoked (based on the 
wheel which rotates) and scr or scl is incremented.This scl or scr value is compared with the
required count.If it is less then again rotation continues or the loop will break
*/


/*
   Function: angle_rotate(unsigned int Degrees)
   
   This function rotates the bot by given amount of degrees.
   
   Parameters:
   
   degrees - Given amount of degrees to rotate
*/


void angle_rotate(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
	scr = 0; 
	scl = 0; 
	while (1)
	{
		if((scr>= ReqdShaftCountInt) | (scl >= ReqdShaftCountInt))
		{ 
					      
			break;
		}
		else
		{
						
			right();
		}
	}
   	stop(); 
}

/*Angle rotation function for hard right rotation 
Same as above but rotation function is hard right.
*/

/*
   Function:angle_rotate_right_hard(unsigned int Degrees)
   
   This function rotates the bot hard right with the given amount of degrees.
   
   Parameters:
   
   Degrees - Given amount of degrees to rotate
*/


void angle_rotate_right_hard(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
	scr = 0; 
	scl = 0; 
	while (1)
	{
		if((scr>= ReqdShaftCountInt) | (scl >= ReqdShaftCountInt))
		{ 
					      
			break;
		}
		else
		{
						
			right_hard();
		}

	}
   	stop(); 

}

/*Angle rotation function for hard left rotation 
Same as above but rotation function is hard left.
*/

/*
   Function: angle_rotate_left(unsigned int Degrees)
   
   This function rotates the bot hard left with the given amount of degrees.
   Parameters:
   
   Degrees - Given amount of degrees to rotate
*/


void angle_rotate_left(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
	scr = 0; 
	scl = 0; 
	while (1)
	{
		if((scr>= ReqdShaftCountInt) | (scl >= ReqdShaftCountInt))
		{ 
					      
			break;
		}
		else
		{
						
			left();

		}

	}

   	stop(); 

}

//ISR for left wheel shaft encoder

ISR(INT4_vect)
{
	scl++;
}
//ISR for right wheel shaft encoder
ISR(INT5_vect)
{
	scr++;

}


/*Linear distance function
When a distance is given as input to this function, number of shaft count needed for that 
distance is calculated by the function.Then as the wheel rotates, the ISR is invoked 
and the scr or scl value increments in ISR and this value is compared with the required count
and if that value is reached, the loop breaks else the function for forward movement is 
executed 
*/


/* 
    Function: linear_distance_mm(unsigned int DistanceInMM)
   
    Linear distance function
    When a distance is given as input to this function, number of shaft count needed for that 
    distance is calculated by the function.Then as the wheel rotates, the ISR is invoked 
    and the scr or scl value increments in ISR and this value is compared with the required count
    and if that value is reached, the loop breaks else the function for forward movement is 
    executed.
    
    Parameters:
    
    DistanceInMM - distance given in mm. 
*/


void linear_distance_mm(unsigned int DistanceInMM)
{ 	
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = DistanceInMM / 5.338; 	// division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	scr = 0; 	
	while(1) 	
	{
					  		
		if(scr > ReqdShaftCountInt)
		{
			break;
		}
		else
		{
			forward();
		}	

	} 

	stop(); //Stop action
}



/* code for distance calculation using IR sensor  */

/*
   Function: lcd_port_config(void) 
   This function configures the lcd output port
*/ 


void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; 		//all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; 		// all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration

/*
   Function: adc_port_config(void)
   This function configures the adc output port
*/
void adc_pin_config (void)
{
	DDRF = 0x00; 
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

/*
   Function: port_init()
   This function intializes the different ports
   
*/

void port_init()
{
	buzzer_pin_config();
	lcd_port_config();
	adc_pin_config();	
}

/*
   Function: adc_init()
   Function initializing adc
*/
	
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;				//MUX5 = 0
	ADMUX = 0x20;				//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;				//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void init_devices (void)
{
	cli(); 						//Clears the global interrupts
	port_init();
	adc_init();
	sei(); 						//Enables the global interrupts
}

/*
   Function: ADC_Conversion(unsigned char Ch)
   This function converts the 
   
*/
	
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;			// select the ch. > 7
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		//do not disturb the left adjustment
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; 		//clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}
	

/*
   Function: Sharp_GP2D12_estimation (unsigned char adc_reading)
   
   This function estimates the distance of the obstacle given an adc_reading
   
   Parameters:
   
   adc_reading - The value of adc.
   
*/


unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}



// Servo motor control codes


/*
   Function: servo1_pin_config()
   This function configures the servo motor 1 which is the motor controlling the motion of the camera.
*/

void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  		//making PORTB 5 pin output
 	PORTB = PORTB | 0x20; 		//setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
/*
   Function: servo2_pin_config()
   Configure PORTB 6 pin for servo motor 2 operation
*/

void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  		//making PORTB 6 pin output
 	PORTB = PORTB | 0x40; 		//setting PORTB 6 pin to logic 1
}



//Configure PORTB 7 pin for servo motor 3 operation
/*
   Function: servo3_pin_config()
   Configure PORTB 7 pin for servo motor 3 operation
*/

void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  		//making PORTB 7 pin output
 	PORTB = PORTB | 0x80; 		//setting PORTB 7 pin to logic 1
}

//Initialize the ports
/*
   Function: port_init_servo()
   Initialize the servo ports
*/

void port_init_servo(void)
{ 
	servo1_pin_config(); 		//Configure PORTB 5 pin for servo motor 1 operation
 	servo2_pin_config(); 		//Configure PORTB 6 pin for servo motor 2 operation 
 	servo3_pin_config(); 		//Configure PORTB 7 pin for servo motor 3 operation  
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 42.187Hz 

/*
   Function: timer1_init()
   TIMER1 initialization in 10 bit fast PWM mode  
*/
void timer1_init(void)
{
 	TCCR1B = 0x00; 				//stop
 	TCNT1H = 0xFC; 				//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				//Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				//Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				//Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				//Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				///Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				//Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to initialize all the peripherals
/*
   Function: init_devices_servo()
   Function to initialize all the peripherals
*/

void init_devices_servo(void)
{
 	cli(); 						//disable all interrupts
 	port_init_servo();
 	timer1_init();
 	sei(); 						//re-enable interrupts 
}


//Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees


/*
    Function: servo_1(unsigned char degrees)
    Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees
    
    Parameters:
    
    degrees - Amount of degrees to rotate servo motor 1
*/
void servo_1(unsigned char degrees)  
{
 	float PositionPanServo = 0;
 	PositionPanServo = ((float)degrees / 2.25) + 21.0;
 	OCR1AH = 0x00;
 	OCR1AL = (unsigned char) PositionPanServo;
}



/*
    Function: servo_2(unsigned char degrees)
    Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees
    
    Parameters:
    
    degrees - Amount of degrees to rotate servo motor 2
*/

void servo_2(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1BH = 0x00;
 	OCR1BL = (unsigned char) PositionTiltServo;
}

/*
   Function: right90_at_place()
   Function to rotate the bot by 90 degrees to its right at its place
*/

void right90_at_place()
{
	// init_devices();
	velocity (220, 220); //Set robot velocity here. Smaller the value lesser will be the velocity
						 //Try different valuse between 0 to 255
	// firebird 18
	int count = 0;
	while(count < 16)
	{
		right();
		_delay_ms(100);
		count++;		
	}
	stop();
	_delay_ms(5);
}

/*
   Function: left90_at_place()
   Function to rotate the bot by 90 degrees to its left at its place
*/

void left90_at_place()
{
	// init_devices();
	velocity (220, 220); //Set robot velocity here. Smaller the value lesser will be the velocity
						 //Try different valuse between 0 to 255
	// firebird 18
	int count = 0;
	while(count < 10)
	{
		left();
		_delay_ms(100);
		count++;		
	}
	stop();
	_delay_ms(5);
}

/*
   Function: rotate_in_circle()
   This function rotates the bot in circle
*/


int rotate_in_circle()
{
		velocity(200,130);
		
		int count = 0;
		data = 1;
		while(count < 64 && data == 1)
		{			
			//right();
			//_delay_ms(50);
			forward();
			_delay_ms(300);	
			stop();
			_delay_ms(1000);
			count++;
		}
		velocity(0,0);
		stop();
		_delay_ms(500);
		return data;
}



/*
     Function: servo_3(unsigned char degrees)
     Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees
     
     Parameters:
     
     degrees - Amount of degrees
*/

void servo_3(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1CH = 0x00;
 	OCR1CL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

/*
   Function: servo_1_free(void)
   servo_free functions unlocks the servo motors from the any angle 
*/

void servo_1_free (void) 	//makes servo 1 free rotating
{
 	OCR1AH = 0x03; 
 	OCR1AL = 0xFF; 			//Servo 1 off
}

/*
   Function: servo_2_free(void)
   servo_free functions unlocks the servo motors from the any angle 
*/

void servo_2_free (void) 	//makes servo 2 free rotating
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; 			//Servo 2 off
}


/*
   Function: servo_3_free(void)
   servo_free functions unlocks the servo motors from the any angle 
*/

void servo_3_free (void) 	//makes servo 3 free rotating
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; 			//Servo 3 off
} 

int get_angle(int h)
{
	if(data > atoi(5))return 80;
	else return 60;
}


/*
   Function: main()
   Performs the movement according to the data sent by matlab code.

*/

void main()
{
	unsigned int value,value1;
	int a=0,b=0;
	cli();
	INIT_PORTS();										//Initialize ports
	uart0_init();										//Initialize UART0 for xbee communication
	timer5_init();
	sei();

	INIT_PORTS_ROTATE();								//Initialize ports 
	right_position_encoder_interrupt_init();			//Initialize control registers for wheel
	left_position_encoder_interrupt_init();				//           encoders.
			
	init_devices();
	lcd_set_4bit();										//LCD initialization functions.
	lcd_init();

	unsigned char angle = 0;
 	init_devices_servo();								//Initialize servo motors.

	data='0';
	sharp = ADC_Conversion(11);							//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
//	lcd_print(1,1,value,3);

//	sharp1 = ADC_Conversion(10);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
//	value1 = Sharp_GP2D12_estimation(sharp1);			//Stores Distance calsulated in a variable "value".
//	lcd_print(1,5,value1,3);


// set servo for camera to ground zero
			servo_1(0);								// code to bring the camera to ground state
			_delay_ms(1000);
// set servo for incline to ground zero	
			servo_2(90);							// align camera	
			_delay_ms(500);

	
// the bot rotating slowly trying to find the bot 
	int terminate = 0;
			while(data=='0' && terminate < 150)
			{
				velocity(150,150); 							//If no ball is detected the rotate and scan
				angle_rotate_left(3);							// for ball in the arena.
				_delay_ms(500);
				stop();
				_delay_ms(500);
				sharp = ADC_Conversion(11);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value = Sharp_GP2D12_estimation(sharp);		//Stores Distance calsulated in a variable "value".
				terminate++;
			//	lcd_print(1,1,value,3);
			}	

			if(terminate == 60)
			{
					buzzer_on();
					return;
			}

/*If a ball(red colour) is detected then matlab code sends a '5' signal through
 zigbee.If a '5' is received then the robot stops rotating and moves towards the 
 ball*/			

			value2 = 100;
			if(atoi(data) > 5) 
			{
				lcd_print(1,5,value2,2);
			}

			velocity(160,184);

// change value
			while(value>140)
			{
				sharp = ADC_Conversion(11);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value = Sharp_GP2D12_estimation(sharp);		//Stores Distance calsulated in a variable "value".
				lcd_print(1,1,value,3);
				forward();
			}
			stop();
			_delay_ms(2000);
		
/* the servo_1 is for the camera and the servo_2 is for the inclined plane */

// rotate 90 at left
			left90_at_place();
			_delay_ms(1000);
// rotate the camera 90 degrees to the right
			servo_1(90);
			_delay_ms(1000);
//rotate in circle
			rotate_in_circle();
			int perceived_height;
			perceived_height = atoi(data);
			lcd_print(1,5,perceived_height,5);
// rotate 90 degrees to the right to face the ball again
			right90_at_place();
			stop();
			_delay_ms(2000);
// rotate camera also to ground state to face the ball again
			servo_1(0);								// code to bring the camera to ground state
			_delay_ms(1000);
		
// set servo for incline

			int angle_of_inclination = get_angle(perceived_height);
			lcd_print(1,2,angle_of_inclination,5);
  			servo_2(angle_of_inclination);
			_delay_ms(1000);

				
	while(1);
}

