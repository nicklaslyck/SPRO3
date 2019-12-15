/* 
/ This file contains all the functions necessary to control the
/ arduino section of the SPRO3.
/ Functions might me changed into separate files with their 
/ corresponding libraries later on the development.
*/

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#define F_CPU 16000000UL
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

//motor control variables
#define M2F OCR2B // PD3
#define M2B OCR0B // PD5
#define M1F OCR0A // PD6
#define M1B OCR2A // PB3
#define FAST 126
#define SLOW 127

typedef enum{
  DOWN = 0, 
  UP
}way;
//#define UP true
//#define DOWN false
#define MAXHEIGHT 1700 //@TODO set max height
#define MINHEIGHT 2120 //@TODO set min height
static uint8_t motor_mode = FAST;

//battery voltage variables
#define VOLTAGEREFERENCE 4800
#define RESOLUTION 1024.0
#define MINVOLTAGE 3600
#define SMALLVOLTAGE 3650

//echo sensor variabels
#define BAUDRATE 9600
#define SPEEDOFSOUND 0.0343
#define CONTROL_MASK 0xCF
#define FILTERMAX 3
#define MINDISTANCE 20.0
#define ECHOSENSORS 4
#define MAXCOUNTER 1000
	//message send
#define OBALARM 1
#define UNDERVOLTAGE 2
#define LOWVOLTAGE 3
#define BOXLOADED 4200 //@TODO define value for box loaded

	//meddage recieve
#define MESSAGEOK 246
#define SIGN 1
#define NOSIGN 2
#define STORAGE 253
#define GOTOSLOW 252
#define GOTOFAST 251
#define LIFTUP 600 //@TODO set the proper value
#define STOP 0
#define TAKE 4
#define DROP 3

typedef enum
{
  FALSE = 0,
  TRUE = 1
}dir;

//functions
int main(void);

static volatile dir metAlarm = FALSE;
static volatile dir cleared = TRUE;

static void motor_stop(void);
static void motor_control(void);
static void lift_control(way direction);
static float handle_height(void);

static uint16_t adc_read(uint8_t adcChannel);
static void handleBattery(void);

unsigned char rasPi_recieve(void);
void rasPi_send(unsigned char);

static void pin_setup(void);
static void interrupts_setup(void);
static void pwm_setup(void);
static void adc_setup(void);
static void rasPi_setup(void);
static void arduino_init(void);

static void met_actions(void);

int main(void)
{
	arduino_init();
	motor_stop();
    lift_control(DOWN);
	_delay_ms(1500);
	while (1) 
	{	
          handleBattery();
          motor_control();
        }
}

static void motor_stop(void)
{
	M1F = 0;
	M1B = 0;
	M2F = 0;
	M2B = 0;
}

static void motor_control(void)
{
	static unsigned char instruction = 0;
	instruction = rasPi_recieve(); //@TODO verify how often the Pi is sending serial
	float speedM1 = 0;
	float speedM2 = 0;
	if (instruction == STOP)
	{
		motor_stop();
	}
	//Instructions for motor 1
	if (instruction > 0 && instruction <= 70)
	{
		M1F = 1 * motor_mode;
		M1B = 0;
	}
	else if (instruction > 70 && instruction <= 80)
	{
		M1F = (0.6 * motor_mode);
		M1B = 0;
	}
	else if (instruction > 80 && instruction <= 90)
	{
		M1F = (0.5 * motor_mode);
		M1B = 0;
	}
	else if (instruction > 90 && instruction <= 100)
	{
		M1F = (0.4 * motor_mode);
		M1B = 0;
	}
	else if (instruction > 100 && instruction <= 110)
	{
		M1F = (0.3 * motor_mode);
		M1B = 0;
	}
	else if (instruction > 110 && instruction <= 120)
	{
		M1F = (0.2 * motor_mode);
		M1B = 0;
	}
	else if (instruction > 120 && instruction <= 127)
	{
		M1F = 0;
		M1B = 1 * motor_mode;
	}

	//Instructions for motor 2
	if (instruction < 127 && instruction >= 58)
	{
		M2F = 1 * motor_mode;
		M2B = 0;
	}
	if (instruction < 58 && instruction >= 48)
	{
		M2F = (0.6 * motor_mode);
		M2B = 0;
	}
	if (instruction < 48 && instruction >= 38)
	{
		M2F = (0.5 * motor_mode);
		M2B = 0;
	}
	if (instruction < 38 && instruction >= 28)
	{
		M2F = (0.4 * motor_mode);
		M2B = 0;
	}
	if (instruction < 28 && instruction >= 18)
	{
		M2F = (0.3 * motor_mode);
		M2B = 0;
	}
	if (instruction < 18 && instruction >= 8)
	{
		M2F = (0.2 * motor_mode);
		M2B = 0;
	}
	if (instruction < 8 && instruction >= 1)
	{
		M2F = 0;
		M2B = 1 * motor_mode;
	}
	

	//Instructions for arduino
	if (instruction == LIFTUP && motor_mode == SLOW) //@TODO 128 on SLOW operation means storage
	{
		rasPi_send(MESSAGEOK);
		lift_control(UP);
		rasPi_send(BOXLOADED);
	}
	else if (instruction == GOTOFAST && motor_mode != FAST)
	{
		motor_mode = FAST;
	}
	else if (instruction == GOTOSLOW && motor_mode != SLOW)
	{
		motor_mode = SLOW;
	}

	//@TODO set ecuations and movement pattern
}

static void lift_control(way direction)
{
	motor_stop();
	if (direction == UP)
	{
		while (handle_height() > MAXHEIGHT)//@TODO set interrupt for box detection
		{
			PORTB |= (1 << PORTB4);
			PORTB &= ~(1 << PORTB5);
			handleBattery();
		}
	}
	else if (direction == DOWN)
	{
		while (handle_height() < MINHEIGHT)//@TODO set interrupt for box detection
		{
			PORTB &= ~(1 << PORTB4);
			PORTB |= (1 << PORTB5);
			handleBattery();
		}
	}
	PORTB &= ~(1 << PORTB4);
	PORTB &= ~(1 << PORTB5);
}

static float handle_height(void)
{
	float voltage = 0;
	float height = 0;
	voltage = adc_read(0) * (VOLTAGEREFERENCE / RESOLUTION);
	height = 1; //@TODO get equation of voltage to height

	return voltage;
}

static uint16_t adc_read(uint8_t adcChannel)
{
	ADMUX &= 0xf0; //clear any previously used channel, but keep internal reference
	ADMUX |= adcChannel; //set the desired channel
	ADCSRA |= (1 << ADSC); //start a conversion
	while ((ADCSRA & (1 << ADSC))); //wait for the conversion to complete

	return ADC; //now we have the result, so we return it to the calling function as a 16 bit unsigned int
}

static void handleBattery(void)
{
	static bool rasPiState = false;
	static float voltage = 0;
	voltage = adc_read(1) * (VOLTAGEREFERENCE / RESOLUTION);
	if (voltage < MINVOLTAGE)
	{
		motor_stop();
    
                PORTC &= ~(1 << PORTC2);
                _delay_ms(10);
                PORTC |= (1 << PORTC2);
                rasPi_send(UNDERVOLTAGE);
		while (rasPiState == false)
		{
			if (rasPi_recieve() == MESSAGEOK)
			{
				rasPiState = true;
			}
		}
		while (1) { }
	}
	else if (voltage < SMALLVOLTAGE)
	{
          
		motor_stop();
    
                PORTC &= ~(1 << PORTC2);
                _delay_ms(10);
                PORTC |= (1 << PORTC2);
                rasPi_send(LOWVOLTAGE);
		while (rasPiState == false)
		{
			if (rasPi_recieve() == MESSAGEOK)
			{
				rasPiState = true;
			}
		}
	}
	else
	{
		//left empty on purpose to account for all empty cases
	}
}

static void met_actions(void)
{
  static bool rasPiState = false;
  static unsigned char message = 0;
  if (metAlarm == TRUE && cleared == TRUE)
  //&& cleared)
  
  {
    motor_stop();
    PORTC &= ~(1 << PORTC2);
    _delay_ms(10);
    PORTC |= (1 << PORTC2);
    rasPi_send(OBALARM);
    while (rasPiState == false)
    {
      message = rasPi_recieve();
      if (message == SIGN) //Actions to do when sign is present
      {
        rasPiState = true;
        //motor_mode = SLOW;
        //obIgnore = true;
      }
      else if (message == NOSIGN) //Actions to do when no sign is present
      {
        while (rasPiState == false)
        {
          message = rasPi_recieve();
          if (message == TAKE)
          {
            lift_control(UP);
            rasPiState = true;
          }
          else if (message == DROP)
          {
            lift_control(DOWN);
            rasPiState = true;
          }
          else
          {
            //left empty on purpose to account for all empty cases
          }
        }
      }
      else
      {
        //left empty on purpose to account for all empty cases
      }
      //handleBattery();
      //ob_check();
    }
    
    //lift_control(UP);
    //cleared = FALSE;
    metAlarm = FALSE;
  }
}

unsigned char rasPi_recieve(void) 
{
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void rasPi_send(unsigned char data) 
{
	while (!(UCSR0A & (1 << UDRE0))); //wait for transmit buffer
	UDR0 = data; //data to be sent
}

static void pin_setup(void)
{
	//-------------------echo sensor------------------//
	DDRD |= (1 << DDD4); //trigger pin as output
	//DDRC |= (1 << DDC3); //control pin 1 as output
	DDRC |= (1 << DDC4); //control pin 2 as output
	DDRC |= (1 << DDC5); //control pin 3 as output
	DDRD &= ~(1 << DDD2); //echo pin as input (INT0 pin)

	PORTD &= ~(1 << PORTD4); //trigger pin to 0
	PORTC &= ~(1 << PORTC3); //control pin 1 to 0
	PORTC &= ~(1 << PORTC4); //control pin 2 to 0
	PORTC &= ~(1 << PORTC5); //control pin 3 to 0

	//-----------------voltage sensing----------------//
	DDRC &= ~(1 << DDC0); // Set voltage pin as input
	DDRC &= ~(1 << DDC1); // Set potMeter as input
	
	//------------------motor control-----------------//
	DDRD |= (1 << DDD3); // PD3 is now an output      
	DDRD |= (1 << DDD5); // PD5 is now an output       
	DDRD |= (1 << DDD6); // PD6 is now an output      
	DDRB |= (1 << DDB3); // PB3 is now an output
	DDRB |= (1 << DDB4); // PB4 is now an output
	DDRB |= (1 << DDB5); // PB5 is now an output

	PORTD &= ~(1 << PORTD3); //PD3 to 0
	PORTD &= ~(1 << PORTD5); //PD5 to 0
	PORTD &= ~(1 << PORTD6); //PD6 to 0
	PORTB &= ~(1 << PORTB3); //PB3 to 0
	PORTB &= ~(1 << PORTB4); //PB4 to 0
	PORTB &= ~(1 << PORTB5); //PB5 to 0
	
	//-----------------button sensing----------------//

	//----------------Pi communication---------------//
	DDRC |= (1 << DDC2); //Message ready as output
	//PORTC &= ~(1 << PORTC2); //Message ready to 0
        PORTC |= (1 << PORTC2);
        
        //-----------------Inductor Sensor---------------//
        DDRC &= ~(1 << DDC3); //echo pin as input (PCINT11 pin)
        
}

static void interrupts_setup(void)
{
	//-------------------echo sensor------------------//
	//>-- INT0 setup
	EICRA |= (1 << ISC00); //Set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0); //Turns on interrupt for INT0

	//>-- Timer setup for 1 micro second
	TCCR1B |= (1 << WGM12); //Set the Timer Mode to CTC
        TCCR1B |= (1 << CS11); //Set prescaler to 8 and start the timer
	//OCR1A = 1; //Set the value that you want to count to
	///> 1 for 1 us, 19 for 10 us, 199 for 100 us
	//TIMSK1 |= (1 << OCIE1A); //Set the ISR COMPA vect
  
        //-----------------Inductor Sensor---------------//
        PCICR |= (1 << PCIE1);
        PCMSK1 |= (1 << PCINT11);
        
	//>-- Enable interrupts  
	sei(); //Enable interrupts
	
}

static void pwm_setup(void)
{
	OCR0A = 0; // PD6
	OCR0B = 0; // PD5
	OCR2A = 0; // PB3
	OCR2B = 0; // PD3

	TCCR0A |= (1 << COM0A1); //Set non-inverting mode for PD6
	TCCR0A |= (1 << COM0B1); //Set non inverting mode for PD5
	TCCR2A |= (1 << COM2A1); //Set non-inverting mode for PB3
	TCCR2A |= (1 << COM2B1); //Set non-inverting mode for PD3

	TCCR0A |= (1 << WGM01) | (1 << WGM00); //Set fast PWM mode for PD5 and PD6
	TCCR2A |= (1 << WGM21) | (1 << WGM20); //Set fast PWM mode for PB3 and PD3

	TCCR0B |= (1 << CS01); //Set 8 prescalar and start PWM
	TCCR2B |= (1 << CS21); //Set 8 prescalar and start PWM
}

static void adc_setup(void)
{
	ADMUX = (1 << REFS0); // Select Vref = AVcc
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //set prescaler to 128 and turn on the ADC module
}

static void rasPi_setup(void)
{
	UBRR0H = (uint8_t)(BAUD_PRESCALER >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALER);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = ((1 << UCSZ00) | (1 << UCSZ01));
}

ISR(PCINT1_vect)
{
  if (!(PINC & (1 << PINC3)))
  {
    metAlarm = TRUE;
  }
  else
  {
    //cleared = TRUE;
  }
  met_actions();
}


static void arduino_init(void)
{

	pin_setup();
	interrupts_setup();
	pwm_setup();
	adc_setup();
	rasPi_setup();
}
