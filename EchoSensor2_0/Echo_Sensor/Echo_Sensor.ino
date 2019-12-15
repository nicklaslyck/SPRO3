#include "usart.h"

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#define CONTROL_MASK (~((0b00000000 | (1 << PINC4)) | (1 << PINC5)))

static volatile int demoxControl = 0;
static volatile long int pulse[4] = {0, 0, 0, 0};
float distance[4] = {0, 0, 0, 0};

static void interrupts_setup(void);
static void ob_detect(void);


int main(void)
{
    uart_init();
    io_redirect();
    interrupts_setup();
    while(1)
    {
        ob_detect();
    }
}

static void ob_detect(void)
{
    distance[demoxControl] = pulse[demoxControl] * 0.1715 / 2;
    printf("%i = %2.f", demoxControl, distance[demoxControl]);
}

static void interrupts_setup(void)
{
  DDRD &= ~(1 << DDD2); //echo pin as input (INT0 pin)
  DDRD |= (1 << DDD4); //trigger pin as output
	DDRC |= (1 << DDC4); //control pin 1 as output
	DDRC |= (1 << DDC5); //control pin 2 as output
	DDRD &= ~(1 << DDD2); //echo pin as input (INT0 pin)

	PORTD &= ~(1 << PORTD4); //trigger pin to 0
	PORTC &= ~(1 << PORTC4); //control pin 1 to 0
	PORTC &= ~(1 << PORTC5); //control pin 2 to 0

    TCCR1B |= (1 << WGM12); //CTC mode
	TCCR1B |= (1 << CS11);  //set prescaler to 8 and start timer
	
	EICRA |= (1 << ISC00); //Set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0); //Turns on interrupt for INT0
	sei();
}

ISR (INT0_vect)
{
    if ((PIND & (1 << PIND2) == 1))
    {
        demoxControl++;
        if (demoxControl > 3)
        {
            demoxControl = 0;
        }
        pulse[demoxControl] = TCNT1;
    }
    else
    {
        PORTC = ((PORTC & CONTROL_MASK) | (demoxControl << PINC4));
        _delay_us(10);
        PORTD |= (1 << PIND4);
        _delay_us(10);
        PORTD &= ~(1 << PIND4);
        TCNT1 = 0;
        _delay_ms(1);
    } 
}
