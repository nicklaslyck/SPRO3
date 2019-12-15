#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

static void interrupts_setup (void);



static void interrupts_setup (void)
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


ISR(INT0_vect)
{
    if ((PIND & (1 << PIND2) == 1)
    {
        /* HIGH to LOW */
    }
    else
    {
        /* LOW to HIGH */
    } 
}