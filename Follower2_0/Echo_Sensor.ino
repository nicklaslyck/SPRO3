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

    TCCR1B |= (1 << WGM12); //CTC mode
	TCCR1B |= (1 << CS11);  //set prescaler to 8 and start timer
	
	PCICR |= (1 << PCIE0); //sets up group of pin change interrupts
	PCMSK0 |= (1 << PCINT0); //enabled first interrupt
	sei();
}
