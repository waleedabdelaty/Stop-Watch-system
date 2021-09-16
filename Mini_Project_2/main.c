/*
 * main.c
 *
 *  Created on: Sep 16, 2021
 *      Author: Waleed Abdelaty
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include<util/delay.h>


void Timer1_CTC_Init(void)
{
	TCNT1 = 0;		/* Set timer1 initial count to zero */

	OCR1A = 977;

	TIMSK |= (1<<OCIE1A); /* Enable Timer1 Compare A Interrupt */

	/* Configure timer control register TCCR1A
	 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	 * 2. FOC1A=1 FOC1B=0
	 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
	 */

	//TCCR1A = (1<<FOC1A);

	/* Configure timer control register TCCR1B
	 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * 2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B = (1<<WGM12) | (1<<CS10)|(1<<CS12);
}



void Interrupt_Init(void)
{
		//Disable Global interrupt
		SREG&=~(1<<7);

		// ENABLE EXTERNAL INTERRUPT
		GICR|=(1<<INT1)|(1<<INT0)|(1<<INT2);

		// DEFINE LEVEL TRIGGER
		MCUCR = (1<<ISC01)|(1<<ISC10)|(1<<ISC11);

		// INT2 TRIGGER AT FALLING EDGE ISC2 -> 1
		MCUCSR &=~(1<<ISC2);

		// ENABLE I-BIT
		SREG|=(1<<7);

}


void Pin_init(void)
{
	// 4 PINS CONNECTED TO 7447 DECODER
	// CONFIGURE IT AS OUTOUT
	DDRC|=0x0f;
	// 6-PINS IN PORTA WORD AS ENABLE/DISABLE PINS
	// CONFIGURE IT AS OUTOUT
	DDRA|=0x3f;
	// PORTD PIN 2 LET IT AS INPUT
	// IT IS FOR INT0 TO WORK AS RESET
	// WE USE INTERNAL PULL UP RESISTER
	DDRD&=~(1<<2);
	PORTD|=(1<<2);
	// PORTD PIN 3 LET IT AS INPUT
	// IT IS FOR INT0 TO WORK AS PAUSED
	// WE USE PULL DOWN RESISTER
	DDRD&=~(1<<3);
	// PORTB PIN 2 LET IT AS INPUT
	// IT IS FOR INT2 TO WORK AS RESUMED
	// WE USE INTERNAL PULL UP RESISTER
	DDRB&=~(1<<2);
	PORTB|=(1<<2);
}







unsigned char sec_tick=0 , min_tick=0 , h_tick=0 ;





ISR(INT0_vect)
{
	TCNT1=0;
	sec_tick=0;
	min_tick=0;
	h_tick=0;
}

ISR(INT1_vect)
{
	// clear cs10,sc11,sc12 TO STOP CLOCK
	TCCR1B&=~0x07;

}

ISR(INT2_vect)
{
	TCCR1B|=(1<<CS10)|(1<<CS12);

}



ISR(TIMER1_COMPA_vect)
{
	sec_tick++;
	if(sec_tick==60)
	{
		sec_tick=0;
		min_tick++;
		if(min_tick==60)
		{
			min_tick=0;
			h_tick++;
			if(h_tick==24)
			{
				h_tick=0;
			}
		}
	}
}










int main(void)
{
	Timer1_CTC_Init();
	Interrupt_Init();
	Pin_init();

	while(1)
	{

		int i=0;
				for(i=0;i<6;i++)
				{
					// TO ENABLE 7-SEGMENT
					PORTA =((1<<i) & 0x3f)|(PORTA & 0X00);
					if(i==0)
					{
						PORTC = (PORTC&0XF0)|((sec_tick%10)&0X0F);
					}
					else if(i==1)
					{
						PORTC = (PORTC&0XF0)|((sec_tick/10)&0X0F);
					}
					else if(i==2)
					{
						PORTC = (PORTC&0XF0)|((min_tick%10)&0X0F);
					}
					else if(i==3)
					{
						PORTC = (PORTC&0XF0)|((min_tick/10)&0X0F);
					}
					else if(i==4)
					{
						PORTC = (PORTC&0XF0)|((h_tick%10)&0X0F);
					}
					else if(i==5)
					{
						PORTC = (PORTC&0XF0)|((h_tick/10)&0X0F);
					}

					// WE MAKE DELAY 4MS TO RUN FAST IN 6 7-SEGMENT SO WE WILL SEE THE 6 WORKS
					_delay_ms(4);

				}








	}


}
