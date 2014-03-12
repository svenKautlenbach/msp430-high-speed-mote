#include "olimex.h"

#include <cc430x613x.h>

#define LED1_PIN	BIT0
#define BUTTON1_PIN BIT1

static void initLeds()
{
	// Led P1_0.
	P1REN &= ~LED1_PIN;
	P1DIR |= LED1_PIN;
}

static void initPushButtons()
{
	// Button P1_1.

	P1DIR &= ~BIT1;
	//P1REN |= BIT1;
	//P1OUT |= BIT1;
	P1IES &= BIT1;
	P1IFG = 0;
	//P1IE  |= BIT1;
}

static void resetIO()
{
	// Inputs with pulldown to conserve power.
	P1OUT = 0x00;
	P1DIR = 0x00;
	P1REN = 0x00;
	P2OUT = 0x00;
	P2DIR = 0x00;
	P2REN = 0x00;
	P3OUT = 0x00;
	P3DIR = 0x00;
	P3REN = 0x00;
	// Not modifying JTAG here.
	P5OUT = 0x00;
	P5DIR = 0x00;
	P5REN = 0x00;
}

void olimex_init()
{
	//WDTCTL = WDTPW + WDTHOLD;

	//resetIO();
	//initLeds();
	initPushButtons();
}

void olimex_delay(unsigned long delay)
{
	while (delay--)
	{
		//unsigned long a = 100000;
		//while (a--)
			asm("nop");
	}
}
