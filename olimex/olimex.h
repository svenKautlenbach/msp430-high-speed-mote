#pragma once

#define LED_On()	P1OUT |= 0x01;		P1DIR |= 0x01;
#define LED_Off()	P1OUT &= (~0x01); 	P1DIR |= 0x01;

void olimex_init();
void olimex_delay(unsigned long delay);
