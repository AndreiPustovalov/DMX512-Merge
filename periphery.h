/*
 * perifery.h
 *
 *  Created on: 29 дек. 2016 г.
 *      Author: Andrew
 */

#ifndef PERIPHERY_H_
#define PERIPHERY_H_
#include <msp430.h>

#define INLINE inline

#define set_uart_mode() \
	P1SEL |= BIT3


#define pin_down() \
	P1SEL &= ~BIT3


#define green_led_on() \
	P2OUT |= BIT6


#define green_led_off() \
	P2OUT &= ~BIT6


#define yellow_led_on() \
	P2OUT |= BIT5


#define yellow_led_off() \
	P2OUT &= ~BIT5


#define relay_on() \
	P1OUT |= BIT6


#define relay_off() \
	P1OUT &= ~BIT6


#define get_green_state() \
	(P2OUT | BIT6)

#define rs485_rx() \
	P2OUT &= ~BIT4

#define rs485_tx() \
	P2OUT |= BIT4
#endif /* PERIPHERY_H_ */
