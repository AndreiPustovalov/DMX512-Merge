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

INLINE void set_uart_mode() {
	P1SEL |= BIT3;
}

INLINE void pin_down() {
	P1SEL &= ~BIT3;
}

INLINE void green_led_on() {
	P2OUT |= BIT6;
}

INLINE void green_led_off() {
	P2OUT &= ~BIT6;
}

INLINE void yellow_led_on() {
	P2OUT |= BIT5;
}

INLINE void yellow_led_off() {
	P2OUT &= ~BIT5;
}

INLINE void yellow_led_toggle() {
	P2OUT ^= BIT5;
}

INLINE void relay_on() {
	P1OUT |= BIT6;
}

INLINE void relay_off() {
	P1OUT &= ~BIT6;
}

INLINE int get_green_state() {
	return P2OUT | BIT6;
}

#endif /* PERIPHERY_H_ */
