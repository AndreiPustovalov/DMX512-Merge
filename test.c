//******************************************************************************
//
//
//                MSP430F6721
//             -----------------
//        /|\ |                 |
//         |  |                 |
//         ---|RST              |
//            |                 |
//            |     P1.3/UCA0TXD|------------>
//            |                 | 9600 - 8N1
//            |     P1.2/UCA0RXD|<------------
//
//******************************************************************************
#include <msp430.h>
#include <stdio.h>
#include "driverlib.h"

#define SEND_INTERVAL 10000
#define INLINE inline

void setFreqDL();
INLINE void set_uart_mode();
INLINE void pin_down();
INLINE void green_led_on();
INLINE void green_led_off();
INLINE int get_green_state();
INLINE void yellow_led_on();
INLINE void yellow_led_off();
INLINE void relay_on();
INLINE void relay_off();
INLINE void yellow_led_toggle();

unsigned char tx = 0;
unsigned char c_cur = 0x1, w_cur = 0xfe;
unsigned char tx_state = 4;
unsigned char yl_state = BIT5;

int main(void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    yl_state ^= BIT5;
    setFreqDL();

    P1SEL = BIT2 | BIT3 | BIT4;     // Set P1.2, P1.3, P1.4 to non-IO
    P1DIR = BIT2 | BIT3 | BIT4 | BIT6;     // Enable UCA0RXD, UCA0TXD, UCA1RXD
    P1OUT = 0;

    // Setup P2.2 UCA2RXD, P2.3 UCA2TXD
    P2SEL = BIT2 | BIT3;                       // Set P2.2, P2.3 to non-IO
    P2DIR = BIT2 | BIT3 | BIT4 | BIT5 | BIT6;  // Enable UCA2RXD, UCA2TXD

    P2OUT = yl_state;

    // Setup eUSCI_A0
    UCA0CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA0CTLW0 |= UCSSEL__SMCLK | UCSPB;
    UCA0BRW_L = 0x40;                       // 250k
    UCA0BRW_H = 0x00;                       //
    UCA0MCTLW = 0x00;                       //
    UCA0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA0IE |= UCRXIE | UCTXIE;              // Enable USCI_A0 RX TX interrupt

    // Setup TA1
    TA1CCTL0 = CCIE;                        // CCR0 interrupt enabled
    TA1CCR0 = SEND_INTERVAL;
    TA1EX0 = TAIDEX_7;
    TA1CTL = TASSEL_2 | MC_2 | TACLR | ID__8;       // SMCLK, contmode, clear TAR

    tx_state = 0;
    __bis_SR_register(GIE);     // interrupts enabled
	static signed char c_dir = 1, w_dir = 1;
    for (;;) {
        __bis_SR_register(LPM3_bits);     // Enter LPM3, interrupts enabled
		c_cur += c_dir;
		w_cur += w_dir;
		if (c_cur == 255)
			c_dir = -1;
		if (c_cur == 0)
			c_dir = 1;
		if (w_cur == 255)
			w_dir = -1;
		if (w_cur == 0)
			w_dir = 1;
    }
}

// USCI_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(UCA0IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
            break;
        case USCI_UART_UCTXIFG:
        	if (tx_state == 3) {
				++tx;
				switch (tx) {
				case 1:
					UCA0TXBUF = c_cur;
					break;
				case 2:
					UCA0TXBUF = w_cur;
					break;
				case 17:
					tx_state = 0;
					green_led_off();
					TA1CCR0 = SEND_INTERVAL;
					TA1CTL |= MC_2 | TACLR; //start timer
					break;
				default:
					UCA0TXBUF = 0;
					break;
				}
        	}
        	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

// Timer1_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	switch (tx_state) {
	case 0:
		pin_down();
		tx_state = 1;
		green_led_on();
		TA1CCR0 += 30;
		break;
	case 1:
		set_uart_mode();
		tx_state = 2;
		TA1CCR0 += 3;
		break;
	case 2:
		tx = 0;
		tx_state = 3;
		UCA0TXBUF = 0;
		TA1CTL &= ~(BIT5 | BIT6); //stop timer
        __bic_SR_register_on_exit(LPM3_bits);
		break;
	case 3:
		yellow_led_on();
		break;
	default:
		relay_on();
		break;
	}
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=UNMI_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(UNMI_VECTOR)))
#endif
void NMI_ISR(void)
{
	relay_on();
	static uint16_t status = STATUS_SUCCESS;
    do
    {
        // If it still can't clear the oscillator fault flags after the timeout,
        // trap and wait here.
        status = UCS_clearAllOscFlagsWithTimeout(1000);
    }
    while(status != 0);
}

void setFreqDL() {
    //Set VCore = 2 for 16MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_2);

    //Set DCO FLL reference = REFO
    UCS_initClockSignal(
        UCS_FLLREF,
        UCS_REFOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
        );
    //Set SMCLK = DCO
    UCS_initClockSignal(
    	UCS_SMCLK,
		UCS_DCOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
        );
    //Set ACLK = REFO
    UCS_initClockSignal(
        UCS_ACLK,
        UCS_REFOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
        );

    //Set Ratio and Desired MCLK Frequency  and initialize DCO
    UCS_initFLLSettle(
        16000,
        488
        );

    // Enable global oscillator fault flag
    SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
//    SFR_enableInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
}

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
