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
#include "driverlib.h"

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

#define THREE_BYTES 8000

unsigned char rx1 = 0, rx2 = 0, tx = 0;
unsigned char rx1_time = 0, rx2_time, tx_time = 0;
unsigned char ch1[3];
unsigned char ch2[3];
unsigned char* cur_ch = ch1;

char uart2_tx_buf[8];
unsigned int uart2_tx_pos = 8;

int main(void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    setFreqDL();

    P1SEL = BIT2 | BIT3 | BIT4;     // Set P1.2, P1.3, P1.4 to non-IO
    P1DIR = BIT2 | BIT3 | BIT4 | BIT6;     // Enable UCA0RXD, UCA0TXD, UCA1RXD

    // Setup P2.2 UCA2RXD, P2.3 UCA2TXD
    P2SEL = BIT2 | BIT3;                       // Set P2.2, P2.3 to non-IO
    P2DIR = BIT2 | BIT3 | BIT4 | BIT5 | BIT6;  // Enable UCA2RXD, UCA2TXD

    P2OUT = 0;

    // Setup eUSCI_A0
    UCA0CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA0CTLW0 |= UCSSEL_2 | UCSPB;
    UCA0BRW_L = 0x40;                       // 250k
    UCA0BRW_H = 0x00;                       //
    UCA0MCTLW = 0x00;                       //
    UCA0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA0IE |= UCRXIE | UCTXIE;              // Enable USCI_A0 RX TX interrupt

    // Setup eUSCI_A1
    UCA1CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA1CTLW0 |= UCSSEL_2 | UCSPB;
    UCA1BRW_L = 0x40;                       // 250k
    UCA1BRW_H = 0x00;                       //
    UCA1MCTLW = 0x00;                       //
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt

    // Setup eUSCI_A2
    UCA2CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA2CTLW0 |= UCSSEL_2 | UCSPB;
    UCA2BRW_L = 0x40;                       // 250k
    UCA2BRW_H = 0x00;                       //
    UCA2MCTLW = 0x00;                       //
    UCA2CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA2IE |= UCRXIE | UCTXIE;              // Enable USCI_A2 RX TX interrupt

    // Setup TA0
    TA0CCTL0 = CCIE;                        // CCR0 interrupt enabled
    TA0CCR0 = 32768;
    TA0CTL = TASSEL_1 | MC_1 | TACLR;       // SMCLK, upmode, clear TAR

    // Setup TA1
    TA1CCTL0 = CCIE;                        // CCR0 interrupt enabled
    TA1CCR0 = THREE_BYTES;
    TA1CTL = TASSEL_2 | MC_1;       // SMCLK, upmode, clear TAR
    __bis_SR_register(LPM3_bits | GIE);     // Enter LPM3, interrupts enabled
    __no_operation();                       // For debugger
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
	static unsigned char gist = 0;
    switch (__even_in_range(UCA0IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	char x = UCA0RXBUF;
        	switch (rx1) {
        	case 0:
        		break;
        	case 3:
        		if (ch1[0] != x) {
        			++gist;
        		}
        		if (ch1 == cur_ch) {
        			ch1[0] = x;
        		}
        		break;
        	case 4:
        		if (ch1[1] != x) {
        			++gist;
        		}
        		if (ch1 == cur_ch) {
        			ch1[1] = x;
        		}
        		break;
        	case 7:
        		if (ch1[2] != x) {
        			++gist;
        		}
        		if (ch1 == cur_ch) {
        			ch1[2] = x;
        		}
        		break;
        	case 16:
        		break;
        	}
        	if (gist == 6) {
        		cur_ch = ch1;
        		gist = 0;
        	}
        	++rx1;
        	rx1_time = 0;
            break;
        }
        case USCI_UART_UCTXIFG:
        	if (!get_green_state())
        		break;
        	switch (tx) {
        	case 1:
        		tx_time = 0;
				UCA0TXBUF = cur_ch[0];
        		break;
        	case 2:
				UCA0TXBUF = cur_ch[1];
        		break;
        	case 17:
        		break;
        	default:
        		UCA0TXBUF = 0;
        		break;
        	}
        	++tx;
        	if (tx == 17) {                 // TX over?
        		green_led_off();
        	}
        	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

// USCI_A1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
	static unsigned char gist = 0;
    switch (__even_in_range(UCA1IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	char x = UCA1RXBUF;
        	switch (rx2) {
        	case 0:
        		yellow_led_off();
        		break;
        	case 3:
        		if (ch2[0] != x) {
        			++gist;
        		}
        		if (ch2 == cur_ch) {
        			ch2[0] = x;
        		}
        		break;
        	case 4:
        		if (ch2[1] != x) {
        			++gist;
        		}
        		if (ch2 == cur_ch) {
        			ch2[1] = x;
        		}
        		break;
        	case 7:
        		if (ch2[2] != x) {
        			++gist;
        		}
        		if (ch2 == cur_ch) {
        			ch2[2] = x;
        		}
        		break;
        	case 16:
        		yellow_led_on();
        		break;
        	}
        	++rx2;
        	rx2_time = 0;
        	if (gist == 6) {
        		cur_ch = ch1;
        		gist = 0;
        	}
            break;
        }
        case USCI_UART_UCTXIFG:
        	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

// USCI_A2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A2_VECTOR))) USCI_A2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(UCA2IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
            break;
        case USCI_UART_UCTXIFG:
        	if (uart2_tx_pos < 8) {
        		UCA2TXBUF = uart2_tx_buf[uart2_tx_pos++];
        	}
        	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
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
	++tx_time;
	++rx1_time;
	if (rx1_time == 3) {
		rx1_time = 0;
		rx1 = 0;
	}
	++rx2_time;
	if (rx2_time == 3) {
		rx2_time = 0;
		rx2 = 0;
	}
	switch (tx_time & 0x3f) {
	case 61:
		pin_down();
		break;
	case 62:
		set_uart_mode();
		break;
	case 63:
		tx_time = 255;
		tx = 1;
		green_led_on();
		UCA0TXBUF = 0;
//		{
//			int r = snprintf(uart2_tx_buf, 8, "%d %d\r\n", c_cur, w_cur);
//			if (r > 0 && r <= 8) {
//				uart2_tx_pos = 1;
//				UCA2TXBUF = uart2_tx_buf[0];
//			}
//		}
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
    SFR_enableInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
}

INLINE void set_uart_mode() {
	P1SEL |= BIT3;
}

INLINE void pin_down() {
	P1OUT &= ~BIT3;
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

INLINE void relay_on() {
	P1OUT |= BIT6;
}

INLINE void relay_off() {
	P1OUT &= ~BIT6;
}

INLINE int get_green_state() {
	return P2OUT | BIT6;
}
