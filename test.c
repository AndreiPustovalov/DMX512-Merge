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
#include "periphery.h"

#define SEND_INTERVAL 60000
#define TX_FINISHED 1
#define START_TX 2
#define BYTE_TXED 3

void setFreqDL();

uint8_t tx = 0, next_tx;
uint8_t c_cur = 0x1, w_cur = 0xfe;
uint8_t tx_state = 4;
uint16_t time = 0;

uint8_t msg = 0;

int main(void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    setFreqDL();

    P1SEL = BIT2 | BIT3 | BIT4;     // Set P1.2, P1.3, P1.4 to non-IO
    P1DIR = BIT2 | BIT3 | BIT4 | BIT6;     // Enable UCA0RXD, UCA0TXD, UCA1RXD
    P1OUT = 0;

    // Setup P2.2 UCA2RXD, P2.3 UCA2TXD
    P2SEL = BIT2 | BIT3;                       // Set P2.2, P2.3 to non-IO
    P2DIR = BIT2 | BIT3 | BIT4 | BIT5 | BIT6;  // Enable UCA2RXD, UCA2TXD
    P2OUT = 0;

    // Setup eUSCI_A0
    UCA0CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA0CTLW0 |= UCSSEL__SMCLK | UCSPB;
    UCA0BRW_L = 0x50;                       // 250k
    UCA0BRW_H = 0x00;                       //
    UCA0MCTLW = 0x00;                       //
    UCA0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA0IE |= UCRXIE | UCTXIE;              // Enable USCI_A0 RX TX interrupt

    // Setup TA0 (rx timer)
//    TA2CCTL0 = CCIE;                        // CCR0 interrupt enabled
//    TA2CCR0 = 60000;
//    TA2EX0 = TAIDEX_7;
//    TA2CTL = TASSEL_2 | ID__8 | MC__UP | TACLR;       // SMCLK, divider up mode, clear TAR

	tx_state = 0;
    // Setup TA1 (tx timer)
    TA1CCTL0 = CCIE;                        // CCR0 interrupt enabled
    TA1EX0 = TAIDEX_7;
    TA1CCR0 = SEND_INTERVAL;
    TA1CTL = TASSEL_2 | ID__8 | TACLR;       // SMCLK, divider //contmode, clear TAR

    msg = TX_FINISHED;
    for (;;) {
        uint8_t msg_loc = msg;
        msg = 0;
        switch (msg_loc) {
        case TX_FINISHED:
			green_led_off();
			++w_cur;
			--c_cur;
			tx_state = 0;
		    TA1CCR0 = SEND_INTERVAL;
		    TA1CTL |= MC_2 | TACLR;       // SMCLK, divider //contmode, clear TAR
        	break;
        case START_TX:
    		tx_state = 3;
    		UCA0TXBUF = 0;
    		tx = 0;
    		/* no break */
        case BYTE_TXED:
        	++tx;
			switch (tx) {
			case 1:
				next_tx = c_cur;
				break;
			case 2:
				next_tx = w_cur;
				break;
			case 17:
				msg = TX_FINISHED;
				break;
			default:
				next_tx = 0;
				break;
			}
			break;
		default:
			yellow_led_on();
        }
        if (!msg) {
            __bis_SR_register(GIE);     // interrupts enabled
        	LPM3;     // Enter LPM3, interrupts enabled
            __bic_SR_register(GIE);     // interrupts disabled
        }
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
        {
            break;
        }
        case USCI_UART_UCTXIFG:
        	if (tx_state == 3) {
        		UCA0TXBUF = next_tx;
    			++tx;
        		msg = BYTE_TXED;
        		LPM3_EXIT;
        	}
        	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

// Timer2 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A0_VECTOR))) TIMER2_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
//	++time;
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
		TA1CTL &= ~(BIT5 | BIT6); //stop timer
		msg = START_TX;
		LPM3_EXIT;
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
    PMM_setVCore(PMM_CORE_LEVEL_3);

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
        UCS_CLOCK_DIVIDER_4
        );
    //Set ACLK = REFO
    UCS_initClockSignal(
        UCS_ACLK,
        UCS_REFOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
        );

    //Set Ratio and Desired MCLK Frequency  and initialize DCO
    UCS_initFLLSettle(
        20000,
        610
        );

    // Enable global oscillator fault flag
    SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
//    SFR_enableInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
}

