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
#include "periphery.h"
#include "crc.h"

void setFreqDL();

#define THREE_BYTES 8000

unsigned char rx1 = 0, rx2 = 0, tx = 0;
unsigned char rx1_time = 0, rx2_time, tx_time = 0;
unsigned char c1 = 0x0, w1 = 0x0, bra1 = 0x0;
unsigned char c2 = 0x0, w2 = 0x0, bra2 = 0x0;
unsigned char c3 = 0x0, w3 = 0x0;
unsigned char active = 0;

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
    switch (__even_in_range(UCA0IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	char x = UCA0RXBUF;
        	switch (rx1) {
        	case 0:
        		if (x != 0) {
        			rx1 = 16;
        		}
        	case 1:
        	case 2:
        		break;
        	case 3:
        		if (x != c1) {
        			c1 = x;
        			active = 0;
        		}
        		break;
        	case 4:
        		if (x != w1) {
        			w1 = x;
        			active = 0;
        		}
        		break;
        	case 7:
        		if (x != bra1) {
        			if (x > bra1) {
        				relay_on();
        			} else {
        				relay_off();
        			}
        			active = 0;
        			bra1 = x;
        		}
        		break;
        	case 16:
        		break;
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
        		switch (active) {
        		case 0:
        			UCA0TXBUF = c1;
        			break;
        		case 1:
        			UCA0TXBUF = c2;
        			break;
        		case 2:
        			UCA0TXBUF = c3;
        			break;
        		}
        		break;
        	case 2:
        		switch (active) {
        		case 0:
        			UCA0TXBUF = w1;
        			break;
        		case 1:
        			UCA0TXBUF = w2;
        			break;
        		case 2:
        			UCA0TXBUF = w3;
        			break;
        		}
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
    switch (__even_in_range(UCA1IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	char x = UCA1RXBUF;
        	switch (rx2) {
        	case 0:
        		if (x != 0) {
        			rx2 = 16;
        		}
        		yellow_led_off();
        	case 1:
        	case 2:
        		break;
        	case 3:
        		if (x != c2) {
        			c2 = x;
        			active = 1;
        		}
        		break;
        	case 4:
        		if (x != w2) {
        			w2 = x;
        			active = 1;
        		}
        		break;
        	case 7:
        		if (x != bra2) {
        			if (x > bra2) {
        				relay_on();
        			} else {
        				relay_off();
        			}
        			active = 1;
        			bra2 = x;
        		}
        		break;
        	case 16:
        		yellow_led_on();
        		break;
        	}
        	++rx2;
        	rx2_time = 0;
            break;
        }
        case USCI_UART_UCTXIFG:
        	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

#define CMD_GETACTIVE 1
#define CMD_SET 2
#define STATE_IDLE 3
#define STATE_RX_SET 4
#define STATE_RX_CH1 5
#define STATE_RX_CH2 6
#define STATE_RX_CRC 7
#define STATE_TX_CMDSET 8
#define STATE_TX_CMDGETACTIVE 9
#define STATE_TX_ACTIVE 10
#define STATE_TX_CRC 11
#define STATE_TX_STATUS_OK 14
#define STATE_RX_GETACTIVE 12
#define STATE_ERROR 13
#define STATE_TX_FINISH 15

unsigned char state = STATE_IDLE;
unsigned char rx_crc = 0;
unsigned char tx_crc = 0;

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
	static unsigned char ch1, ch2;

    switch (__even_in_range(UCA2IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	unsigned char x = UCA2RXBUF;
        	rx_crc = crc8LUT[x ^ rx_crc];
        	switch (state) {
        	case STATE_IDLE:
        		switch (x) {
        		case CMD_GETACTIVE:
        			state = STATE_RX_GETACTIVE;
        			break;
        		case CMD_SET:
        			state = STATE_RX_SET;
        			break;
        		default:
        			state = STATE_ERROR;
        			break;
        		}
        		break;
			case STATE_RX_SET:
				ch1 = x;
				state = STATE_RX_CH1;
				break;
			case STATE_RX_CH1:
				ch2 = x;
				state = STATE_RX_CH2;
				break;
			case STATE_RX_CH2:
				if (rx_crc) {
					state = STATE_ERROR;
				} else {
					rs485_tx();
					c3 = ch1;
					w3 = ch2;
					active = 2;
					state = STATE_TX_STATUS_OK;
					UCA2TXBUF = CMD_SET;
					tx_crc = crc8LUT[CMD_SET ^ 0];
				}
				rx_crc = 0;
				break;
			case STATE_RX_GETACTIVE:
				if (rx_crc) {
					state = STATE_ERROR;
				} else {
					rs485_tx();
					state = STATE_TX_ACTIVE;
					rx_crc = 0;
					tx_crc = crc8LUT[CMD_GETACTIVE ^ 0];
					UCA2TXBUF = CMD_GETACTIVE;
				}
				break;
        	}
        }
		break;
        case USCI_UART_UCTXIFG:
        	switch (state) {
        	case STATE_TX_ACTIVE:
        		UCA2TXBUF = active;
        		tx_crc = crc8LUT[active ^ tx_crc];
        		state = STATE_TX_CRC;
        		break;
        	case STATE_TX_STATUS_OK:
        		UCA2TXBUF = 0;
        		tx_crc = crc8LUT[0 ^ tx_crc];
        		state = STATE_TX_CRC;
        		break;
        	case STATE_TX_CRC:
        		UCA2TXBUF = tx_crc;
        		state = STATE_TX_FINISH;
        		break;
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
	static unsigned char in_err = 200;
	if (STATE_IDLE != state) {
		--in_err;
		if (!in_err) {
			state = STATE_IDLE;
			in_err = 200;
			tx_crc = 0;
			rx_crc = 0;
    		rs485_rx();
		}
	} else {
		in_err = 200;
	}
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
