#include <msp430.h>

volatile unsigned char blinking = 0;

static void delay_ms(unsigned int ms)
{
    while (ms--)
    {
        __delay_cycles(1000);  // assumes ~1 MHz
    }
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog
    PM5CTL0 &= ~LOCKLPM5;       // Unlock GPIO

    // LED: P1.0
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // Button S1: P5.6
    P5DIR &= ~BIT6;             // input
    P5REN |= BIT6;              // enable resistor
    P5OUT |= BIT6;              // pull-up
    P5IES |= BIT6;              // falling edge (press)
    P5IFG &= ~BIT6;             // clear flag
    P5IE  |= BIT6;              // enable interrupt

    __enable_interrupt();

    while (1)
    {
        if (blinking)
        {
            P1OUT ^= BIT0;
            delay_ms(200);
        }
        else
        {
            P1OUT &= ~BIT0;     // keep LED off
            __no_operation();   // optional: place breakpoint / keep loop non-empty
        }
    }
}

// GCC-style ISR for Port 5
void __attribute__((interrupt(PORT5_VECTOR))) Port_5_ISR(void)
{
    if (P5IFG & BIT6)
    {
        blinking ^= 1;          // toggle state

        // crude debounce (fine for now)
        delay_ms(20);

        P5IFG &= ~BIT6;         // clear flag
    }
}
