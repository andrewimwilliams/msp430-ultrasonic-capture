#include <msp430.h>

static void delay(void) {
    volatile unsigned long i;
    for (i = 0; i < 60000; i++) { __no_operation(); }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;      // stop watchdog

    PM5CTL0 &= ~LOCKLPM5;          // <<< IMPORTANT on FRAM devices

    P1DIR |= BIT0;                 // LED1 (P1.0)
    P1OUT &= ~BIT0;

    while (1) {
        P1OUT ^= BIT0;
        delay();
    }
}
