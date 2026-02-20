#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// ---- Pin mapping ----
// HC-SR04 TRIG -> P3.5
// HC-SR04 ECHO -> P3.7  (through voltage divider to ~3.3V)
// LED1 -> P1.0

#define TRIG_DIR   P3DIR
#define TRIG_OUT   P3OUT
#define TRIG_PIN   BIT5

#define ECHO_DIR   P3DIR
#define ECHO_IN    P3IN
#define ECHO_REN   P3REN
#define ECHO_OUT   P3OUT
#define ECHO_IE    P3IE
#define ECHO_IES   P3IES
#define ECHO_IFG   P3IFG
#define ECHO_PIN   BIT7

#define LED_DIR    P1DIR
#define LED_OUT    P1OUT
#define LED_PIN    BIT0

// ---- Clocks ----
#define ACLK_HZ    32768UL     // REFO default ACLK (~32.768 kHz)

// ---- Behavior tuning ----
#define MEAS_PERIOD_MS        80u    // how often a distance measurement is taken
#define DETECT_MAX_CM         60u    // max distance to blink
#define DETECT_MIN_CM         2u     // min distance to blink
#define BLINK_MIN_MS          60u    // fastest blink when close
#define BLINK_MAX_MS          900u   // slowest blink when far

static volatile uint16_t echo_start = 0;
static volatile uint16_t echo_width_us = 0;
static volatile bool echo_got_pulse = false;
static volatile bool echo_wait_falling = false;
static volatile bool blink_enabled = false;
static volatile bool measure_due = false;

// Simple clamp
static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void timers_init(void) {
    // TA1: free-running timebase @ SMCLK (~1MHz), continuous mode
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

    // TA0: blink scheduler using ACLK
    TA0CCTL0 = 0;
    TA0CCR0  = (uint16_t)(ACLK_HZ / 2);
    TA0CTL   = TASSEL__ACLK | MC__STOP | TACLR;

    // TA2: measurement tick using ACLK
    TA2CCTL0 = CCIE;
    TA2CCR0  = (uint16_t)((uint32_t)ACLK_HZ * MEAS_PERIOD_MS / 1000u);
    if (TA2CCR0 < 1) TA2CCR0 = 1;
    TA2CTL   = TASSEL__ACLK | MC__UP | TACLR;
}

static void clocks_init_1mhz_smclk(void) {
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = (CSCTL2 & ~((uint16_t)(SELS_7 | SELM_7))) | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

static void gpio_init(void) {
    // LED1
    LED_DIR |= LED_PIN;
    LED_OUT &= ~LED_PIN;

    // TRIG output low
    TRIG_DIR |= TRIG_PIN;
    TRIG_OUT &= ~TRIG_PIN;

    // ECHO input
    ECHO_DIR &= ~ECHO_PIN;

    // Pulldown so the input never floats
    ECHO_REN |= ECHO_PIN;
    ECHO_OUT &= ~ECHO_PIN;

    // Port interrupt for ECHO, start on rising edge
    ECHO_IES &= ~ECHO_PIN;
    ECHO_IFG &= ~ECHO_PIN;
    ECHO_IE  |= ECHO_PIN;
}

// Busy-wait delay in microseconds using TA1
static void delay_us(uint16_t us) {
    uint16_t start = TA1R;
    while ((uint16_t)(TA1R - start) < us) {
        __no_operation();
    }
}

// HC-SR04 approx distance_cm = pulse_us / 58
static uint16_t pulse_us_to_cm(uint16_t pulse_us) {
    return (uint16_t)(pulse_us / 58u);
}

// ---- Blink control ----

static void blink_stop_hard(void) {
    // After this returns: timer is stopped AND LED is off
    uint16_t gie = __get_SR_register() & GIE;
    __disable_interrupt();

    blink_enabled = false;

    TA0CCTL0 &= ~CCIE;
    TA0CCTL0 &= ~CCIFG;
    TA0CTL = (TA0CTL & ~MC_3) | MC__STOP;
    TA0CTL |= TACLR;

    LED_OUT &= ~LED_PIN;

    if (gie) __enable_interrupt();
}

static void blink_start_or_update_ms(uint16_t period_ms) {
    uint32_t half_ms = (uint32_t)period_ms / 2u;
    uint32_t ccr0 = (uint32_t)ACLK_HZ * half_ms / 1000u;
    if (ccr0 < 1) ccr0 = 1;
    if (ccr0 > 0xFFFF) ccr0 = 0xFFFF;

    uint16_t gie = __get_SR_register() & GIE;
    __disable_interrupt();

    if (!blink_enabled) {
        // Transition OFF -> BLINK: make state deterministic
        blink_enabled = true;
        LED_OUT &= ~LED_PIN;

        TA0CTL   = (TA0CTL & ~MC_3) | MC__STOP;
        TA0CTL  |= TACLR;
        TA0CCR0  = (uint16_t)ccr0;
        TA0CCTL0 &= ~CCIFG;
        TA0CCTL0 |= CCIE;
        TA0CTL   = (TA0CTL & ~MC_3) | MC__UP;
    } else {
        // Already blinking: update rate WITHOUT resetting TAR
        TA0CCTL0 &= ~CCIE;
        TA0CCR0   = (uint16_t)ccr0;
        TA0CCTL0 &= ~CCIFG;
        TA0CCTL0 |= CCIE;
    }

    if (gie) __enable_interrupt();
}

// Trigger a measurement and wait up to ~35ms for an echo pulse width
static bool measure_echo_pulse(void) {
    uint16_t gie = __get_SR_register() & GIE;
    __disable_interrupt();

    echo_got_pulse = false;
    echo_wait_falling = false;

    // Ensure interrupt is set to rising edge before triggering
    ECHO_IE  &= ~ECHO_PIN;
    ECHO_IFG &= ~ECHO_PIN;
    ECHO_IES &= ~ECHO_PIN;   // rising
    ECHO_IE  |= ECHO_PIN;

    if (gie) __enable_interrupt();

    // Send TRIG: low -> high 10us -> low
    TRIG_OUT &= ~TRIG_PIN;
    delay_us(2);
    TRIG_OUT |= TRIG_PIN;
    delay_us(10);
    TRIG_OUT &= ~TRIG_PIN;

    // Wait up to ~35ms
    uint16_t t0 = TA1R;
    while (!echo_got_pulse) {
        if ((uint16_t)(TA1R - t0) > 35000) {
            // Put edge select back to rising for next time
            ECHO_IE &= ~ECHO_PIN;
            ECHO_IES &= ~ECHO_PIN;
            echo_wait_falling = false;
            return false;
        }
        __no_operation();
    }
    return true;
}

static void update_blink_from_measurement(uint16_t cm, bool valid) {
    // If not valid or out of range: LED forced OFF, timer stopped
    // If valid and in range: timer running and LED blinking

    if (!valid || cm < DETECT_MIN_CM || cm > DETECT_MAX_CM) {
        blink_stop_hard();
        return;
    }

    // Closer distance => faster pulse
    uint16_t period_ms = (uint16_t)(cm * 15u);
    period_ms = clamp_u16(period_ms, BLINK_MIN_MS, BLINK_MAX_MS);

    blink_start_or_update_ms(period_ms);
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    clocks_init_1mhz_smclk();
    gpio_init();
    timers_init();

    blink_stop_hard();

    __enable_interrupt();

    while (1) {
        // Sleep until the measurement timer says wake
        while (!measure_due) {
            __bis_SR_register(LPM0_bits | GIE);
            __no_operation();
        }
        measure_due = false;

        bool ok = measure_echo_pulse();
        uint16_t cm = ok ? pulse_us_to_cm(echo_width_us) : 0;
        update_blink_from_measurement(cm, ok);
    }
}

// ---- Interrupts ----

// ECHO edge timing (Port 3)
__attribute__((interrupt(PORT3_VECTOR)))
void port3_isr(void) {
    switch (__even_in_range(P3IV, P3IV_P3IFG7)) {
    case P3IV_NONE:
        break;

    case P3IV_P3IFG7:
        if (!echo_wait_falling) {
            // Rising edge
            echo_start = TA1R;
            echo_wait_falling = true;
            P3IES |= ECHO_PIN; // falling next
        } else {
            // Falling edge
            uint16_t end = TA1R;
            echo_width_us = (uint16_t)(end - echo_start);
            echo_got_pulse = true;

            echo_wait_falling = false;
            P3IES &= ~ECHO_PIN; // rising next
        }
        break;

    default:
        break;
    }
}

// Blink ISR (TimerA0 CCR0)
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void timer0_a0_isr(void) {
    if (blink_enabled) {
        LED_OUT ^= LED_PIN;
    }
}

// Measurement scheduler ISR (TimerA2 CCR0)
// Fires every MEAS_PERIOD_MS using ACLK
// Sets measure_due flag, wakes CPU from LPM0 so main() can trigger new ultrasonic measurement
__attribute__((interrupt(TIMER2_A0_VECTOR)))
void timer2_a0_isr(void) {
    measure_due = true;
    __bic_SR_register_on_exit(LPM0_bits); // wake main
}