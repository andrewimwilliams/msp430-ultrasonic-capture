#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// ---- LCD (1602 + I2C backpack) ----
// HD44780 16x2 module with a PCF8574 I2C expander
#define LCD_I2C_ADDR  0x27u

// PCF8574 -> LCD bit mapping
#define LCD_PCF_RS        0x01u
#define LCD_PCF_RW        0x02u
#define LCD_PCF_EN        0x04u
#define LCD_PCF_BL        0x08u

// MSP-EXP430FR5994 LaunchPad I2C pins
// UCB2SDA -> P7.0, UCB2SCL -> P7.1
#define LCD_SDA_DIR   P7DIR
#define LCD_SDA_SEL0  P7SEL0
#define LCD_SDA_SEL1  P7SEL1
#define LCD_SDA_PIN   BIT0

#define LCD_SCL_DIR   P7DIR
#define LCD_SCL_SEL0  P7SEL0
#define LCD_SCL_SEL1  P7SEL1
#define LCD_SCL_PIN   BIT1

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

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void clocks_init_1mhz_smclk(void) {
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_0; // ~1MHz
    CSCTL2 = (CSCTL2 & ~((uint16_t)(SELS_7 | SELM_7))) | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
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

// ---- I2C + LCD low-level ----

static void i2c_init_100khz(void) {
    // Put pins into I2C peripheral mode (UCB2 on P7.0/P7.1)
    LCD_SDA_DIR &= ~LCD_SDA_PIN;
    LCD_SCL_DIR &= ~LCD_SCL_PIN;

    LCD_SDA_SEL0 |= LCD_SDA_PIN;
    LCD_SDA_SEL1 &= ~LCD_SDA_PIN;
    LCD_SCL_SEL0 |= LCD_SCL_PIN;
    LCD_SCL_SEL1 &= ~LCD_SCL_PIN;

    // eUSCI_B2 I2C master, SMCLK @ ~1MHz, 100kHz
    UCB2CTLW0 = UCSWRST;
    UCB2CTLW0 |= UCSSEL__SMCLK | UCMST | UCMODE_3 | UCSYNC;
    UCB2BRW = 10; // ~1MHz / 10 = 100kHz
    UCB2I2CSA = LCD_I2C_ADDR;
    UCB2CTLW0 &= ~UCSWRST;
}

static bool i2c_write_byte(uint8_t byte) {
    // Blocking single-byte write to LCD_I2C_ADDR
    // Returns false if a NACK is observed
    while (UCB2CTLW0 & UCTXSTP) { /* wait */ }

    UCB2CTLW0 |= UCTR | UCTXSTT;          // TX mode, START

    // Wait for START to be sent (or NACK)
    while (UCB2CTLW0 & UCTXSTT) {
        if (UCB2IFG & UCNACKIFG) {
            UCB2IFG &= ~UCNACKIFG;
            UCB2CTLW0 |= UCTXSTP;
            return false;
        }
    }

    // Wait for TX buffer ready
    while (!(UCB2IFG & UCTXIFG0)) {
        if (UCB2IFG & UCNACKIFG) {
            UCB2IFG &= ~UCNACKIFG;
            UCB2CTLW0 |= UCTXSTP;
            return false;
        }
    }
    UCB2TXBUF = byte;

    // Wait for byte to shift out
    while (!(UCB2IFG & UCTXIFG0)) {
        if (UCB2IFG & UCNACKIFG) {
            UCB2IFG &= ~UCNACKIFG;
            UCB2CTLW0 |= UCTXSTP;
            return false;
        }
    }

    UCB2CTLW0 |= UCTXSTP;
    while (UCB2CTLW0 & UCTXSTP) { /* wait */ }
    return true;
}

static uint8_t lcd_backlight = LCD_PCF_BL;

static void lcd_pcf_write(uint8_t v) {
    (void)i2c_write_byte((uint8_t)(v | lcd_backlight));
}

static void lcd_pulse_en(uint8_t v) {
    lcd_pcf_write((uint8_t)(v | LCD_PCF_EN));
    delay_us(1);
    lcd_pcf_write((uint8_t)(v & ~LCD_PCF_EN));
    delay_us(50);
}

static void lcd_write4(uint8_t nibble, uint8_t control) {
    uint8_t v = (uint8_t)((nibble & 0xF0u) | (control & (LCD_PCF_RS | LCD_PCF_RW)));
    lcd_pulse_en(v);
}

static void lcd_send(uint8_t byte, bool rs) {
    uint8_t control = rs ? LCD_PCF_RS : 0u;
    lcd_write4((uint8_t)(byte & 0xF0u), control);
    lcd_write4((uint8_t)((byte << 4) & 0xF0u), control);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_send(cmd, false);
    if (cmd == 0x01u || cmd == 0x02u) {
        delay_us(2000);
    }
}

static void lcd_data(uint8_t data) {
    lcd_send(data, true);
}

static void lcd_clear(void) {
    lcd_cmd(0x01u);
}

static void lcd_goto(uint8_t row, uint8_t col) {
    // 16x2 addressing: row0=0x00, row1=0x40
    uint8_t addr = (row == 0) ? 0x00u : 0x40u;
    addr = (uint8_t)(addr + col);
    lcd_cmd((uint8_t)(0x80u | addr));
}

static void lcd_print_padded_16(const char *s) {
    // Print up to 16 chars, pad remainder with spaces to fully overwrite the line
    uint8_t i = 0;
    while (i < 16 && s[i] != '\0') {
        lcd_data((uint8_t)s[i]);
        i++;
    }
    while (i < 16) {
        lcd_data((uint8_t)' ');
        i++;
    }
}

static void lcd_init(void) {
    i2c_init_100khz();

    // HD44780 init sequence (4-bit) over PCF8574
    // Wait >40ms after power-up
    delay_us(50000);

    // Reset sequence: 0x03 (nibble) x3, then 0x02
    lcd_write4(0x30u, 0u);
    delay_us(4500);
    lcd_write4(0x30u, 0u);
    delay_us(4500);
    lcd_write4(0x30u, 0u);
    delay_us(150);
    lcd_write4(0x20u, 0u); // 4-bit
    delay_us(150);

    lcd_cmd(0x28u); // function set: 4-bit, 2-line, 5x8
    lcd_cmd(0x08u); // display OFF
    lcd_clear();
    lcd_cmd(0x06u); // entry mode: increment, no shift
    lcd_cmd(0x0Cu); // display ON, cursor OFF
}

// ---- Ultrasonic + blink logic ----

// HC-SR04 approx distance_cm = pulse_us / 58
static uint16_t pulse_us_to_cm(uint16_t pulse_us) {
    return (uint16_t)(pulse_us / 58u);
}

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

// ---- LCD application layer ----

typedef enum {
    LCD_STATE_NO_OBJECT = 0,
    LCD_STATE_DETECTED,
    LCD_STATE_TOO_CLOSE,
} lcd_state_t;

static void u16_to_dec(char *out, uint16_t v) {
    char tmp[5];
    uint8_t n = 0;
    if (v == 0) {
        out[0] = '0';
        out[1] = '\0';
        return;
    }
    while (v > 0 && n < 5) {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    for (uint8_t i = 0; i < n; i++) {
        out[i] = tmp[n - 1u - i];
    }
    out[n] = '\0';
}

static void lcd_render(lcd_state_t st, uint16_t cm) {
    char num[6];
    char line1[17];
    char line2[17];

    for (uint8_t i = 0; i < 16; i++) {
        line1[i] = ' ';
        line2[i] = ' ';
    }
    line1[16] = '\0';
    line2[16] = '\0';

    switch (st) {
    case LCD_STATE_NO_OBJECT:
        {
            const char *a = "No object";
            const char *b = "detected!";
            for (uint8_t i = 0; a[i] && i < 16; i++) line1[i] = a[i];
            for (uint8_t i = 0; b[i] && i < 16; i++) line2[i] = b[i];
        }
        break;

    case LCD_STATE_TOO_CLOSE:
        {
            const char *a = "Object too";
            const char *b = "close!";
            for (uint8_t i = 0; a[i] && i < 16; i++) line1[i] = a[i];
            for (uint8_t i = 0; b[i] && i < 16; i++) line2[i] = b[i];
        }
        break;

    case LCD_STATE_DETECTED:
    default:
        {
            const char *a = "Object:";
            const char *b = "Blinking";
            for (uint8_t i = 0; a[i] && i < 16; i++) line1[i] = a[i];

            u16_to_dec(num, cm);
            uint8_t pos = 7; // after "Object:"
            if (pos < 16) line1[pos++] = ' ';
            for (uint8_t i = 0; num[i] && pos < 16; i++) line1[pos++] = num[i];
            if (pos < 16) line1[pos++] = ' ';
            if (pos < 16) line1[pos++] = 'c';
            if (pos < 16) line1[pos++] = 'm';

            for (uint8_t i = 0; b[i] && i < 16; i++) line2[i] = b[i];
        }
        break;
    }

    lcd_goto(0, 0);
    lcd_print_padded_16(line1);
    lcd_goto(1, 0);
    lcd_print_padded_16(line2);
}

static void lcd_update_from_measurement(uint16_t cm, bool ok) {
    static lcd_state_t last_state = (lcd_state_t)0xFF;
    static uint16_t last_cm = 0xFFFFu;

    lcd_state_t st;
    if (!ok) {
        st = LCD_STATE_NO_OBJECT;
    } else if (cm < DETECT_MIN_CM) {
        st = LCD_STATE_TOO_CLOSE;
    } else if (cm > DETECT_MAX_CM) {
        st = LCD_STATE_NO_OBJECT;
    } else {
        st = LCD_STATE_DETECTED;
    }

    if (st != last_state || (st == LCD_STATE_DETECTED && cm != last_cm)) {
        lcd_render(st, cm);
        last_state = st;
        last_cm = cm;
    }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    clocks_init_1mhz_smclk();
    gpio_init();
    timers_init();

    lcd_init();
    lcd_clear();
    lcd_render(LCD_STATE_NO_OBJECT, 0);

    blink_stop_hard();

    __enable_interrupt();

    for (;;) {
        // Sleep until the measurement timer says wake
        while (!measure_due) {
            __bis_SR_register(LPM0_bits | GIE);
            __no_operation();
        }
        measure_due = false;

        bool ok = measure_echo_pulse();
        uint16_t cm = ok ? pulse_us_to_cm(echo_width_us) : 0;

        update_blink_from_measurement(cm, ok);
        lcd_update_from_measurement(cm, ok);
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