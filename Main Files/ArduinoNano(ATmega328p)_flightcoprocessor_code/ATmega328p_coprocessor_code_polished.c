/**
 * @file coprocessor.c
 * @brief Bare‑metal ATmega328P peripheral supervisor for ESP32 Flight Controller.
 *
 * Responsibilities:
 *  - Receive current FSM state via UART @ 9600 baud.
 *  - Monitor Li‑ion 3S battery voltage using ADC0.
 *  - Control buzzer for warning tones.
 *  - Execute quick low‑battery and failsafe alerts.
 *
 * Executes in a cooperative super‑loop; a 1 ms timer interrupt maintains a
 * global millisecond tick used for buzzer scheduling.
 *
 * Fixes applied:
 *  - [BUG FIX] Atomic read of volatile uint32_t ms via get_ms() to prevent
 *    corrupted reads caused by Timer0 ISR firing mid‑read on the 8‑bit AVR.
 *  - [REMOVED] WS2812B LED strip feature and all related driver code.
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/*────────────────────────── Pin & Electrical Definitions ───────────────────*/
#define BUZZ_PIN PD5           ///< Piezo buzzer
#define BATT_CH  0             ///< ADC channel (A0)
#define R1 10000.0f            ///< Divider high leg (Ω)
#define R2 4700.0f             ///< Divider low leg (Ω)
#define VREF 5.0f
#define LOW_BATT 10.5f         ///< 3S battery threshold (~3.5 V/cell)

/*────────────────────────── State Context Structure ────────────────────────*/
typedef struct {
    uint8_t state;         ///< FSM code from ESP32
    float   vbat;          ///< Battery voltage
    uint8_t low;           ///< Low‑battery flag
} Context;

static volatile uint32_t ms = 0;   ///< Millisecond tick counter
ISR(TIMER0_COMPA_vect){ ms++; }    ///< 1 kHz system tick

/**
 * @brief Atomically reads the volatile 32‑bit millisecond counter.
 *
 * On an 8‑bit AVR, reading a uint32_t requires four separate load
 * instructions. Without disabling interrupts, the Timer0 ISR can fire
 * between loads and corrupt the value mid‑read.
 *
 * @return Consistent snapshot of the ms counter.
 */
static inline uint32_t get_ms(void){
    uint32_t t;
    cli();
    t = ms;
    sei();
    return t;
}

/*────────────────────────── Hardware Abstraction Layer ─────────────────────*/

/** Configures buzzer pin as output (low‑level default). */
static void HAL_GPIO_Init(void){
    DDRD  |=  (1<<BUZZ_PIN);
    PORTD &= ~(1<<BUZZ_PIN);
}

/** Initializes UART0 @ 9600 baud, 8N1, receive‑only. */
static void HAL_UART_Init(void){
    uint16_t ubrr = 103; UBRR0H = ubrr>>8; UBRR0L = ubrr;
    UCSR0B = (1<<RXEN0); UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

/**
 * @brief Non‑blocking UART read.
 * @param b Pointer for received byte.
 * @return 1 = new byte available.
 */
static uint8_t HAL_UART_Read(uint8_t *b){
    if(UCSR0A & (1<<RXC0)){ *b = UDR0; return 1; }
    return 0;
}

/** Initializes ADC in single‑ended mode using AVCC reference. */
static void HAL_ADC_Init(void){
    ADMUX  = (1<<REFS0)|BATT_CH;
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

/** Performs blocking ADC read (~100 µs). */
static uint16_t HAL_ADC_Read(void){
    ADCSRA |= (1<<ADSC); while(ADCSRA & (1<<ADSC)); return ADC;
}

/** Configures Timer0 CTC to trigger 1 ms ISR. */
static void HAL_Timer_Init(void){
    TCCR0A = (1<<WGM01); TCCR0B = (1<<CS01)|(1<<CS00);
    OCR0A  = 249; TIMSK0 = (1<<OCIE0A);
}

/*────────────────────────── Application Tasks ─────────────────────────────*/

/**
 * @brief Samples telemetry and updates the global context.
 */
static void Task_Update(Context *c){
    uint8_t in;
    if(HAL_UART_Read(&in)) c->state = in;
    uint16_t raw = HAL_ADC_Read();
    float vpin   = (raw / 1023.0f) * VREF;
    c->vbat      = vpin / (R2 / (R1 + R2));
    c->low       = (c->vbat < LOW_BATT);
}

/**
 * @brief Drives buzzer output based on current state and battery level.
 *
 * Buzzer patterns:
 *  - Low battery   : fast beep (400 ms period)
 *  - FAILSAFE (4)  : rapid beep (100 ms period)
 *  - All others    : silent
 */
static void Task_Output(Context *c){
    uint32_t now = get_ms();   // Single atomic read for this output cycle

    if(c->low){
        /* Critical battery → fast beep */
        if((now % 400) < 200) PORTD |=  (1<<BUZZ_PIN);
        else                  PORTD &= ~(1<<BUZZ_PIN);
        return;
    }

    switch(c->state){
    case 4:
        /* FAILSAFE → rapid beep */
        if((now % 100) < 50) PORTD |=  (1<<BUZZ_PIN);
        else                 PORTD &= ~(1<<BUZZ_PIN);
        break;
    default:
        /* All other states → buzzer off */
        PORTD &= ~(1<<BUZZ_PIN);
        break;
    }
}

/*────────────────────────── Main Program ───────────────────────────────────*/
int main(void){
    HAL_GPIO_Init();
    HAL_UART_Init();
    HAL_ADC_Init();
    HAL_Timer_Init();
    sei();

    Context ctx = {0};
    for(;;){
        Task_Update(&ctx);
        Task_Output(&ctx);
    }
}
