/**
 * @file coprocessor.c
 * @brief Bare‑metal ATmega328P peripheral supervisor for ESP32 Flight Controller.
 *
 * Responsibilities:
 *  - Receive current FSM state via UART @ 9600 baud.
 *  - Monitor Li‑ion 3S battery voltage using ADC0.
 *  - Drive WS2812B RGB LED strip (8 LEDs) for visual feedback.
 *  - Control buzzer for warning tones.
 *  - Execute quick low‑battery and failsafe alerts.
 *
 * Executes in a cooperative super‑loop; a 1 ms timer interrupt maintains a
 * global millisecond tick used for LED flashing schedules.
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/*────────────────────────── Pin & Electrical Definitions ───────────────────*/
#define LED_PIN  PD6           ///< WS2812B data pin
#define BUZZ_PIN PD5           ///< Piezo buzzer
#define BATT_CH  0             ///< ADC channel (A0)
#define R1 10000.0f            ///< Divider high leg (Ω)
#define R2 4700.0f             ///< Divider low leg (Ω)
#define VREF 5.0f
#define NUM_LEDS 8
#define LOW_BATT 10.5f         ///< 3S battery threshold (~3.5 V/cell)

/*────────────────────────── State Context Structure ────────────────────────*/
typedef struct {
    uint8_t state;         ///< FSM code from ESP32
    float   vbat;          ///< Battery voltage
    uint8_t low;           ///< Low‑battery flag
} Context;

static volatile uint32_t ms=0;    ///< Millisecond tick counter
ISR(TIMER0_COMPA_vect){ ms++; }   ///< 1 kHz system tick

/*────────────────────────── Hardware Abstraction Layer ─────────────────────*/

/** Configures LED and buzzer pins as outputs (low‑level default). */
static void HAL_GPIO_Init(void){
    DDRD |= (1<<BUZZ_PIN)|(1<<LED_PIN);
    PORTD &= ~((1<<BUZZ_PIN)|(1<<LED_PIN));
}

/** Initializes UART0 @ 9600 baud, 8N1, receive‑only. */
static void HAL_UART_Init(void){
    uint16_t ubrr=103; UBRR0H=ubrr>>8; UBRR0L=ubrr;
    UCSR0B=(1<<RXEN0); UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);
}

/**
 * @brief Non‑blocking UART read.
 * @param b Pointer for received byte.
 * @return 1 = new byte available.
 */
static uint8_t HAL_UART_Read(uint8_t *b){
    if(UCSR0A&(1<<RXC0)){ *b=UDR0; return 1; }
    return 0;
}

/** Initializes ADC in single‑ended mode using AVCC reference. */
static void HAL_ADC_Init(void){
    ADMUX=(1<<REFS0)|BATT_CH;
    ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

/** Performs blocking ADC read (~100 µs). */
static uint16_t HAL_ADC_Read(void){
    ADCSRA|=(1<<ADSC); while(ADCSRA&(1<<ADSC)); return ADC;
}

/** Configures Timer0 CTC to trigger 1 ms ISR. */
static void HAL_Timer_Init(void){
    TCCR0A=(1<<WGM01); TCCR0B=(1<<CS01)|(1<<CS00);
    OCR0A=249; TIMSK0=(1<<OCIE0A);
}

/*────────────────────────── WS2812 Driver (800 kHz) ────────────────────────*/
static void WS_SendByte(uint8_t b){
    for(uint8_t i=0;i<8;i++){
        if(b&0x80){ PORTD|=(1<<LED_PIN); __builtin_avr_delay_cycles(12);
                    PORTD&=~(1<<LED_PIN); __builtin_avr_delay_cycles(4);}
        else      { PORTD|=(1<<LED_PIN); __builtin_avr_delay_cycles(4);
                    PORTD&=~(1<<LED_PIN); __builtin_avr_delay_cycles(12);}
        b<<=1;
    }
}

/** Sends uniform RGB color to entire LED strip. */
static void WS_SetColor(uint8_t r,uint8_t g,uint8_t b){
    cli();
    for(uint8_t i=0;i<NUM_LEDS;i++){ WS_SendByte(g); WS_SendByte(r); WS_SendByte(b); }
    sei();
    _delay_us(50); // Latch code
}

/*────────────────────────── Application Tasks ─────────────────────────────*/

/**
 * @brief Samples telemetry and updates the global context.
 */
static void Task_Update(Context *c){
    uint8_t in;
    if(HAL_UART_Read(&in)) c->state=in;
    uint16_t raw=HAL_ADC_Read();
    float vpin=(raw/1023.0f)*VREF;
    c->vbat = vpin/(R2/(R1+R2));
    c->low  = (c->vbat < LOW_BATT);
}

/**
 * @brief Drives LED/buzzer outputs based on current state and battery.
 */
static void Task_Output(Context *c){
    if(c->low){               /* Critical battery → flash red + beep */
        if((ms%400)<200){ PORTD|=(1<<BUZZ_PIN); WS_SetColor(255,0,0); }
        else             { PORTD&=~(1<<BUZZ_PIN); WS_SetColor(0,0,0); }
        return;
    }

    PORTD&=~(1<<BUZZ_PIN);    /* Ensure buzzer off */

    switch(c->state){
    case 0:  WS_SetColor(0,0,255); break;                    /* BOOT – solid blue */
    case 1:  if((ms%500)<250) WS_SetColor(200,200,0);
             else WS_SetColor(0,0,0); break;                 /* CALIB – blink yellow */
    case 2:  WS_SetColor(0,255,0); break;                    /* DISARM – green */
    case 3:  if((ms%200)<100) WS_SetColor(255,0,0);
             else WS_SetColor(0,0,0); break;                 /* ARMED – red blink */
    case 4:  if((ms%100)<50)  WS_SetColor(150,0,255);
             else WS_SetColor(0,0,0); break;                 /* FAILSAFE – purple flash */
    default: WS_SetColor(0,0,0); break;
    }
}

/*────────────────────────── Main Program ───────────────────────────────*/
int main(void){
    HAL_GPIO_Init(); HAL_UART_Init(); HAL_ADC_Init(); HAL_Timer_Init(); sei();

    Context ctx={0};
    for(;;){
        Task_Update(&ctx);
        Task_Output(&ctx);
    }
}
