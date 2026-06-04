/**
 * ============================================================================
 * @file    coprocessor.c
 * @brief   Bare-metal ATmega328P peripheral supervisor for ESP32 Flight Controller.
 * @author  Troy Franco G. Celdran, and Gemini+Claude(AI Assistants)
 * @date    2026-05-25
 * * @details 
 * This firmware acts as an independent watchdog and telemetry monitor for the 
 * primary ESP32 flight controller. It utilizes a cooperative super-loop 
 * architecture with a deterministic 1 ms hardware timer tick.
 * * Responsibilities:
 * - UART Polling: Receives the finite state machine (FSM) status from the ESP32.
 * - ADC Polling: Monitors the 3S Li-ion battery voltage via a voltage divider.
 * - Acoustic Alarms: Drives a passive piezo buzzer to alert the pilot of
 * critical conditions (low voltage, failsafe).
 * * Technical Highlights:
 * - [CONCURRENCY] Uses an atomic read wrapper `get_ms()` to prevent 8-bit
 * register tearing when reading the 32-bit timer variable.
 * - [REMOVED] WS2812B LED strip driver has been deprecated to free up cycles 
 * and guarantee low-latency acoustic response.
 * ============================================================================
 */

#define F_CPU 16000000UL // System clock frequency (16 MHz)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/*────────────────────────── Pin & Electrical Definitions ───────────────────*/

#define BUZZ_PIN PD5           ///< Digital Output: Passive piezo buzzer
#define BATT_CH  0             ///< Analog Input: ADC0 (Pin A0)

/* Voltage Divider Configuration: $V_{bat} = V_{pin} \times \frac{R_1 + R_2}{R_2}$ */
#define R1 10000.0f            ///< Divider high leg (10 kΩ)
#define R2 4700.0f             ///< Divider low leg (4.7 kΩ)
#define VREF 5.0f              ///< ADC reference voltage (AVCC)
#define LOW_BATT 10.5f         ///< 3S battery critical threshold (~3.5 V/cell)

/*────────────────────────── State Context Structure ────────────────────────*/

/** * @brief Centralized state structure passed through the super-loop.
 */
typedef struct {
    uint8_t state;         ///< FSM code parsed from ESP32 UART stream
    float   vbat;          ///< Reconstructed total battery voltage
    uint8_t low;           ///< Boolean flag indicating critical battery state
} Context;

/** * @brief Global millisecond tick counter. 
 * @note Marked volatile because it is modified asynchronously inside the ISR. 
 */
static volatile uint32_t ms = 0;   

/**
 * @brief Timer0 Compare Match A Interrupt Service Routine.
 * @details Fires exactly once per millisecond to increment the global tick.
 */
ISR(TIMER0_COMPA_vect){ ms++; }    

/**
 * @brief Atomically reads the volatile 32-bit millisecond counter.
 * * @details On an 8-bit AVR architecture, fetching a 32-bit integer (uint32_t) 
 * requires four separate byte-sized load instructions. If the Timer0 ISR fires 
 * between these loads, the resulting 32-bit integer will be corrupted (tearing). 
 * Disabling global interrupts (`cli`) ensures the 4 bytes are read as a single 
 * indivisible transaction before re-enabling them (`sei`).
 * * @return uint32_t A consistent, uncorrupted snapshot of the ms counter.
 */
static inline uint32_t get_ms(void){
    uint32_t t;
    cli(); 
    t = ms;
    sei(); 
    return t;
}

/*────────────────────────── Hardware Abstraction Layer ─────────────────────*/

/** * @brief Configures the buzzer pin as an output.
 * @details Sets the Data Direction Register (DDR) for the pin and initializes
 * the port to a logic low state to prevent accidental noise on boot.
 */
static void HAL_GPIO_Init(void){
    DDRD  |=  (1<<BUZZ_PIN);
    PORTD &= ~(1<<BUZZ_PIN);
}

/** * @brief Initializes UART0 for receive-only communication at 9600 baud (8N1).
 * @details The UBRR (USART Baud Rate Register) value is calculated using the formula:
 * $UBRR = \frac{f_{CPU}}{16 \times Baud} - 1$
 * For 16 MHz and 9600 Baud: (16,000,000 / (16 * 9600)) - 1 = 103.167 (Truncated to 103).
 */
static void HAL_UART_Init(void){
    uint16_t ubrr = 103; 
    UBRR0H = ubrr>>8; 
    UBRR0L = ubrr;
    UCSR0B = (1<<RXEN0);                 // Enable receiver module
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);    // Configure frame: 8 data bits, 1 stop bit
}

/**
 * @brief Non-blocking check and read of the UART hardware buffer.
 * @param b Pointer to store the received byte.
 * @return 1 if a new byte was successfully read, 0 if the buffer is empty.
 */
static uint8_t HAL_UART_Read(uint8_t *b){
    if(UCSR0A & (1<<RXC0)){ // RXC0 bit is set when unread data is in the UDR0 buffer
        *b = UDR0; 
        return 1; 
    }
    return 0;
}

/** * @brief Initializes the Analog-to-Digital Converter (ADC).
 * @details Configures the ADC for single-ended reads on BATT_CH, referencing AVCC (5V).
 * Sets the ADC clock prescaler to 128 (16 MHz / 128 = 125 kHz), which falls within 
 * the optimal 50-200 kHz range for maximum 10-bit resolution accuracy.
 */
static void HAL_ADC_Init(void){
    // Disable digital input buffer on A0 to prevent shoot-through 
    // current and logic toggling from intermediate analog voltages
    DIDR0 |= (1<<ADC0D);

    ADMUX  = (1<<REFS0)|BATT_CH; 
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Enable ADC, Prescaler = 128
}

/** * @brief Performs a blocking ADC conversion.
 * @details Initiates a conversion and polls the ADSC (Start Conversion) bit 
 * until the hardware clears it, indicating the read is complete (~104 µs).
 * @return uint16_t The 10-bit ADC result (0 to 1023).
 */
static uint16_t HAL_ADC_Read(void){
    ADCSRA |= (1<<ADSC); 
    while(ADCSRA & (1<<ADSC)); 
    return ADC;
}

/** * @brief Configures Timer0 in Clear Timer on Compare Match (CTC) mode.
 * @details Generates a highly precise 1 ms interrupt.
 * Math: 
 * - Prescaler = 64. Timer Clock = 16 MHz / 64 = 250 kHz (4 µs per tick).
 * - OCR0A = 249. Timer counts from 0 to 249 (250 steps).
 * - Period = 250 steps * 4 µs = 1000 µs = 1 ms.
 */
static void HAL_Timer_Init(void){
    TCCR0A = (1<<WGM01);               // Set CTC mode (Clear Timer on Compare Match)
    TCCR0B = (1<<CS01)|(1<<CS00);      // Set Prescaler to 64
    OCR0A  = 249;                      // Compare match threshold for 1 ms
    TIMSK0 = (1<<OCIE0A);              // Enable Timer0 Output Compare Match A Interrupt
}

/*────────────────────────── Application Tasks ─────────────────────────────*/

/**
 * @brief Samples external telemetry and updates the global logical state.
 * @param c Pointer to the centralized state context.
 */
static void Task_Update(Context *c){
    uint8_t in;
    if(HAL_UART_Read(&in)) c->state = in;
    
    uint16_t raw = HAL_ADC_Read();
    
    // Convert 10-bit ADC reading (0-1023) to the physical voltage present at the pin
    float vpin   = (raw / 1023.0f) * VREF;
    
    // Algebraically reverse the voltage divider to find the true battery voltage
    c->vbat      = vpin / (R2 / (R1 + R2));
    
    // Evaluate if the calculated voltage has crossed the critical threshold
    c->low       = (c->vbat < LOW_BATT);
}

/**
 * @brief Schedules and drives the acoustic warning system.
 * * @details Uses non-blocking modulo arithmetic against the global millisecond 
 * tick to schedule when the buzzer should be active. When active, it uses a 
 * blocking delay to generate a 2 kHz square wave directly on the digital pin.
 * * Buzzer Patterns:
 * - Low battery   : 400 ms cycle (200 ms ON, 200 ms OFF)
 * - FAILSAFE (4)  : 100 ms cycle (50 ms ON, 50 ms OFF)
 * - All others    : Silent
 * * @param c Pointer to the centralized state context.
 */
static void Task_Output(Context *c){
    uint32_t now = get_ms();
    uint8_t toggle_buzz = 0;

    // Evaluate the temporal windows for active warnings
    if (c->low) {
        /* Low battery -> Moderately fast beep */
        if ((now % 400) < 200) toggle_buzz = 1;
    } 
    else if (c->state == 4) {
        /* Failsafe state -> Rapid panic beep */
        if ((now % 100) < 50) toggle_buzz = 1;
    }

    // Physical waveform generation
    if (toggle_buzz) {
        /* * To generate a passive tone, the membrane must be pushed and pulled 
         * at the target frequency.
         * For a 2 kHz tone:
         * Period $T = \frac{1}{f} = \frac{1}{2000} = 500 \mu s$
         * Therefore, we hold the pin HIGH for 250 µs, and LOW for 250 µs.
         */
        PORTD |=  (1 << BUZZ_PIN);
        _delay_us(250);
        PORTD &= ~(1 << BUZZ_PIN);
        _delay_us(250);
    } else {
        /* Enforce silence when no warnings are active */
        PORTD &= ~(1 << BUZZ_PIN);
    }
}

/*────────────────────────── Main Program ───────────────────────────────────*/

int main(void){
    // Initialize Hardware Peripherals
    HAL_GPIO_Init();
    HAL_UART_Init();
    HAL_ADC_Init();
    HAL_Timer_Init();
    
    // Enable Global Interrupts (required for Timer0 and the millis() clock)
    sei();

    // Initialize state context to zero
    Context ctx = {0};
    
    // Infinite Super-Loop
    for(;;){
        Task_Update(&ctx); // Gather data
        Task_Output(&ctx); // Actuate outputs
    }
}
