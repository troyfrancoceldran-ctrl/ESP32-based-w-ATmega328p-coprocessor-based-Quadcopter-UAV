#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define LED_PIN  PD6
#define BUZZ_PIN PD5
#define BATT_PIN 0

void WS_send(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (g & 0x80) { PORTD |= (1<<LED_PIN); _delay_us(0.7); PORTD &= ~(1<<LED_PIN); _delay_us(0.6); }
    else          { PORTD |= (1<<LED_PIN); _delay_us(0.35); PORTD &= ~(1<<LED_PIN); _delay_us(0.8); }
    g <<= 1;
  }
  for (uint8_t i = 0; i < 8; i++) {
    if (r & 0x80) { PORTD |= (1<<LED_PIN); _delay_us(0.7); PORTD &= ~(1<<LED_PIN); _delay_us(0.6); }
    else          { PORTD |= (1<<LED_PIN); _delay_us(0.35); PORTD &= ~(1<<LED_PIN); _delay_us(0.8); }
    r <<= 1;
  }
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) { PORTD |= (1<<LED_PIN); _delay_us(0.7); PORTD &= ~(1<<LED_PIN); _delay_us(0.6); }
    else          { PORTD |= (1<<LED_PIN); _delay_us(0.35); PORTD &= ~(1<<LED_PIN); _delay_us(0.8); }
    b <<= 1;
  }
  _delay_us(50);
}

uint16_t read_adc() {
  ADMUX = (1 << REFS0) | BATT_PIN;
  ADCSRA = (1 << ADEN) | (1 << ADSC) | 7;
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

int main(void) {
  DDRD |= (1<<LED_PIN) | (1<<BUZZ_PIN);
  uint8_t state = 0;
  uint16_t v;
  while (1) {
    v = read_adc();                // battery monitor
    float volts = (v / 1023.0f) * 5.0f * (10000+4700)/4700.0;
    if (volts < 10.5) {
      PORTD |= (1<<BUZZ_PIN);
      WS_send(255,0,0);            // red flashing low battery
      _delay_ms(200);
      PORTD &= ~(1<<BUZZ_PIN);
      WS_send(0,0,0);
      _delay_ms(200);
      continue;
    }

    if (state == 0) WS_send(0,0,255);
    else if (state == 1) WS_send(255,255,0);
    else if (state == 2) WS_send(0,255,0);
    else if (state == 3) WS_send(255,0,0);
    else WS_send(150,0,255);

    if (UCSR0A & (1 << RXC0)) state = UDR0; // receive 1 byte from ESP32
    _delay_ms(20);
  }
}
