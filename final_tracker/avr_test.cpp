// Define the microcontroller we are using
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

/*
 * FinalProject.cpp
 *
 * Created: 2023/5/14 §U§» 09:47:26
 * Author : yylun
 */

// M=600,500 3.87 3.85 3.84 66.7% 530 155 3.86 3.85 3.84 100%

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
uint16_t ADCRead(const int);

int main(void)
{
  CLKPR = (1 << CLKPCE);
  CLKPR = 0b00000000; // time division 1
  DDRC = 0;
  DDRB = 0xFF;
  DDRD = 0xFF;
  OCR0A = 210;
  OCR0B = 0;
  TCCR0A = 0b10100011; // fast PWM, non-inverted
  TCCR0B = 0b00000010; // timer prescaler

  OCR2A = 210;
  OCR2B = 0;
  TCCR2A = 0b10100011; // fast PWM, non-inverted
  TCCR2B = 0b10100010;

  ADCSRA |= (1 << ADEN);
  int setspeedR = 130, setspeedL = 135, rightTurn = 130, leftTurn = 135; // 20 115 best for now

  while (1)
  {

    float R = 0, L = 0, M = 0;

    M += (float)ADCRead(2);
    R += (float)ADCRead(0);
    L += (float)ADCRead(1);

    if (M >= 330)
    {
      OCR0A = setspeedR;
      OCR2A = setspeedL;
      _delay_ms(5);
      if (M > 500 && R > 500 && L > 500)
      {
        _delay_ms(15);
      }
    }
    else if (R > 190 && M < 100) // ®Æ®≠§j¥T∞æ•™ ±j§O≠◊•ø
    {
      OCR0A = 0;
      OCR2A = leftTurn;
    }
    else if (L > 190 && M < 100) // ®Æ®≠§j¥T∞æ•k ±j§O≠◊•ø
    {
      OCR0A = rightTurn;
      OCR2A = 0;
    }
  }
}

uint16_t ADCRead(const int channel)
{
  ADMUX = 0b01000000;
  ADMUX |= channel;
  ADCSRA |= (1 << ADSC) | (1 << ADIF);
  while ((ADCSRA & (1 << ADIF)) == 0)
    ;
  ADCSRA &= ~(1 << ADSC);
  return ADC;
}