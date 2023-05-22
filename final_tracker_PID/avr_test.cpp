#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

uint16_t ADC0Read(const int channel);

uint16_t ADC1Read(const int channel);

uint16_t ADC2Read(const int channel);

void motorInit();

void serialOutput();

void USART_putstring(char* StringPtr);

void PIDcontrol(float* error, float* Kp, float* Ki, float* Kd, const float* Tp, float* integral, float* lastError, float* derivative);

int main(void) {
 motorInit();
 
 // serialOutput();

 float Kp = 215;
 float Ki = 0.1;
 float Kd = 150;

 // float Kp = 380;
 // float Ki = 0;
 // float Kd = 350;

 const float Tp = 255;
 const float offset = 900;

 float integral = 0;
 float lastError = 0;
 float derivative = 0;

 while(1) {
        // CLKPR=0b10000000; // set timer for serial output
        // CLKPR=0b00000000; // set timer for serial output
       
  ADCSRA |= (1<<ADEN);

  float sv0 = 0;
  float sv1 = 0;
  float sv2 = 0;

  sv0 = (float)ADC0Read(1); // left
  sv1 = (float)ADC1Read(0); // right
  sv2 = (float)ADC2Read(2); // mid

  float error = 0;
  error += sv0;
  error += sv1;
  error += sv2;
  
        error = offset/error;
        
  // if (sv1>100) {   // 偏左
        //     error *= -1.1;
  // }
  // else if (sv0>200) {
  //  error *= 1.2; // 偏右 左輪減速
  // }
  if (sv1>100 && sv0>200 && sv2>200) {
   error = lastError;
  }  
  else if (sv1>190) {   // 偏左
            error *= -1.2;
  }
        else if (sv0>190) {
   error *= 1.2; // 偏右 左輪減速
  }
  else error = 0 ; // straight
   
  PIDcontrol(&error, &Kp, &Ki, &Kd, &Tp, &integral, &lastError, &derivative);

     // char Buffer[50];
        // char *intStr = itoa((float) sv0, Buffer, 10);
        // strcat(intStr, ", ");
        // USART_putstring(intStr);

        // intStr = itoa((float) sv1, Buffer, 10);
        // strcat(intStr, ", ");
        // USART_putstring(intStr);

        // intStr = itoa((float) sv2, Buffer, 10);
        // strcat(intStr, ", ");
        // USART_putstring(intStr);

        // intStr = itoa((float) error, Buffer, 10);
        // strcat(intStr, "\n");
        // USART_putstring(intStr);

        // _delay_ms(500);
        
 }

 return 0;
}

uint16_t ADC0Read(const int channel) {
 ADMUX = 0b01000000;
 ADMUX |= channel;
 ADCSRA |= (1<<ADSC) | (1<<ADIF);
 while ( (ADCSRA & (1<<ADIF)) == 0);
 ADCSRA &= ~(1<<ADSC);
 return ADC;
}

uint16_t ADC1Read(const int channel) {
 ADMUX = 0b01000001;
 ADMUX |= channel;
 ADCSRA |= (1<<ADSC) | (1<<ADIF);
 while ( (ADCSRA & (1<<ADIF)) == 0);
 ADCSRA &= ~(1<<ADSC);
 return ADC;
}

uint16_t ADC2Read(const int channel) {
 ADMUX = 0b010000010;
 ADMUX |= channel;
 ADCSRA |= (1<<ADSC) | (1<<ADIF);
 while ( (ADCSRA & (1<<ADIF)) == 0);
 ADCSRA &= ~(1<<ADSC);
 return ADC;
}

void motorInit() {
 CLKPR=(1<<CLKPCE);
 CLKPR=0b00000011; // set clk to 1Mhz
 DDRD=0xFF; // PORTD as output
 DDRB=0xFF; // PORTB as output
 
    DDRC = 0;
 TCCR0A=0b10100001; // phase correct PWM
 TCCR0B=0b00000010; // timer prescaler
 TCCR2A=0b10100001; // phase correct PWM
 TCCR2B=0b00000010; // timer prescaler
}

void serialOutput() {
 unsigned int BaudR = 9600;
 unsigned int ubrr = (F_CPU / (BaudR*16UL))-1;
 UBRR0H = (unsigned char)(ubrr>>8);
 UBRR0L = (unsigned char)ubrr;
 UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
 UCSR0B |= (1<<TXEN0);
}

void USART_putstring(char* StringPtr) {
 while(*StringPtr != 0x00){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = *StringPtr;
  StringPtr++;
 }
}

void PIDcontrol(float* error, float* Kp, float* Ki, float* Kd, const float* Tp, float* integral, float* lastError, float* derivative) {
 *integral += *error;
 *derivative = *error-*lastError;
 
 float turn = *Kp**error + *Ki**integral + *Kd**derivative; // revice turn

    float right = *Tp+turn;
    float left = *Tp-turn;

    if (right>255) right = 255;
 if (right<0) right = 0;

    if (left>255) left = 255;
    if (left<0) left = 0;

 OCR0A = right; // PD3 lN4 MOTOR2正轉 (right)
 OCR2A = left; // PD5 lN2 MOTOR1正轉 (left)
 
 *lastError = *error;
}