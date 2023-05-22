


// Define the microcontroller we are using
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

/*
 * final_ob.cpp
 *
 * Created: 2023/5/20 下午 02:21:20
 * Author : yylun
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
uint16_t ADCRead(const int);

void leftTurn(int speed);
void rightTurn(int speed);
void straight(int speedR, int speedL);
void bigTurn(int speed);
float factor = 2;
// int delay_t = 100;
int main(void)
{
    CLKPR = (1 << CLKPCE);
    CLKPR = 0b00000000; // time division 1
    DDRC = 0;
    DDRB = 0xFF;
    DDRD = 0xFF;
    // OCR0A=160;
    // OCR0B=0;
    TCCR0A = 0b10100011; // fast PWM, non-inverted
    TCCR0B = 0b00000010; // timer prescaler

    // OCR2A=160;
    // OCR2B=0;
    TCCR2A = 0b10100011; // fast PWM, non-inverted
    TCCR2B = 0b10100010;

    ADCSRA |= (1 << ADEN);
    int setspeedR = 180, setspeedL = 185, bigSpeed = 90, lSpeed = 110, rSpeed = 110; // 160 155 20 115 best for now
    // int setspeedR = 160, setspeedL = 165, bigSpeed = 90, lSpeed = 130, rSpeed = 130; // 160 155 20 115 best for now

    while (1)
    {

        float F = 0, L = 0, R = 0;
        for (int i = 0; i < 3; i++)
        {
            F += (float)ADCRead(3);
            L += (float)ADCRead(4);
            R += (float)ADCRead(5);
        }
        F = F / 3;
        L = L / 3;
        R = R / 3;
        // if (true) //straight
        OCR0B = 0;
        OCR2B = 0;
        if (L < 400) // 離牆太遠則往右修正
        {
            if (F < 500) // 偵測前方有障礙
            {
                bigTurn(bigSpeed);
            }
            else if (R < 500) // 距離右側太近
            {
                leftTurn(lSpeed);
                /*_delay_ms(100);*/
            }
            else
            {
                rightTurn(rSpeed);
            }
        }

        else // 離牆保持適當距離
        {

            if (F < 500) // 偵測前方有障礙
            {
                bigTurn(bigSpeed);
            }
            else if (L < 800 && L > 500) // 距離右側太近
            {
                leftTurn(lSpeed);
            }
            else if (R < 500) // 距離右側太近
            {
                leftTurn(lSpeed);
            }
            else
            {
                straight(setspeedR, setspeedL);
            }
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

void leftTurn(int speed)
{
    // speed = speed * 0.6;
    OCR0A = speed;
    OCR2A = speed / 1.8;
}
void rightTurn(int speed)
{
    OCR2A = speed;
    // OCR0A=setspeedL-lightTurn;
    OCR0A = speed / factor;
}

void straight(int speedR, int speedL)
{
    OCR0A = speedR;
    OCR2A = speedL;
    OCR0B = 0;
    OCR2B = 0;
}
void bigTurn(int speed)
{
    OCR0A = speed;
    // OCR2A=setspeedL - sharpTurn;
    OCR2A = 0;
    OCR2B = speed;
    _delay_ms(100);
}
