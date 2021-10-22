#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/delay.h>

#include "uart.h"

volatile unsigned int risingEdge = 0;
volatile unsigned int fallingEdge = 0;
volatile unsigned int highTime = 0;
volatile unsigned int counter = 0;
volatile unsigned int dutyCycle = 0;

volatile bool modeDiscrete = true;
volatile bool inputHigh = false;

char String[25];

void initialize()
{
    UART_init(BAUD_PRESCALER);
    cli();
    // GPIO
    DDRD |= (1<<DDD5); // PD6 as output buzzer
    DDRB &= ~(1<<DDB0); // PB0 as input echo
    DDRB |= (1<<DDB1); // PB1 as output trigger
    DDRD &= ~(1<<DDD7); //PB5 as input for button

    // Timer 1 setup
    // Prescaler as 8
    TCCR1B &= ~(1<<CS12);
    TCCR1B |= (1<<CS11);
    TCCR1B &= ~(1<<CS10);

    //Set Timer 1 to Normal mode
    TCCR1A &= ~(1<<WGM10);
    TCCR1A &= ~(1<<WGM11);
    TCCR1B &= ~(1<<WGM12);

    // Timer 0 Setup
    // Prescaler is 256
    TCCR0B &= ~(1<<CS00);
    TCCR0B &= ~(1<<CS01);
    TCCR0B |= (1<<CS02);

    // Timer0 set to PWM phase correct
    TCCR0B |= (1<<WGM02);
    TCCR0A &= ~(1<<WGM01);
    TCCR0A |= (1<<WGM00);

    TCCR0A |= (1<<COM0B1);
    TCCR0A &= ~(1<<COM0B0);
    OCR0A = 20;// dummy place holder will be changed by while loop
    OCR0B = OCR0A * 1/2;

    TCCR1B |= (1<<ICES1);// looking for rising edge
    TIFR1 |= (1<<ICF1);// clear interrupt flag for input capture
    TIMSK1 |= (1<<ICIE1);// enable input capture interrupt

    // clear power reduction for ADC
    PRR &= ~(1<<PRADC);
    // select Vref = AVcc
    ADMUX |= (1<<REFS0);
    ADMUX &= ~(1<<REFS1);

    // set the ADC Clock div by 128
    // 16M/128=125k
    ADCSRA |= (1<<ADPS0);
    ADCSRA |= (1<<ADPS1);
    ADCSRA |= (1<<ADPS2);

    // Select Channel 0
    ADMUX &= ~(1<<MUX0);
    ADMUX &= ~(1<<MUX1);
    ADMUX &= ~(1<<MUX2);
    ADMUX &= ~(1<<MUX3);
    // Set to free running
    ADCSRA |= (1<<ADATE);
    ADCSRB &= ~(1<<ADTS0);
    ADCSRB &= ~(1<<ADTS1);
    ADCSRB &= ~(1<<ADTS2);
    // Disable digital input buffer on ADC pin
    DIDR0 |= (1<<ADC0D);
    // Enable ADC
    ADCSRA |= (1<<ADEN);
    // Enable ADC Interrupt
    ADCSRA |= (1<<ADIE);
    // Start Conversion
    ADCSRA |= (1<<ADSC);

    sei();
}

ISR(ADC_vect)
{
    if(ADC < 202)
    {
        dutyCycle = 5;
    }
    else if((ADC >= 282) && (ADC <= 363))
    {
        dutyCycle = 10;
    }
    else if((ADC >= 364) && (ADC <= 444))
    {
        dutyCycle = 15;
    }
    else if((ADC >= 445) && (ADC <= 525))
    {
        dutyCycle = 20;
    }
    else if((ADC >= 526) && (ADC <= 606))
    {
        dutyCycle = 25;
    }
    else if((ADC >= 607) && (ADC <= 687))
    {
        dutyCycle = 30;
    }
    else if((ADC >= 688) && (ADC <= 768))
    {
        dutyCycle = 35;
    }
    else if((ADC >= 769) && (ADC <= 849))
    {
        dutyCycle = 40;
    }
    else if((ADC >= 850) && (ADC <= 930))
    {
        dutyCycle = 45;
    }
    else if( ADC >= 931)
    {
        dutyCycle = 50;
    }

    // Set duty cycle
    OCR0B = OCR0A * dutyCycle / 100;
    counter += 1;
}

void sendTrigSignal()
{
    if (inputHigh==false) {
        PORTB |= (1<<PORTB1);
        _delay_us(10);
        PORTB &= ~(1<<PORTB1);
    }
}

int buttonPress()
{
    if(PIND & (1<<PIND7)) {
        if(modeDiscrete == false) {
            modeDiscrete = true;
            sprintf(String,"Discrete \n");
            UART_putstring(String);
        } else if(modeDiscrete == true) {
            modeDiscrete = false;
            sprintf(String,"Continuous \n");
            UART_putstring(String);
        }
        _delay_ms(500);
    }
}


ISR(TIMER1_CAPT_vect)
{
    TCCR1B ^= (1<<ICES1);

    if(inputHigh == false) {
        inputHigh = true;
        risingEdge = ICR1;
    }
    else if (inputHigh == true) {
        inputHigh = false;
        fallingEdge = ICR1;

        // initial screening, 16240 counts is roughly 1.38 meters
        if(((fallingEdge - risingEdge) > 0) & ((fallingEdge - risingEdge) < 16240))
        {
            highTime = fallingEdge - risingEdge;
        }

        // continuous mode
        if(modeDiscrete == false)
        {
            OCR0A = 0.000504955 * highTime + 5.90101;
        }

        // discrete mode
        if(modeDiscrete == true) {
            if(highTime >= 0 && highTime < 2000)
            {
                OCR0A = 6;
            }
            else if(highTime >= 2000 && highTime < 4000)
            {
                OCR0A = 7;
            }
            else if(highTime >= 4000 && highTime < 6000)
            {
                OCR0A = 8;
            }
            else if(highTime >= 6000 && highTime < 8000)
            {
                OCR0A = 9;
            }
            else if(highTime >= 8000 && highTime < 10000)
            {
                OCR0A = 10;
            }
            else if(highTime >= 10000 && highTime < 12000)
            {
                OCR0A = 11;
            }
            else if(highTime >= 12000 && highTime < 14000)
            {
                OCR0A = 12;
            }
            else if(highTime >= 14000 && highTime < 16240)
            {
                OCR0A = 14;
            }
        }
        sendTrigSignal();
    }
}

int main(void)
{
    initialize();
    sendTrigSignal();
    while (1)
    {
        buttonPress();

        if(counter == 10) {
            counter = 0;
            sprintf(String,"ADC: %d \n", ADC);
            UART_putstring(String);
            sprintf(String,"Duty Cycle %d percent \n", dutyCycle);
            UART_putstring(String);
        }
    }
}


