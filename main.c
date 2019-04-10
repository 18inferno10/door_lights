#include <avr/io.h>
#include<avr/interrupt.h>
#include <util/delay.h>

//volatile uint16_t overflow=0;
uint16_t adc_value;
volatile int pos1=0, pos2=0, pos3=0, door_close=0;
void initADC()
{
ADMUX |= (1 << REFS0); //reference voltage on AVCC
ADCSRA |= (1 << ADEN) | (7<< ADPS0); //ADC clock prescaler /8
}
uint16_t ReadADC(uint8_t ch)
{
    ADMUX&=0xf8;
    ch=ch&0b00000111;
    ADMUX|=ch;

    //START CONVERSION
    ADCSRA|=(1<<ADSC);

    //WAIT FOR THE CONVERSION TO COMPLETE
    while(!(ADCSRA & (1<<ADIF)));

    //RAISE THE FLAG TO START THE NEXT CONVERSION (as the system resets it after conversion)
    ADCSRA|=(1<<ADIF);
    return(ADC);
}

void door(void)
{
    if(door_close==1)
    {
        adc_value=ReadADC(PC0);
        OCR1A=adc_value; //pb1 PIN to see PWM
       // if(OCR1A>=200)
        if(adc_value==0)
        {
             PORTB&=~(1<<PB3); //BUZZER OFF
        }
          if(adc_value>=0)
        {
            PORTB|=(1<<PB3);
        }
        door_close=0;
    }
}

void led_on(void)
{
    PORTB|=(1<<PB2);
}

int main(void)
{

    TCCR1A|=((1<<COM1A1)|(1<<WGM11)|(1<<WGM10));
    TCCR1B|=((1<<WGM12)|(1<<CS01)|(1<<CS00));
    TCNT1=0x00;

    DDRC &= ~(1<<PC0); //pot aka RPM
    DDRB |= (1 << PB1); //pin to see PWM

    DDRD&=~(1<<PD2); //SW1 CLEAR
    PORTD|=(1<<PD2); //SW1 SET

    DDRD&=~(1<<PD3); //SW2 CLEAR
    PORTD|=(1<<PD3); //SW2 SET

    DDRD&=~(1<<PD4); //SW3 CLEAR
    PORTD|=(1<<PD4); //SW3 SET

    DDRB&=~(1<<PB6); //SW4 CLEAR  //DOOR
    PORTB|=(1<<PB6); //SW4 SET

    DDRB|=(1<<PB3); //BUZZER OUTPUT
    DDRB|=(1<<PB2); //LED //LIGHT

    PCMSK2|=(1<<PCINT20); // pin change interrupt for PCINT20
    PCICR|=(1<<PCIE2); // enable pin change for PCINT20

    PCMSK0|=(1<<PCINT6); // pin change interrupt for PCINT6
    PCICR|=(1<<PCIE0); // enable pin change for PCINT6

    EICRA|=(1<<ISC00); //Any logical change on INT0 generates an interrupt request
    EIMSK|=(1<<INT0); //When the INT1 bit is set (one) and the I-bit in the status register (SREG) is set (one), the external pin interrupt is enabled.

    EICRA|=(1<<ISC10); //Any logical change on INT0 generates an interrupt request
    EIMSK|=(1<<INT1);
    sei(); //set global interrupt

    initADC();


    while (1)
    {
        if(pos1==1)//SW1 (light OFF)
        {
            PORTB&=~(1<<PB2); //light off
            door();
            pos1=0;
        }
        if(pos2==1)//SW2 (light ON)
        {
            //PORTB|=(1<<PB2); //LED ON
          //  _delay_ms(2000);
            led_on();
            door();
            pos2=0;
        }
         if(pos3==1)//SW2 (light ON)
        {
          //  PORTB|=(1<<PB2); //LED ON
          // _delay_ms(2000);
            led_on();
            door();
            pos3=0;
        }
        else
        {
            PORTB&=~(1<<PB3); //BUZZER OFF
           // PORTB&=~(1<<PB2); //LED //LIGHT OFF
        }
    }

    return (0);
}

ISR(INT0_vect)
{
    pos1=1;
}
ISR(INT1_vect)
{
    pos2=1;
}
ISR(PCINT2_vect)
{
    pos3=1;
}
ISR(PCINT0_vect) //pb6 //pcint6
{
    door_close=1;
}
