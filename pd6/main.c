#include <avr/io.h>
#include <util/delay.h>


int main()
{
    DDRD |= (1<<PD6);
    while (1)
    {
        PORTD |= (1<<PD6);
        _delay_ms(1000);
        PORTD &= ~(1<<PD6);
        _delay_ms(1000);
    }
    
}