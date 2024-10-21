#include <avr/io.h>
#include <util/delay.h>
#define BAUD 38400
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr)
{
    /*Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;

    /* Enable receiver and transmitter PIN */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit(unsigned char data)
{
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)))
    ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

int main(){
    DDRD |= (1 << PD6); 
    USART_Init(MYUBRR);
    while(1){
        PORTD |= (1 << PD6); 


        _delay_ms(1000);


        PORTD &= ~(1 << PD6);

        USART_Transmit('H');

        _delay_ms(1000);

    }
}