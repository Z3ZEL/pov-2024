#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h> 
#include <stdarg.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ring_buffer.h"

#define BAUD 38400
#define MYUBRR F_CPU/16/BAUD-1


//LED
#define OE_PIN      PC1   // Output Enable pin (connect to OE)
#define LE_PIN      PC2   // Latch Enable pin (connect to LE)
#define SDI_PIN     PB3   // Serial Data Input pin (connect to SDI)
#define CLK_PIN     PB5   // Clock pin (connect to CLK)
#define SS_PIN PB2  


ring_buffer_t tx_buffer;
// ring_buffer_t rx_buffer;


uint8_t USART_Available();
uint8_t USART_Read_byte();

// Gestionnaire d'interruption pour la transmission
ISR(USART_UDRE_vect) {
    /* Check if there's data in the buffer */
    if (ring_buffer_available_bytes(&tx_buffer) > 0) {
        /* Transmit a byte from the buffer */
        UDR0 = ring_buffer_get(&tx_buffer);
    } else {
        /* Disable interrupt if the buffer is empty */
        UCSR0B &= ~(1 << UDRIE0);
    }
}

void USART_Init(unsigned int ubrr)
{
    /*Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;

    /* Enable receiver and transmitter PIN */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    

    ring_buffer_init(&tx_buffer);

    sei();

}

void USART_Transmit(unsigned char data)
{
    ring_buffer_put(&tx_buffer, data);

    // Activer l'interruption de transmission si elle ne l'est pas déjà
    UCSR0B |= (1 << UDRIE0);
}


void USART_Printf(const char* format, ...) {
    char buffer[128];  // Buffer to hold the formatted string
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    for (int i = 0; buffer[i] != '\0'; i++) {
        USART_Transmit(buffer[i]);
    }
}
// Function to initialize SPI
void SPI_Init(void) {
    /* Set MOSI, SCK, and LE as output, others as input */
    DDRB |= (1 << SDI_PIN) | (1 << CLK_PIN) | (1 << SS_PIN);
    DDRC |= (1 << LE_PIN) | (1 << OE_PIN);

    PORTB |= (1 << SS_PIN);  // Set SS high to stay in master mode
    
    /* Enable SPI, Master, set clock rate fck/16 */
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

    
void MBI5024_Latch() {
    // Toggle LE pin (latch enable)
    PORTC |= (1 << LE_PIN);   // Set LE high
    _delay_us(1);             // Small delay for latch
    PORTC &= ~(1 << LE_PIN);  // Set LE low
}
void MBI5024_Send(uint16_t data) {
    // Send the high byte first
    SPDR = (data >> 8);
    while (!(SPSR & (1 << SPIF)));  // Wait for transmission to complete
    
    // Send the low byte
    SPDR = data & 0xFF;
    while (!(SPSR & (1 << SPIF)));  // Wait for transmission to complete

    MBI5024_Latch();        

}


int main(){
    DDRD |= (1 << PD6); 
    DDRD &= ~(1 << PD2);//HALL SENSOR

    SPI_Init();
    PORTC &= ~(1 << OE_PIN);  // Set OE low to enable outputs

    USART_Init(MYUBRR);
    
    TCCR0B |= (1 << CS02) | (1 << CS00);  
    TCNT0 = 0;


    
    uint8_t last_state = 0;
    uint8_t last_speed = 0;
        
    while(1){
        PORTD |= (1 << PD6); 
        PORTD &= ~(1 << PD6);

        //Retrieve the data from the hall sensor
        if(PIND & (1 << PD2)){
            // No magnetic field
            if (last_state == 1){
                //Calculate the speed
                uint8_t speed = TCNT0;
                TCNT0 = 0;
                USART_Printf("Speed: %d\n", speed);
                last_speed = speed;

            }
            MBI5024_Send(0x0000);      
            last_state = 0;
        }else{
            //Magnetic field
            
            MBI5024_Send(0xFFFF);
            last_state = 1;
        }

        

        

        


    }
}