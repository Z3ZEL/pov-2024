#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h> 
#include <stdarg.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ring_buffer.h"
#include "draw.h"

#define BAUD 38400
#define MYUBRR F_CPU/16/BAUD-1
#define TIMER0_PRESCALER 1024

//Hall sensor
#define HALL_PIN PD2

//LED
#define OE_PIN      PC1   // Output Enable pin (connect to OE)
#define LE_PIN      PC2   // Latch Enable pin (connect to LE)
#define SDI_PIN     PB3   // Serial Data Input pin (connect to SDI)
#define CLK_PIN     PB5   // Clock pin (connect to CLK)
#define SS_PIN PB2  


//Void def
void MBI5024_Send(uint16_t data);
void USART_Printf(const char* format, ...);

ring_buffer_t tx_buffer;
uint8_t last_timer;




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

ISR(INT0_vect) {
    // Interrupt service routine for INT0 (PD2)
    // Handle the hall sensor interrupt here
    // For example, toggle an LED or update a counter
    PORTD ^= (1 << PD6); // Toggle PD6 (LED)

    last_timer = TCNT0;
    TCNT0 = 0;
    
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

    PORTC &= ~(1 << OE_PIN);  // Set OE low to enable outputs
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

void HallSensor_Init(){
    DDRD &= ~(1 << HALL_PIN);

    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0); 
}

int get_rad_position(){
    return 2 * PI * TCNT0 / last_timer;
}


int main(){
    DDRD |= (1 << PD6); 
    
    draw_buffer_init();
    SPI_Init();
    USART_Init(MYUBRR);
    HallSensor_Init();
    
    // Set the prescaler to 1024
    TCCR0B |= (1 << CS02) | (1 << CS00);
    TCNT0 = 0;

    draw_at(PI/2, 0xFFFF);

    sei();
    while(1){
        PORTD |= (1 << PD6); 
        PORTD &= ~(1 << PD6);

        
        int current_rad = get_rad_position();
        uint16_t data = get_draw_at(current_rad);
        MBI5024_Send(data);


        _delay_us(10);
        
      

    }
}