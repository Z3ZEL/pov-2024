#include "usart.h"
#include <avr/interrupt.h>
#include "ring_buffer.h"
#include <stdarg.h>
#include <stdio.h>


ring_buffer_t tx_buffer;
ring_buffer_t rx_buffer;
uint8_t usart_line_available = 0;

void USART_Init(unsigned int ubrr)
{
    /*Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;

    /* Enable receiver and transmitter PIN */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    

    ring_buffer_init(&tx_buffer);
}


uint8_t USART_LineAvailable(){
    return usart_line_available;
}
uint8_t USART_Available() {
    return ring_buffer_available_bytes(&rx_buffer);
}

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

ISR(USART_RX_vect) {
    /* Read the received byte */
    uint8_t data = UDR0;

    if (data == '\n' || data == '\r') {
        usart_line_available = 1;
    }
    /* Put the received byte in the buffer */
    ring_buffer_put(&rx_buffer, data);

    
}
void USART_ReadLn(char* buffer, uint8_t size) {
    uint8_t i = 0;
    usart_line_available = 0;
    while (i < size - 1) {
        while (!USART_Available());
        char c = ring_buffer_get(&rx_buffer);
        if (c == '\n' || c == '\r') {
            break;
        }
        buffer[i++] = c;
    }
    buffer[i] = '\0';
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

void USART_Transmit(unsigned char data)
{
    ring_buffer_put(&tx_buffer, data);

    // Activer l'interruption de transmission si elle ne l'est pas déjà
    UCSR0B |= (1 << UDRIE0);
}