#ifndef USART_H
#define USART_H
#include <avr/io.h>
#include <stdlib.h>



uint8_t USART_Available();
uint8_t USART_Read_byte();
uint8_t USART_LineAvailable();
void USART_ReadLn(char* buffer, uint8_t size);
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void USART_Printf(const char* format, ...);

#endif