#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h> 
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "draw.h"
#include "conversion.h"


// CONSTANTS
#define BAUD 38400
#define MYUBRR F_CPU/16/BAUD-1
#define TIMER0_PRESCALER 1024


//PIN DEFINITION
#define OE_PIN      PC1   // Output Enable pin (connect to OE)
#define LE_PIN      PC2   // Latch Enable pin (connect to LE)
#define SDI_PIN     PB3   // Serial Data Input pin (connect to SDI)
#define CLK_PIN     PB5   // Clock pin (connect to CLK)
#define SS_PIN PB2  
#define HALL_PIN PD2


//GLOBAL VARIABLES
uint16_t last_timer;
uint16_t last_second_state = 0;
int need_redraw_second = 0, need_redraw_digits = 1;
int overflow_count = 0;
int second = 0;
int minute = 0;
int hour = 0;
vector_c_t hour_first_digit_origin = {8,17};
vector_c_t hour_second_digit_origin = {18,17};
vector_c_t minute_first_digit_origin = {8,5};
vector_c_t minute_second_digit_origin = {18,5};

/// INTERRUPTIONS HANDLERS ///////
ISR(INT0_vect) {
    // Interrupt service routine for INT0 (PD2)
    // Handle the hall sensor interrupt here
    // For example, toggle an LED or update a counter
    PORTD ^= (1 << PD6); // Toggle PD6 (LED)

    last_timer = TCNT1;
    TCNT1 = 0;
}

ISR(TIMER0_COMPA_vect) 
{
    // Handle Timer0 compare match interrupt
    // Increment the timer counter
    TCNT0 = 0;

    overflow_count++;
    if (overflow_count == 1625)
    {
        second++;
        overflow_count = 0;
        need_redraw_second = 1;

        if(second == 60){
            second = 0;
            minute++;
            need_redraw_digits = 1;
        }
        if (minute == 60){
            hour++;    
        }
        if (hour == 24){
            hour = 0;
        }
    }
}
/// INTERRUPT HANDLERS ///////



/// LED CONTROLLER METHODS ///
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
/// LED CONTROLLER METHODS ///

/// INITIALIZATION METHODS ///
void TimerInit(){

    TCNT0 = 0;
    //valeur supérieur de l'horloge (125)
    OCR0A = 0b01111100;
    //Activation de l'interrupt
    //Activation des interrupts globaux
    SREG |= (1 << SREG_I);
    //Activation du flag de la comparaison de OCR0A
    // TIFR0 |= (1 << OCF0A);
    //Activation de l'interrupt
    TIMSK0 |= (1 << OCIE0A);  
    //mode : CTC
    TCCR0A |= (1 << WGM01);
    //prescailer de 64
    TCCR0B |= (1 << CS01) | (1 << CS00);

}

void HallSensor_Init(){
    DDRD &= ~(1 << HALL_PIN);

    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0); 
}
/////////////////////////////


/// UTILS METHODS //////////
void display_second(uint8_t second){
    draw_force_at(second_to_rad(second-1), last_second_state);
    float rad = second_to_rad(second);
    last_second_state = get_draw_at(rad);
    draw_at(rad, 0x0FFF);
}

void extract_digit(int number, int* first_digit, int* second_digit){
    *first_digit = number / 10;
    *second_digit = number % 10;
}
float get_rad_position(){
    return 2 * PI * TCNT1 / last_timer;
}
/////////////////////////////








int main(){
    DDRD |= (1 << PD6); 
    
    draw_buffer_init();
    SPI_Init();
    USART_Init(MYUBRR);
    HallSensor_Init();
    TimerInit();
    

    // Set the prescaler to 64
    TCCR1B |= (1 << CS11) | (1 << CS10);
    TCNT1 = 0;

    sei();
    
    
    while(1){        
        
        // Draw the current position
        float current_rad = get_rad_position();
        uint16_t data = get_draw_at(current_rad);      
        MBI5024_Send(data);
        

        // _delay_us(500);
        

        //Check if it needs to redraw something
        if (need_redraw_second){
            display_second(second);
            need_redraw_second = 0;
        }
        if(need_redraw_digits){
            int first_digit, second_digit;
            reset_draw();
            extract_digit(hour, &first_digit, &second_digit);
            draw_int_at(hour_first_digit_origin, first_digit, 2);
            draw_int_at(hour_second_digit_origin, second_digit, 2);

            extract_digit(minute, &first_digit, &second_digit);
            draw_int_at(minute_first_digit_origin, first_digit, 2);
            draw_int_at(minute_second_digit_origin, second_digit, 2);

            need_redraw_digits = 0;
        }
        //////////////////////////


        // Listen for user time input
        if (USART_LineAvailable()) {
            char buffer[64];
            USART_ReadLn(buffer, sizeof(buffer));

            int t_hour, t_minute, t_second;
            sscanf(buffer, "%d:%d:%d", &t_hour, &t_minute, &t_second);
            if (t_hour < 0 || t_hour > 23 || t_minute < 0 || t_minute > 59 || t_second < 0 || t_second > 59) {
                USART_Printf("Invalid time format\n");
                continue;
            }
            cli();

            hour = t_hour;
            minute = t_minute;
            second = t_second;

            need_redraw_digits = 1;
            need_redraw_second = 1;

            USART_Printf("Time set to %d:%d:%d\n", hour, minute, second);
            sei(); 

            
        }

      

 

    }
}