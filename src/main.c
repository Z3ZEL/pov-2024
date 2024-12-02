#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h> 
#include <stdarg.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ring_buffer.h"
#include "draw.h"
#include "conversion.h"

#define BAUD 38400
#define MYUBRR F_CPU/16/BAUD-1
#define TIMER0_PRESCALER 1024
#define FREQ 13000000



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
ring_buffer_t rx_buffer;
uint16_t last_timer;




uint8_t USART_Available();
uint8_t USART_Read_byte();
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
    /* Put the received byte in the buffer */
    ring_buffer_put(&rx_buffer, data);

    
}

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

void USART_ReadLn(char* buffer, uint8_t size) {
    uint8_t i = 0;
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

uint8_t USART_Available() {
    return ring_buffer_available_bytes(&rx_buffer);
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

float get_rad_position(){
    return 2 * PI * TCNT1 / last_timer;
}




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
        
        float current_rad = get_rad_position();
        uint16_t data = get_draw_at(current_rad);      
        MBI5024_Send(data);

        // _delay_us(500);
        

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


        if (USART_Available()) {
        //     char buffer[128];
        //     USART_ReadLn(buffer, sizeof(buffer));

        //     int number;

        //     sscanf(buffer, "%d", &number);

        //     // reset_draw();

        //     vector_c_t first_digit_origin = {5,8};
        //     vector_c_t second_digit_origin = {20,8};

        //     int first_digit = number / 10;
        //     int second_digit = number % 10;

        //     // draw_int_at(first_digit_origin, first_digit, 3);
        //     // draw_int_at(second_digit_origin, second_digit, 3);
            

        //     USART_Printf("Number: %d\n", number);






        //     // convert "x y" string receive
        //     // int x,y;
        //     // sscanf(buffer, "%d %d", &x, &y);
        //     // USART_Printf("X: %d\n", x);
        //     // USART_Printf("Y: %d\n", y);
        //     // vector_c_t point = {x, y};
        //     // vector_p_t polar = cartesian_to_polar(point);


        //     // // draw_point_at(point);

            
            
            

        //     // //convert float to string
        //     // char print_buffer[6];
        //     // dtostrf(polar.rho, 4, 2, print_buffer);
        //     // USART_Printf("Rho: %s\n", print_buffer);
        //     // dtostrf(polar.phi, 4, 2, print_buffer);
        //     // USART_Printf("Phi: %s\n", print_buffer);

            
        }

      

        
    
        // uint8_t count = TCNT0;
        // USART_Printf("count: %d\n", count);
        // USART_Printf("Seconde: %d\n", second);
        // USART_Printf("Count: %d\n", overflow_count);


        // _delay_ms(1000);
                



    }
}