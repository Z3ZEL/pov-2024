#include "draw.h"

struct draw_data draw_data;

void draw_buffer_init(){
    for(int i = 0 ; i<DRAW_BUFFER_SIZE ; i++){
        draw_data.buffer[i] = 0x8000;
    }

    draw_at(PI,0xC000);
    draw_at(3*PI/2,0xC000);
    draw_at(PI/2,0xC000);
    draw_at(0,0xC000);

}
void draw_at(double rad, uint16_t data){
    int index = (int)((DRAW_BUFFER_SIZE - 1) * rad) / (2 * PI);
    draw_data.buffer[index] = data;
}
uint16_t get_draw_at(double rad){
    int index = (int)((DRAW_BUFFER_SIZE - 1) * rad) / (2 * PI);
    return draw_data.buffer[index];
}
void reset_buffer(){
    draw_buffer_init();
}
