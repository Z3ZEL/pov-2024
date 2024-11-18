#include <stdint.h>
#define PI 3.14159
#define DRAW_BUFFER_SIZE 128

struct draw_data {
    uint16_t buffer[DRAW_BUFFER_SIZE];
};

void draw_buffer_init();
void draw_at(double rad, uint16_t data);
uint16_t get_draw_at(double rad);
void reset_buffer();
