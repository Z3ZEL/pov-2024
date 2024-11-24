#ifndef DRAW_H
#define DRAW_H
#include <stdint.h>
#include "conversion.h"

#define RHO_CORRESPONDANCE rho_correspondance
#define DIGIT_CORRESPONDANCE digit_correspondance

#define V_LINE "0,0 0,15"

#define DIGIT_1 "0,4 1,5 2,6 2,0"
#define DIGIT_2 "0,4 1,5.5 2.5,5.5 3,4 1.5,2 0,0 3,0"
#define DIGIT_3 "0.25,4 1.5,5 3,4 1.5,2.5 3,1 1.5,0 0.25,1"
#define DIGIT_4 "1,4 0,1 2,1 2,2 2,0"
#define DIGIT_5 "3,4 0,4 0,2 1.2,2.2 2.5,2 3,1 2.5,0.2 1.2,0 0,0.4"
#define DIGIT_6 "1.5,4 0.2,1.8 0,1 0.2,0.2 1,0 1.8,0.2 2,1 1.8,1.8 1,2 0.2,1.8"
#define DIGIT_7 "0,4 2,4 1,2 0,2 2,2 1,2 0,0"
#define DIGIT_8 "0.7,4 0,3 1,2 2,1 1.3,0 0.7,0 0,1 2,3 1.3,4 0.7,4"
#define DIGIT_9 "1.5,0 0.2,2.2 0,3 0.2,3.8 1,4 1.8,3.8 2,3 1.8,2.2 1,2 0.2,2.2"
#define DIGIT_0 "0,4 0,0 2,0 2,4 0,4"



struct draw_data {
    uint16_t buffer[POLAR_PHI_DISCRETISATION];
};

void draw_buffer_init();
void draw_at(double rad, uint16_t data);
void draw_digit_at(vector_c_t origin,const char* digit, double scale);
void draw_int_at(vector_c_t origin, int number, double scale);
void draw_point_at(vector_c_t point);
uint16_t get_draw_at(double rad);
void reset_draw();


#endif
