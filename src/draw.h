#ifndef DRAW_H
#define DRAW_H
#include <stdint.h>
#include "conversion.h"

#define RHO_CORRESPONDANCE rho_correspondance
#define DIGIT_CORRESPONDANCE digit_correspondance

#define V_LINE "0,0 0,15"

// Digit definition
#define DIGIT_1 "0,3 1,4 1,0"
#define DIGIT_2 "0,2.9 0.67,4 1.67,4 2,2.9 1,1.45 0,0 2,0"
#define DIGIT_3 "0.25,4 1.5,5 3,4 1.5,2.5 3,1 1.5,0 0.25,1"
#define DIGIT_4 "1,4 0,1 2,1 2,2 2,0"
#define DIGIT_5 "3,4 0,4 0,2 1.2,2.2 2.5,2 3,1 2.5,0.2 1.2,0 0,0.4"
#define DIGIT_6 "1.5,4 0.2,1.8 0,1 0.2,0.2 1,0 1.8,0.2 2,1 1.8,1.8 1,2 0.2,1.8"
#define DIGIT_7 "0,4 2,4 1,2 0,2 2,2 1,2 0,0"
#define DIGIT_8 "0.7,4 0,3 1,2 2,1 1.3,0 0.7,0 0,1 2,3 1.3,4 0.7,4"
#define DIGIT_9 "0.5,0 1.8,2.2 2,3 1.8,3.8 1,4 0.2,3.8 0,3 0.2,2.2 1,2 1.8,2.2"
#define DIGIT_0 "0,4 0,0 2,0 2,4 0,4"



struct draw_data {
    uint16_t buffer[POLAR_PHI_DISCRETISATION];
};


void draw_buffer_init();
/// @brief Draw data on a specific position (OR operation with current data)
void draw_at(float rad, uint16_t data);
/// @brief Draw multiples lines from string code definition with a specific origin and scale
/// @param origin  Origin of the drawing (Bound here are 32x32)
/// @param digit  String code definition of the drawing
/// @param scale  Scale of the drawing (<1 for zoom in, >1 for zoom out)
void draw_curve_at(vector_c_t origin,const char* digit, float scale);
/// @brief Draw digit at a specific position
/// @param origin  Origin of the drawing (Bound here are 32x32)
/// @param number  Number to draw (0-9)
/// @param scale  Scale of the drawing (<1 for zoom in, >1 for zoom out)
void draw_int_at(vector_c_t origin, int number, float scale);
/// @brief Draw a point at a specific position
/// @param point  Position of the point (Bound here are 32x32)
void draw_point_at(vector_c_t point);
/// @brief Get the data at a specific position
/// @param rad  Position in radian
/// @return Data at the position
uint16_t get_draw_at(float rad);
/// @brief Draw data on a specific position (Replace current data)
void draw_force_at(float rad, uint16_t data);
/// @brief Reset draw on the screen
void reset_draw();


#endif
