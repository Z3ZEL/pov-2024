#ifndef CONVERSION_H
#define CONVERSION_H

#include <stdint.h>
#define PI 3.14159
#define POLAR_PHI_DISCRETISATION 256
#define POLAR_RHO_DISCRETISATION 16


struct vector_cartesian {
    float x;
    float y;
} typedef vector_c_t;


struct vector_polar {
    float rho;
    float phi;
} typedef vector_p_t;


float second_to_rad(uint8_t second);
int rad_to_index(int length, float rad);
vector_p_t cartesian_to_polar(vector_c_t v);

#endif



