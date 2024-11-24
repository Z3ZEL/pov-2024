#ifndef CONVERSION_H
#define CONVERSION_H

#include <stdint.h>
#define PI 3.14159
#define POLAR_PHI_DISCRETISATION 128
#define POLAR_RHO_DISCRETISATION 16


struct vector_cartesian {
    double x;
    double y;
} typedef vector_c_t;


struct vector_polar {
    double rho;
    double phi;
} typedef vector_p_t;


double second_to_rad(uint8_t second);
int rad_to_index(int length, double rad);
vector_p_t cartesian_to_polar(vector_c_t v);

#endif



