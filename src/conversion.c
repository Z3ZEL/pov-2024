#include "conversion.h"
#include <math.h>

#define K1 0.00004  // Example distortion coefficient
#define K2 0.000004 // Example distortion coefficient


float second_to_rad(uint8_t second){
    return 2 * PI - ((second * 2 * PI) / 60);
}

int rad_to_index(int length, float rad){
    // Normalize rad to be within [0, 2*PI)
    while (rad < 0) {
        rad += 2 * PI;
    }
 

    return (int)((length - 1) * rad) / (2 * PI);
}



vector_p_t cartesian_to_polar(vector_c_t v){
    //Translate vector first
    v.x -= POLAR_RHO_DISCRETISATION;
    v.y -= POLAR_RHO_DISCRETISATION;

    vector_p_t p;
    p.rho = sqrt(v.x * v.x + v.y * v.y);
    p.phi = 2*PI - atan2(v.x, v.y) ;
    p.phi = fmod(p.phi, 2*PI);


    // Correcting radial distortion
    float r2 = p.rho * p.rho;
    float distortion_factor = 1 + K1 * r2 + K2 * r2 * r2;
    // float distortion_factor = 1;
    p.rho *= distortion_factor;



    return p;
}