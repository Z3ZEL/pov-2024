#include "draw.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
uint16_t rho_correspondance[] = { 0x8000, 0x4000, 0x2000, 0x1000, 0x0800, 0x0400, 0x0200, 0x0100, 0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001};
char* digit_correspondance[] = {DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4, DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9};
struct draw_data draw_data;


void draw_buffer_init(){
    for(int i = 0 ; i<POLAR_PHI_DISCRETISATION ; i++){
        draw_data.buffer[i] = 0x8000;
    }

    draw_at(PI,0xC000);
    draw_at(3*PI/2,0xC000);
    draw_at(PI/2,0xC000);
    draw_at(0,0xC000);

}
void reset_draw(){
    draw_buffer_init();
}
void draw_at(float rad, uint16_t data){
    int index = rad_to_index(POLAR_PHI_DISCRETISATION, rad);
    uint16_t current_data = draw_data.buffer[index];
    draw_data.buffer[index] = data | current_data;
}

void draw_force_at(float rad, uint16_t data){
    int index = rad_to_index(POLAR_PHI_DISCRETISATION, rad);    
    draw_data.buffer[index] = data;
}
uint16_t get_draw_at(float rad){
    int index =  rad_to_index(POLAR_PHI_DISCRETISATION, rad);
    return draw_data.buffer[index];
}

void draw_point_at(vector_c_t vector_c_t){
    vector_p_t polar = cartesian_to_polar(vector_c_t);
    int polar_index = POLAR_RHO_DISCRETISATION - (int) polar.rho;
    if(polar_index >= POLAR_RHO_DISCRETISATION){
        polar_index = POLAR_RHO_DISCRETISATION - 1;
    }
    uint16_t data = RHO_CORRESPONDANCE[polar_index];

    draw_at(polar.phi, data);
}  


void interpolateLine(vector_c_t start, vector_c_t end, float step, vector_c_t result[], size_t *count) {
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float distance = sqrt(dx * dx + dy * dy);
    size_t numSteps = (size_t)(distance / step);
    if (numSteps == 0) {
        return; // Skip very short segments
    }

    for (size_t i = 0; i <= numSteps; i++) {
        float t = (float)i / numSteps;
        result[*count] = (vector_c_t){start.x + t * dx, start.y + t * dy};
        (*count)++;
    }
}

void draw_digit_at(vector_c_t origin, const char* digit, float scale) {
    // Parse the digit string
    vector_c_t points[10];

    char* digit_copy = strdup(digit);

    size_t count = 0;

    char* token = strtok(digit_copy, " ");
    while (token != NULL) {
        char* comma_pos = strchr(token, ',');
        if (comma_pos != NULL) {
            *comma_pos = '\0';
            float x = atof(token) * scale;
            float y = atof(comma_pos + 1) * scale;

            points[count++] = (vector_c_t){origin.x + x, origin.y + y};
        }
        token = strtok(NULL, " ");
    }

    // Interpolate lines between points
    vector_c_t interpolated[64];
    size_t interpolatedCount = 0;
    for (size_t i = 0; i < count; i++) {
        if (i == count - 1) {
            continue;
        }
        interpolateLine(points[i], points[(i + 1)], 0.25 * scale, interpolated, &interpolatedCount);
    }

    // Draw the interpolated lines
    for (size_t i = 0; i < interpolatedCount; i++) {
        draw_point_at(interpolated[i]);
    }

    free(digit_copy);
}

void draw_int_at(vector_c_t origin, int number, float scale) {
    if (number >= 10){
        number = 9;
    }else if(number < 0){
        number = 0;
    }
    draw_digit_at(origin, DIGIT_CORRESPONDANCE[number], scale);
}

