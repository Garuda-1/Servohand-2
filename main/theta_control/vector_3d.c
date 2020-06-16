#include "vector_3d.h"
#include <math.h>

double vector_len(vector_3d* v) {
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double vector_sc_prod(vector_3d* v1, vector_3d* v2) {
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

double vector_angle(vector_3d* v1, vector_3d* v2) {
    return acos(vector_sc_prod(v1, v2) / (vector_len(v1) * vector_len(v2)));
}

double vector_phi(vector_3d* v) {
    vector_3d i = {vector_len(v), 0, 0};
    return vector_angle(v, &i);
}

double vector_theta(vector_3d* v) {
    vector_3d i = {v->x, v->y, 0};
    return vector_angle(v, &i);
}

double rad_to_deg(double r) {
    return r / M_PI * 180;
}