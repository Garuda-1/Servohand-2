//
// Created by oktet on 16.06.2020.
//

#ifndef ESP32_VECTOR_3D_H
#define ESP32_VECTOR_3D_H

typedef struct vector_3d {
    double x, y, z;
} vector_3d;

double vector_len(vector_3d*);

double vector_sc_prod(vector_3d*, vector_3d*);

double vector_angle(vector_3d*, vector_3d*);

double vector_phi(vector_3d*);

double vector_theta(vector_3d*);

double rad_to_deg(double r);

#endif //ESP32_VECTOR_3D_H
