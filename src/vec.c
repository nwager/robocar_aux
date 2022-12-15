#include "vec.h"

void vec_zero(float vec[3]) {
    for (int i = 0; i < 3; i++) {
        vec[i] = 0.0;
    }
}

void vec_add(float a[3], float b[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = a[i] + b[i];
    }
}

void vec_sub(float a[3], float b[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = a[i] - b[i];
    }
}

void vec_scalar_mul(float vec[3], float scalar, float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = vec[i] * scalar;
    }
}

void vec_mean(float vecs[][3], int vecs_size, float mean[3]) {
    vec_zero(mean);
    for (int i = 0; i < vecs_size; i++) {
        vec_add(mean, vecs[i], mean);
    }
    for (int i = 0; i < 3; i++) {
        mean[i] /= vecs_size;
    }
}

void vec_copy(float src[3], float dst[3]) {
    for (int i = 0; i < 3; i++) {
        dst[i] = src[i];
    }
}

float vec_mag(float vec[3]) {
    float sq_sum = 0.0;
    for (int i = 0; i < 3; i++) {
        sq_sum += vec[i] * vec[i];
    }
    return sqrtf(sq_sum);
}

#ifndef NO_PRINT

void vec_print(float v[3]) {
    printf("[ %f, %f, %f ]", v[0], v[1], v[2]);
}

void vec_println(float v[3]) {
    printf("[ %f, %f, %f ]\n", v[0], v[1], v[2]);
}

#endif // NO_PRINT