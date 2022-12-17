#include "vec3.h"

void vec_zero(vec3_t *vec) {
    for (int i = 0; i < 3; i++) {
        vec->v[i] = 0.0;
    }
}

void vec_add(vec3_t *a, vec3_t *b, vec3_t *out) {
    for (int i = 0; i < 3; i++) {
        out->v[i] = a->v[i] + b->v[i];
    }
}

void vec_sub(vec3_t *a, vec3_t *b, vec3_t *out) {
    for (int i = 0; i < 3; i++) {
        out->v[i] = a->v[i] - b->v[i];
    }
}

void vec_scalar_mul(vec3_t *vec, float scalar, vec3_t *out) {
    for (int i = 0; i < 3; i++) {
        out->v[i] = vec->v[i] * scalar;
    }
}

void vec_mean(vec3_t vecs[], int vecs_size, vec3_t *mean) {
    vec_zero(mean);
    for (int i = 0; i < vecs_size; i++) {
        vec_add(mean, vecs + i, mean);
    }
    for (int i = 0; i < 3; i++) {
        mean->v[i] /= vecs_size;
    }
}

void vec_copy(vec3_t *src, vec3_t *dst) {
    for (int i = 0; i < 3; i++) {
        dst->v[i] = src->v[i];
    }
}

float vec_mag(vec3_t *vec) {
    float sq_sum = 0.0;
    for (int i = 0; i < 3; i++) {
        sq_sum += vec->v[i] * vec->v[i];
    }
    return sqrtf(sq_sum);
}

float vec_dot(vec3_t *a, vec3_t *b) {
    float sum = 0.0;
    for (int i = 0; i < 3; i++) {
        sum += a->v[i] * b->v[i];
    }
    return sum;
}

void vec_project(vec3_t *a, vec3_t *b, vec3_t *out) {
    vec_scalar_mul(b, vec_dot(a, b) / vec_dot(b, b), out);
}

void vec_project_unit(vec3_t *v, vec3_t *unit, vec3_t *out) {
    vec_scalar_mul(unit, vec_dot(v, unit), out);
}

#ifndef NO_PRINT

void vec_print(vec3_t *v) {
    printf("[ %f, %f, %f ]", v->v[0], v->v[1], v->v[2]);
}

void vec_println(vec3_t *v) {
    printf("[ %f, %f, %f ]\n", v->v[0], v->v[1], v->v[2]);
}

#endif // NO_PRINT