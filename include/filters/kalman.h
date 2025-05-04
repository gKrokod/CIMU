#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"

#define DELTA_T 0.04f

void print_matrix(const arm_matrix_instance_f32 *m, const char *name); 

void print_vector(const float32_t *vec, uint32_t size) ;


typedef struct {
    // Матрицы, которые не изменяются
    const arm_matrix_instance_f32 *F;  // Матрица динамики системы
    const arm_matrix_instance_f32 *B;  // Матрица ypravl
    const arm_matrix_instance_f32 *Q;  // Матрица noise provess
    const arm_matrix_instance_f32 *H;  // Матрица nablyde
    const arm_matrix_instance_f32 *R;  // Матрица ковариации шума измерения
    const arm_matrix_instance_f32 *I;  // Identity matrix
    // Матрицы, которые изменяются
    arm_matrix_instance_f32 *P;  // Covariaton 
    arm_matrix_instance_f32 *K;  // Kalman gain
    // Vectors, которые изменяются
    float32_t *vec_X;
    float32_t *vec_Z;
    float32_t *vec_U;
    
} KalmanFilter_t;

void kalman_init(KalmanFilter_t *kf, float32_t pitch, float32_t roll); 

void kalman_step(KalmanFilter_t *kf) ;
#endif

