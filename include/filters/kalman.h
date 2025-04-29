#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"

#define STATE_DIM 4
#define MEAS_DIM 2
#define CTRL_DIM 2

typedef struct {
    arm_matrix_instance_f32 state;      // x: [pitch, roll, bias_wE, bias_wN]
    arm_matrix_instance_f32 covariance; // P
    arm_matrix_instance_f32 F;          // State transition matrix
    arm_matrix_instance_f32 H;          // Measurement matrix
    arm_matrix_instance_f32 Q;          // Process noise covariance
    arm_matrix_instance_f32 R;          // Measurement noise covariance
    arm_matrix_instance_f32 B;          // Control matrix
    arm_matrix_instance_f32 K;          // Kalman gain
    arm_matrix_instance_f32 tmp1;       // Временные матрицы для расчетов
    arm_matrix_instance_f32 tmp2;
    arm_matrix_instance_f32 tmp3;
    arm_matrix_instance_f32 tmp4;
    arm_matrix_instance_f32 identity; // I
} KalmanFilter;

void Kalman_Init(KalmanFilter* kf, float32_t initialPitch, float32_t initialRoll);
void Kalman_Predict(KalmanFilter* kf, float32_t dt, float32_t wN, float32_t wE);
void Kalman_Update(KalmanFilter* kf, float32_t measuredPitch, float32_t measuredRoll);
void Kalman_GetAngles(KalmanFilter* kf, float32_t* pitch, float32_t* roll);

#endif

