#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"

typedef struct {
    arm_matrix_instance_f32 F; // 4 >< 4  const       // State transition matrix
    arm_matrix_instance_f32 H; // 2 >< 4  const       // Measurement matrix
    arm_matrix_instance_f32 Q; // 4 >< 4  const       // Process noise covariance
    arm_matrix_instance_f32 R; // 2 >< 2  const       // Measurement noise covariance
    arm_matrix_instance_f32 B; // 4 >< 2  const       // Control matrix
    arm_matrix_instance_f32 I; // 4 >< 4  const Identity
} KalmanState;

typedef struct {
    float32_t vec_X[4];      // x: [pitch, roll, bias_wE, bias_wN]
    float32_t vec_Z[2];      // x: [measuredPitch, measuredRoll]
    arm_matrix_instance_f32 P; // 4 >< 4
    arm_matrix_instance_f32 K; // 4 >< 2        // Kalman gain
    KalmanState * state;
} KalmanFilter;


int Kalman_Init(KalmanFilter* kf, float32_t initialPitch, float32_t initialRoll);
// void Kalman_Predict(KalmanFilter* kf, float32_t dt, float32_t wN, float32_t wE);
// void Kalman_Update(KalmanFilter* kf, float32_t measuredPitch, float32_t measuredRoll);
// void Kalman_GetAngles(KalmanFilter* kf, float32_t* pitch, float32_t* roll);

#endif

