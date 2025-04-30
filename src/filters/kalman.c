#include "filters/kalman.h"
#include <math.h>

#define DELTA_T 0.04f

static void matrix_init(arm_matrix_instance_f32* m, 
                      uint16_t rows, 
                      uint16_t cols, 
                      float32_t* data) {
    arm_mat_init_f32(m, rows, cols, data);
}

static void mat_mult(const arm_matrix_instance_f32* A,
                    const arm_matrix_instance_f32* B,
                    arm_matrix_instance_f32* C) {
    arm_mat_mult_f32(A, B, C);
}

static void mat_trans(const arm_matrix_instance_f32* A,
                     arm_matrix_instance_f32* AT) {
    arm_mat_trans_f32(A, AT);
}

static void mat_add(const arm_matrix_instance_f32* A,
                   const arm_matrix_instance_f32* B,
                   arm_matrix_instance_f32* C) {
    arm_mat_add_f32(A, B, C);
}

static void mat_sub(const arm_matrix_instance_f32* A,
                   const arm_matrix_instance_f32* B,
                   arm_matrix_instance_f32* C) {
    arm_mat_sub_f32(A, B, C);
}

static arm_status mat_inv(const arm_matrix_instance_f32* A,
                         arm_matrix_instance_f32* invA) {
    return arm_mat_inverse_f32(A, invA);
}

void Kalman_Init(KalmanFilter* kf, float32_t initialPitch, float32_t initialRoll) {
    /* // Выделение памяти для всех матриц */
    /*  */
    static float32_t F_data[STATE_DIM * STATE_DIM] = {
        1.0f, 0.0f,  (-DELTA_T),   0.0f,
        0.0f,  1.0f, 0.0f,   (-DELTA_T),
        0.0f,  0.0f,  1.0f,0.0f,
        0.0f,  0.0f,  0.0f,   1.0f
    };
    /*  */
    static float32_t H_data[MEAS_DIM * STATE_DIM] = {
        1.0f, 0.0f, 0.0f,  0.0f,
        0.0f, 1.0f, 0.0f, 0.0f
    };
    /*  */
    static float32_t B_data[STATE_DIM * CTRL_DIM] = {
        0.0f, (DELTA_T),
        (DELTA_T), 0.0f,
        0.0f,  0.0f,
        0.0f,  0.0f
    };
    /*  */
    static float32_t Q_data[STATE_DIM * STATE_DIM] = {
        0.01f * DELTA_T, 0.0f,   0.0f,    0.0f,
        0.0f,    0.01f * DELTA_T,0.0f,    0.0f,
        0.0f,    0.0f,   0.0016f * DELTA_T,0.0f,
        0.0f,    0.0f,   0.0f,    0.0016f * DELTA_T
    };
    /*  */
    static float32_t R_data[MEAS_DIM * MEAS_DIM] = {
        0.0007f, 0.0f,
        0.0f,    0.0009f
    };

    //todo разбираться тут
    static float32_t K_data[STATE_DIM * MEAS_DIM] = {0}; // 4 >< 2
    static float32_t tmp1_data[STATE_DIM * STATE_DIM];
    static float32_t tmp2_data[STATE_DIM * MEAS_DIM];
    static float32_t tmp3_data[MEAS_DIM * MEAS_DIM];
    static float32_t tmp4_data[MEAS_DIM * MEAS_DIM];
    /*  */
    /* // Инициализация единичной матрицы */
    static float32_t identity_data[STATE_DIM * STATE_DIM] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    
    static float32_t state_data[STATE_DIM];
    state_data[0] = initialPitch * (float32_t)M_PI / 180.0f;
    state_data[1] = initialRoll * (float32_t)M_PI / 180.0f;
    state_data[2] = 0.0f;
    state_data[3] = 0.0f;

    static float32_t P_data[STATE_DIM * STATE_DIM] = {
        0.01f, 0.0f,  0.0f,   0.0f,
        0.0f,  0.01f, 0.0f,   0.0f,
        0.0f,  0.0f,  0.0016f,0.0f,
        0.0f,  0.0f,  0.0f,   0.0016f
    };
    // Инициализация матричных структур
    matrix_init(&kf->state, STATE_DIM, 1, state_data);
    matrix_init(&kf->covariance, STATE_DIM, STATE_DIM, P_data);
    matrix_init(&kf->F, STATE_DIM, STATE_DIM, F_data);
    matrix_init(&kf->H, MEAS_DIM, STATE_DIM, H_data);
    matrix_init(&kf->Q, STATE_DIM, STATE_DIM, Q_data);
    matrix_init(&kf->R, MEAS_DIM, MEAS_DIM, R_data);
    matrix_init(&kf->B, STATE_DIM, CTRL_DIM, B_data);
    matrix_init(&kf->K, STATE_DIM, MEAS_DIM, K_data);
    matrix_init(&kf->tmp1, STATE_DIM, STATE_DIM, tmp1_data);
    matrix_init(&kf->tmp2, STATE_DIM, MEAS_DIM, tmp2_data);
    matrix_init(&kf->tmp3, MEAS_DIM, MEAS_DIM, tmp3_data);
    matrix_init(&kf->tmp4, MEAS_DIM, MEAS_DIM, tmp4_data);
    matrix_init(&kf->identity, STATE_DIM, STATE_DIM, identity_data);
}

void Kalman_Predict(KalmanFilter* kf, float32_t dt, float32_t wN, float32_t wE) {
    // Обновление матрицы перехода F
    float32_t F_data[STATE_DIM * STATE_DIM] = {
        1.0f, 0.0f, -dt, 0.0f,
        0.0f, 1.0f, 0.0f, -dt,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    arm_copy_f32(F_data, kf->F.pData, STATE_DIM * STATE_DIM);

    // Обновление управляющей матрицы B
    float32_t B_data[STATE_DIM * CTRL_DIM] = {
        0.0f, dt,
        dt, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f
    };
    arm_copy_f32(B_data, kf->B.pData, STATE_DIM * CTRL_DIM);

    // Прогноз состояния: x = F*x + B*u
    arm_matrix_instance_f32 u;
    float32_t u_data[CTRL_DIM] = {wN, wE};
    matrix_init(&u, CTRL_DIM, 1, u_data);
    
    arm_mat_mult_f32(&kf->F, &kf->state, &kf->tmp1);
    arm_mat_mult_f32(&kf->B, &u, &kf->tmp2);
    arm_mat_add_f32(&kf->tmp1, &kf->tmp2, &kf->state);

    // Прогноз ковариации: P = F*P*F^T + Q
    arm_mat_mult_f32(&kf->F, &kf->covariance, &kf->tmp1);
    arm_mat_trans_f32(&kf->F, &kf->tmp2);
    arm_mat_mult_f32(&kf->tmp1, &kf->tmp2, &kf->tmp3);
    arm_mat_add_f32(&kf->tmp3, &kf->Q, &kf->covariance);
}

void Kalman_Update(KalmanFilter* kf, float32_t measuredPitch, float32_t measuredRoll) {
    // Преобразование измерений в радианы
    float32_t z_data[MEAS_DIM] = {
        measuredPitch * (float32_t)M_PI / 180.0f,
        measuredRoll * (float32_t)M_PI / 180.0f
    };
    arm_matrix_instance_f32 z;
    matrix_init(&z, MEAS_DIM, 1, z_data);

    // Матрица измерений H 
    float32_t H_data[MEAS_DIM * STATE_DIM] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f
    };
    arm_copy_f32(H_data, kf->H.pData, MEAS_DIM * STATE_DIM);

    // Расчет невязки: y = z - H*x
    arm_matrix_instance_f32 Hx;
    float32_t Hx_data[MEAS_DIM];
    matrix_init(&Hx, MEAS_DIM, 1, Hx_data);
    
    arm_mat_mult_f32(&kf->H, &kf->state, &Hx);
    arm_mat_sub_f32(&z, &Hx, &kf->tmp2);

    // Расчет ковариации невязки: S = H*P*H^T + R
    arm_mat_mult_f32(&kf->H, &kf->covariance, &kf->tmp1);
    arm_mat_trans_f32(&kf->H, &kf->tmp3);
    arm_mat_mult_f32(&kf->tmp1, &kf->tmp3, &kf->tmp4);
    arm_mat_add_f32(&kf->tmp4, &kf->R, &kf->tmp3);

    // Расчет коэффициента Калмана: K = P*H^T*S^-1
    arm_mat_trans_f32(&kf->H, &kf->tmp4);
    arm_mat_mult_f32(&kf->covariance, &kf->tmp4, &kf->tmp1);
    
    // Добавляем проверку ошибок инверсии
    arm_status inv_status = mat_inv(&kf->tmp3, &kf->tmp4);
    if(inv_status != ARM_MATH_SUCCESS) {
        // Обработка ошибки: матрица S вырождена
        return;
    }
    
    arm_mat_mult_f32(&kf->tmp1, &kf->tmp4, &kf->K);

    // Обновление состояния: x = x + K*y
    arm_mat_mult_f32(&kf->K, &kf->tmp2, &kf->tmp1);
    arm_mat_add_f32(&kf->state, &kf->tmp1, &kf->state);

    // Обновление ковариации: P = (I - K*H)*P
    arm_mat_mult_f32(&kf->K, &kf->H, &kf->tmp1);
    
    // Используем предварительно созданную единичную матрицу из структуры
    arm_mat_sub_f32(&kf->identity, &kf->tmp1, &kf->tmp2);
    arm_mat_mult_f32(&kf->tmp2, &kf->covariance, &kf->tmp1);
    arm_copy_f32(kf->tmp1.pData, kf->covariance.pData, STATE_DIM * STATE_DIM);
}



void Kalman_GetAngles(KalmanFilter* kf, float32_t* pitch, float32_t* roll) {
    *pitch = kf->state.pData[0] * 180.0f / M_PI; // Convert to degrees
    *roll = kf->state.pData[1] * 180.0f / M_PI;
}

