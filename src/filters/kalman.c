#include "filters/kalman.h"
#include "arm_math.h"
#include <stdio.h>

// Объявление массива для матрицы F размером 4x4
static const float32_t F_data[16] = {
      1.0f, 0.0f,  (-DELTA_T),   0.0f,
      0.0f,  1.0f, 0.0f,   (-DELTA_T),
      0.0f,  0.0f,  1.0f, 0.0f,
      0.0f,  0.0f,  0.0f,   1.0f
    };

// Объявление массива для матрицы R размером 2x2
static const float32_t R_data[4] = {
        0.0007f, 0.0f,
        0.0f,    0.0009f
};

// Объявление массива для матрицы H размером 2x4
static const float32_t H_data[8] = {
    1.0f, 0.0f, 0.0f,  0.0f,
    0.0f, 1.0f, 0.0f, 0.0f
};

// Объявление массива для матрицы B размером 4x2
static float32_t B_data[8] = { // 4 <> 2
    0.0f, (DELTA_T),
    (DELTA_T), 0.0f,
    0.0f,  0.0f,
    0.0f,  0.0f
};

// Объявление массива для матрицы Q размером 4x4
static float32_t Q_data[16] = {
    0.01f * DELTA_T, 0.0f,   0.0f,    0.0f,
    0.0f,    0.01f * DELTA_T,0.0f,    0.0f,
    0.0f,    0.0f,   0.0016f * DELTA_T,0.0f,
    0.0f,    0.0f,   0.0f,    0.0016f * DELTA_T
}; 
// Объявление массива для матрицы I размером 4x4
static float32_t I_data[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
};
// Объявление массива для матрицы P размером 4x4
static float32_t P_data[16] = { 
    0.01f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.01f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0016f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0016f
};
// Объявление массива для матрицы K размером 4x2
static float32_t K_data[8] = { // 4 <> 2
    0.0f, 0.0f,
    0.0f, 0.0f,
    0.0f, 0.0f,
    0.0f, 0.0f
};
static float32_t X_data[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [pitch, roll, bias_wE, bias_wN] [rad, rad, rad/s, ras/s]
static float32_t Z_data[2] = {0.0f, 0.0f}; // [measuredPitch, measuredRoll] [rad, rad] 
static float32_t U_data[2] = {0.0f, 0.0f}; // [gyro WN, gyro WE] [rad/s, rad/s] 
// Объявление матрицы F размером 4x4
static const arm_matrix_instance_f32 F = {4, 4, (float32_t *)F_data};
// Объявление матрицы R размером 2x2
static const arm_matrix_instance_f32 R = {2, 2, (float32_t *)R_data};
// Объявление матрицы H размером 2x4
static const arm_matrix_instance_f32 H = {2, 4, (float32_t *)H_data};
// Объявление матрицы B размером 4x2
static const arm_matrix_instance_f32 B = {4, 2, (float32_t *)B_data};
// Объявление матрицы Q размером 4x4
static const arm_matrix_instance_f32 Q = {4, 4, (float32_t *)Q_data};
// Объявление матрицы I размером 4x4
static const arm_matrix_instance_f32 I = {4, 4, (float32_t *)I_data};
// Объявление матрицы P размером 4x4
static arm_matrix_instance_f32 P = {4, 4, (float32_t *)P_data};
// Объявление матрицы K размером 4x2
static arm_matrix_instance_f32 K = {4, 2, (float32_t *)K_data};

void print_matrix(const arm_matrix_instance_f32 *m, const char *name) {
    printf("Matrix %s (%dx%d):\n", name, m->numRows, m->numCols);
    for (uint16_t i = 0; i < m->numRows; ++i) {
        for (uint16_t j = 0; j < m->numCols; ++j) {
            printf("%10.10f ", m->pData[i * m->numCols + j]);
        }
        printf("\n");
    }
    printf("\n");
}

void print_vector(const float32_t *vec, uint32_t size) {
    printf("Print vector: ");
    for(uint32_t i = 0; i < size; ++i) {
        printf("%10.6f ", vec[i]);
    }
    printf("\n");
}

void kalman_init(KalmanFilter_t *kf, float32_t pitch, float32_t roll) {
    // Присваиваем указатели на статические матрицы
    kf->F = &F;
    kf->B = &B;
    kf->Q = &Q;
    kf->H = &H;
    kf->R = &R;
    kf->I = &I;
    kf->P = &P;
    kf->K = &K;
    X_data[0] = pitch;//rad
    X_data[1] = roll;//rad
    X_data[2] = 0;//rad/s
    X_data[3] = 0;//rad/s
    kf->vec_X = X_data;
    kf->vec_Z = Z_data;
    kf->vec_U = U_data;
}


void kalman_step(KalmanFilter_t *kf, Angles *pitchRoll, Gyro *gyro) {

        float32_t sin_tettha, cos_tettha;
        arm_sin_cos_f32(RAD_TO_DEG * kf->vec_X[0], &sin_tettha, &cos_tettha);
        float32_t sin_phi, cos_phi;
        arm_sin_cos_f32(RAD_TO_DEG * kf->vec_X[1], &sin_phi, &cos_phi);

        // -- Control from gyro
        kf->vec_U[0] = DEG_TO_RAD * (gyro->x * cos_tettha + sin_tettha * (gyro->z * cos_phi + gyro->y * sin_phi)); 
        kf->vec_U[1] = DEG_TO_RAD * (gyro->y * cos_phi - gyro->z * sin_phi);

        /* z = vector [measuredPitch, measuredRoll] -- тут надо в радианах давать */
        kf->vec_Z[0] = DEG_TO_RAD * pitchRoll->pitch;
        kf->vec_Z[1] = DEG_TO_RAD * pitchRoll->roll;

        // -- Шаг предсказания DONE
        /* x_pred = f #> x + b #> u */
        float32_t fx[4] = {0};
        arm_mat_vec_mult_f32(kf->F, kf->vec_X, fx);
        float32_t bu[4] = {0};
        arm_mat_vec_mult_f32(kf->B, kf->vec_U, bu);
        float32_t xpred[4] = {0};
        arm_add_f32(fx, bu, xpred, 4);
        /* p_pred = f <> p <> tr f + q */
        // F_T
        float32_t tempFT[16] = {0};
        arm_matrix_instance_f32 F_T = {4, 4, tempFT};
        arm_mat_trans_f32(kf->F,&F_T);
        // F <> P
        float32_t tempFP[16] = {0};
        arm_matrix_instance_f32 F_P = {4, 4, tempFP};
        arm_mat_mult_f32(kf->F, kf->P, &F_P);
        // F <> P <> F_T
        float32_t tempFPFT[16] = {0};
        arm_matrix_instance_f32 F_P_FT = {4, 4, tempFPFT};
        arm_mat_mult_f32(&F_P, &F_T, &F_P_FT);
        // p_pred = f <> p <> tr f + q 
        float32_t tempPpred[16] = {0};
        arm_matrix_instance_f32 P_pred = {4, 4, tempPpred};
        arm_mat_add_f32(&F_P_FT, kf->Q, &P_pred);
        //
        /* -- Шаг обновления */
        /* y = z - (h #> x_pred)  -- Невязка */
        float32_t hx[2] = {0};
        arm_mat_vec_mult_f32(kf->H, xpred, hx);
        float32_t vec_Y[2] = {0};
        arm_sub_f32(kf->vec_Z, hx, vec_Y, 2);
        /* s = (h <> p_pred <> tr h) + r'  -- Ковариация невязки */
        // H_T
        float32_t tempHT[8] = {0};
        arm_matrix_instance_f32 H_T = {4, 2, tempHT};
        arm_mat_trans_f32(kf->H,&H_T);
        // H <> P_pred
        float32_t tempHPpred[8] = {0};
        arm_matrix_instance_f32 H_Ppred = {2, 4, tempHPpred};
        arm_mat_mult_f32(kf->H, &P_pred, &H_Ppred);
        // H <> P_pred <> H_T
        float32_t tempHPpHT[4] = {0};
        arm_matrix_instance_f32 H_Ppred_HT = {2, 2, tempHPpHT};
        arm_mat_mult_f32(&H_Ppred, &H_T, &H_Ppred_HT);
        // s = (h <> p_pred <> tr h) + r'  -- Ковариация невязки 
        float32_t tempS[4] = {0};
        arm_matrix_instance_f32 S = {2, 2, tempS};
        arm_mat_add_f32(&H_Ppred_HT, kf->R, &S);
        /* k = p_pred <> tr h <> inv s  -- Коэффициент Калмана */
        // inv_S
        float32_t tempIS[4] = {0};
        arm_matrix_instance_f32 I_S = {2, 2, tempIS};
        arm_mat_inverse_f32(&S,&I_S);
        // P_pred <> tr h
        float32_t tempPpredHT[8] = {0};
        arm_matrix_instance_f32 Ppred_HT = {4, 2, tempPpredHT};
        arm_mat_mult_f32(&P_pred, &H_T, &Ppred_HT);
        // k = p_pred <> tr h <> inv s
        arm_mat_mult_f32(&Ppred_HT, &I_S, kf->K);
        /* x_upd = x_pred + k #> y  -- Обновленная оценка состояния */
        float32_t ky[4] = {0};

        arm_mat_vec_mult_f32(kf->K, vec_Y, ky);
        arm_add_f32(xpred, ky, kf->vec_X, 4);

        /* i_kh = ident 4 - (k <> h)  -- I - KH */
        float32_t tempKH[16] = {0};
        arm_matrix_instance_f32 K_H = {4, 4, tempKH};
        arm_mat_mult_f32(kf->K, kf->H, &K_H);
        float32_t tempIKH[16] = {0};
        arm_matrix_instance_f32 I_K_H = {4, 4, tempIKH};
        arm_mat_sub_f32(kf->I, &K_H, &I_K_H);
        /* p_upd = i_kh <> p_pred  -- Обновленная ковариационная матрица */
        arm_mat_mult_f32(&I_K_H, &P_pred, kf->P);
} 
