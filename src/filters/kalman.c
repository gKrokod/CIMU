#include "filters/kalman.h"
#include <math.h>

/* #define DELTA_T 0.04f */

/* typedef struct { */
/*     arm_matrix_instance_f32 F; // 4 >< 4  const       // State transition matrix */
/*     arm_matrix_instance_f32 H; // 2 >< 4  const       // Measurement matrix */
/*     arm_matrix_instance_f32 Q; // 4 >< 4  const       // Process noise covariance */
/*     arm_matrix_instance_f32 R; // 2 >< 2  const       // Measurement noise covariance */
/*     arm_matrix_instance_f32 B; // 4 >< 2  const       // Control matrix */
/*     arm_matrix_instance_f32 I; // 4 >< 4  const Identity */
/* } KalmanState; */
/*  */
/* typedef struct { */
/*     float32_t vec_X[4];      // x: [pitch, roll, bias_wE, bias_wN] */
/*     float32_t vec_Z[2];      // x: [measuredPitch, measuredRoll] */
/*     arm_matrix_instance_f32 P; // 4 >< 4 */
/*     arm_matrix_instance_f32 K; // 4 >< 2        // Kalman gain */
/*     KalmanState * state; */
/* } KalmanFilter; */
/*  */

// make const matrix
/* KalmanState MkKalmanState { */
/*  KalmanState ks; */
/*  return ks; */
/* } */


int Kalman_Init(KalmanFilter* kf, float32_t initialPitch, float32_t initialRoll){
  KalmanState * st;
  kf->state = st;
  return 0;
}

/*  */
/* static void matrix_init(arm_matrix_instance_f32* m,  */
/*                       uint16_t rows,  */
/*                       uint16_t cols,  */
/*                       float32_t* data) { */
/*     arm_mat_init_f32(m, rows, cols, data); */
/* } */
/*  */
/* static void mat_mult(const arm_matrix_instance_f32* A, */
/*                     const arm_matrix_instance_f32* B, */
/*                     arm_matrix_instance_f32* C) { */
/*     arm_mat_mult_f32(A, B, C); */
/* } */
/*  */
/* static void mat_trans(const arm_matrix_instance_f32* A, */
/*                      arm_matrix_instance_f32* AT) { */
/*     arm_mat_trans_f32(A, AT); */
/* } */
/*  */
/* static void mat_add(const arm_matrix_instance_f32* A, */
/*                    const arm_matrix_instance_f32* B, */
/*                    arm_matrix_instance_f32* C) { */
/*     arm_mat_add_f32(A, B, C); */
/* } */
/*  */
/* static void mat_sub(const arm_matrix_instance_f32* A, */
/*                    const arm_matrix_instance_f32* B, */
/*                    arm_matrix_instance_f32* C) { */
/*     arm_mat_sub_f32(A, B, C); */
/* } */
/*  */
/* static arm_status mat_inv(const arm_matrix_instance_f32* A, */
/*                          arm_matrix_instance_f32* invA) { */
/*     return arm_mat_inverse_f32(A, invA); */
/* } */
/*  */
/* void Kalman_Init(KalmanFilter* kf, float32_t initialPitch, float32_t initialRoll) { */
/*     #<{(| // Выделение памяти для всех матриц |)}># */
/*     #<{(|  |)}># */
/*     static float32_t F_data[STATE_DIM * STATE_DIM] = { */
/*         1.0f, 0.0f,  (-DELTA_T),   0.0f, */
/*         0.0f,  1.0f, 0.0f,   (-DELTA_T), */
/*         0.0f,  0.0f,  1.0f, 0.0f, */
/*         0.0f,  0.0f,  0.0f,   1.0f */
/*     }; */
/*     #<{(|  |)}># */
/*     static float32_t H_data[MEAS_DIM * STATE_DIM] = { */
/*         1.0f, 0.0f, 0.0f,  0.0f, */
/*         0.0f, 1.0f, 0.0f, 0.0f */
/*     }; */
/*     #<{(|  |)}># */
/*     static float32_t B_data[STATE_DIM * CTRL_DIM] = { // 4 <> 2 */
/*         0.0f, (DELTA_T), */
/*         (DELTA_T), 0.0f, */
/*         0.0f,  0.0f, */
/*         0.0f,  0.0f */
/*     }; */
/*     #<{(|  |)}># */
/*     static float32_t Q_data[STATE_DIM * STATE_DIM] = { */
/*         0.01f * DELTA_T, 0.0f,   0.0f,    0.0f, */
/*         0.0f,    0.01f * DELTA_T,0.0f,    0.0f, */
/*         0.0f,    0.0f,   0.0016f * DELTA_T,0.0f, */
/*         0.0f,    0.0f,   0.0f,    0.0016f * DELTA_T */
/*     }; */
/*     #<{(|  |)}># */
/*     static float32_t R_data[MEAS_DIM * MEAS_DIM] = { */
/*         0.0007f, 0.0f, */
/*         0.0f,    0.0009f */
/*     }; */
/*  */
/*     //todo разбираться тут */
/*     static float32_t K_data[STATE_DIM * MEAS_DIM] = {// 4 >< 2 */
/*         0.0f, 0.0f, 0.0f,  0.0f, */
/*         0.0f, 0.0f, 0.0f, 0.0f */
/*     }; */
/*  */
/*     static float32_t tmp41a_data[STATE_DIM]; */
/*     static float32_t tmp41b_data[STATE_DIM]; */
/*  */
/*     static float32_t tmp42a_data[STATE_DIM * MEAS_DIM]; */
/*     static float32_t tmp42b_data[STATE_DIM * MEAS_DIM]; */
/*     static float32_t tmp24a_data[STATE_DIM * MEAS_DIM]; */
/*     static float32_t tmp24b_data[STATE_DIM * MEAS_DIM]; */
/*  */
/*     static float32_t tmp21a_data[2]; */
/*     static float32_t tmp21b_data[2]; */
/*  */
/*     static float32_t tmp22a_data[MEAS_DIM * MEAS_DIM]; */
/*     static float32_t tmp22b_data[MEAS_DIM * MEAS_DIM]; */
/*  */
/*     static float32_t tmp44a_data[STATE_DIM * STATE_DIM]; */
/*     static float32_t tmp44b_data[STATE_DIM * STATE_DIM]; */
/*     static float32_t tmp44c_data[STATE_DIM * STATE_DIM]; */
/*     #<{(|  |)}># */
/*     #<{(| // Инициализация единичной матрицы |)}># */
/*     static float32_t identity_data[STATE_DIM * STATE_DIM] = { */
/*         1.0f, 0.0f, 0.0f, 0.0f, */
/*         0.0f, 1.0f, 0.0f, 0.0f, */
/*         0.0f, 0.0f, 1.0f, 0.0f, */
/*         0.0f, 0.0f, 0.0f, 1.0f */
/*     }; */
/*      */
/*     static float32_t state_data[STATE_DIM]; */
/*     state_data[0] = initialPitch * (float32_t)PI / 180.0f; */
/*     state_data[1] = initialRoll * (float32_t)PI / 180.0f; */
/*     state_data[2] = 0.0f; */
/*     state_data[3] = 0.0f; */
/*  */
/*     static float32_t P_data[STATE_DIM * STATE_DIM] = { */
/*         0.01f, 0.0f,  0.0f,   0.0f, */
/*         0.0f,  0.01f, 0.0f,   0.0f, */
/*         0.0f,  0.0f,  0.0016f,0.0f, */
/*         0.0f,  0.0f,  0.0f,   0.0016f */
/*     }; */
/*     // Инициализация матричных структур */
/*     matrix_init(&kf->state, STATE_DIM, 1, state_data); */
/*     matrix_init(&kf->covariance, STATE_DIM, STATE_DIM, P_data); */
/*     matrix_init(&kf->F, STATE_DIM, STATE_DIM, F_data); */
/*     matrix_init(&kf->H, MEAS_DIM, STATE_DIM, H_data); // 2 >< 4 */
/*     matrix_init(&kf->Q, STATE_DIM, STATE_DIM, Q_data); */
/*     matrix_init(&kf->R, MEAS_DIM, MEAS_DIM, R_data); */
/*     matrix_init(&kf->B, STATE_DIM, CTRL_DIM, B_data); // 4 <> 2 */
/*     matrix_init(&kf->K, STATE_DIM, MEAS_DIM, K_data); */
/*  */
/*     matrix_init(&kf->tmp41a, 4, 1, tmp41a_data); */
/*     matrix_init(&kf->tmp41b, 4, 1, tmp41b_data); */
/*  */
/*     matrix_init(&kf->tmp42a, 4, 2, tmp42a_data); */
/*     matrix_init(&kf->tmp42b, 4, 2, tmp42b_data); */
/*     matrix_init(&kf->tmp24a, 2, 4, tmp24a_data); */
/*     matrix_init(&kf->tmp24b, 2, 4, tmp24b_data); */
/*  */
/*     matrix_init(&kf->tmp22a, 2, 2, tmp22a_data); */
/*     matrix_init(&kf->tmp22b, 2, 2, tmp22b_data); */
/*  */
/*     matrix_init(&kf->tmp21a, 2, 1, tmp21a_data); */
/*     matrix_init(&kf->tmp21b, 2, 1, tmp21b_data); */
/*  */
/*     matrix_init(&kf->tmp44a, 4, 4, tmp44a_data); */
/*     matrix_init(&kf->tmp44b, 4, 4, tmp44b_data); */
/*     matrix_init(&kf->tmp44c, 4, 4, tmp44c_data); */
/*  */
/*     matrix_init(&kf->identity, STATE_DIM, STATE_DIM, identity_data); */
/* } */
/*  */
/* void Kalman_Predict(KalmanFilter* kf, float32_t dt, float32_t wN, float32_t wE) { */
/*  */
/*     arm_status status; */
/*  */
/*     // Прогноз состояния: x = F*x + B*u */
/*     float32_t u_data[CTRL_DIM] = { */
/*         wN * (float32_t)PI / 180.0f, */
/*         wE * (float32_t)PI / 180.0f */
/*     }; */
/*     arm_matrix_instance_f32 u; */
/*     matrix_init(&u, CTRL_DIM, 1, u_data); */
/*  */
/*     arm_mat_mult_f32(&kf->F, &kf->state, &kf->tmp41a);   // f #> x  [4><4] * [4><1] =  */
/*     arm_mat_mult_f32(&kf->B, &u, &kf->tmp41b);   // b #> u  [4><2] * [2><1] =  */
/*                                                  // */
/*     arm_mat_add_f32(&kf->tmp41a, &kf->tmp41b, &kf->state); // x_pred = fx + bu */
/*  */
/*     // Прогноз ковариации: P = F*P*F^T + Q// 4>< 4 * 4 >< 4 * 4 >< 4 T   + 4 >< 4 */
/*     arm_mat_mult_f32(&kf->F, &kf->covariance, &kf->tmp44a);   // F * P */
/*     arm_mat_trans_f32(&kf->F, &kf->tmp44b); // F ^ T */
/*     arm_mat_mult_f32(&kf->tmp44a, &kf->tmp44b, &kf->tmp44c); // F * P * F ^T */
/*     arm_mat_add_f32(&kf->tmp44c, &kf->Q, &kf->covariance); // P_pred = all + Q  */
/* } */
/*  */
/* void Kalman_Update(KalmanFilter* kf, float32_t measuredPitch, float32_t measuredRoll) { */
/*     // Преобразование измерений в радианы */
/*     float32_t z_data[MEAS_DIM] = { */
/*         measuredPitch * (float32_t)PI / 180.0f, */
/*         measuredRoll * (float32_t)PI / 180.0f */
/*     }; */
/*     arm_matrix_instance_f32 z; */
/*     matrix_init(&z, MEAS_DIM, 1, z_data); */
/*  */
/*     // Расчет невязки: y = z - H*x */
/*     arm_matrix_instance_f32 Hx; */
/*     float32_t Hx_data[MEAS_DIM]; */
/*     matrix_init(&Hx, MEAS_DIM, 1, Hx_data); */
/*      */
/*     arm_mat_mult_f32(&kf->H, &kf->state, &Hx); */
/*     arm_mat_sub_f32(&z, &Hx, &kf->tmp21a); // = y */
/*     #<{(|  |)}># */
/*     // Расчет ковариации невязки: S = H*P*H^T + R */
/*     arm_mat_mult_f32(&kf->H, &kf->covariance, &kf->tmp24a); // 2 ><4 * 4 >< 4 */
/*     arm_mat_trans_f32(&kf->H, &kf->tmp42a); // h ^ T */
/*     arm_mat_mult_f32(&kf->tmp24a, &kf->tmp42a, &kf->tmp22a); */
/*     #<{(|  |)}># */
/*     arm_mat_add_f32(&kf->tmp22a, &kf->R, &kf->tmp22b); // S = hph + r */
/*     #<{(|  |)}># */
/*     // Расчет коэффициента Калмана: K = P*H^T*S^-1    */
/*     arm_mat_mult_f32(&kf->covariance, &kf->tmp42a, &kf->tmp42b);// 4><4 * 4<>2  = 4 x 2  p * h ^ t */
/*     arm_mat_inverse_f32(&kf->tmp22b, &kf->tmp22a); // S -1 */
/*     #<{(|  |)}># */
/*     arm_mat_mult_f32(&kf->tmp42b, &kf->tmp22a, &kf->K); */
/*     #<{(|  |)}># */
/*     #<{(| // Обновление состояния: x = x + K*y |)}># */
/*     arm_mat_mult_f32(&kf->K, &kf->tmp21a, &kf->tmp41a); // 4><2 * 2 >< 1 =  4 >< 1 */
/*  */
/*     #<{(| arm_mat_add_f32(&kf->state, &kf->tmp41a, &kf->state); // todo. можно ли так складывать, ранее избегал |)}># */
/*     arm_mat_add_f32(&kf->state, &kf->tmp41a, &kf->tmp41b); // todo. можно ли так складывать, ранее избегал */
/*     #<{(| arm_copy_f32(&kf->tmp41b, &kf->state, STATE_DIM * 1 );  //так стало работать, убрал .pPoint |)}># */
/*     #<{(| arm_copy_f32(&kf->tmp41b.pData, &kf->state.pData, STATE_DIM * 1 );  //так стало работать, убрал .pPoint |)}># */
/*  */
/*     #<{(|  |)}># */
/*     // Обновление ковариации: P = (I - K*H)*P */
/*     arm_mat_mult_f32(&kf->K, &kf->H, &kf->tmp44a); // 4 >< 2 * 2 <> 4 = 4 <> 4    K * H */
/*     #<{(|  |)}># */
/*     // Используем предварительно созданную единичную матрицу из структуры */
/*     arm_mat_sub_f32(&kf->identity, &kf->tmp44a, &kf->tmp44b);// I - K * H */
/*     arm_mat_mult_f32(&kf->tmp44b, &kf->covariance, &kf->tmp44c); */
/*     #<{(|  |)}># */
/*     arm_copy_f32(kf->tmp44c.pData, kf->covariance.pData, STATE_DIM * STATE_DIM); */
/* } */
/*  */
/*  */
/*  */
/* void Kalman_GetAngles(KalmanFilter* kf, float32_t* pitch, float32_t* roll) { */
/*     *pitch = kf->state.pData[0] * 180.0f / PI; // Convert to degrees */
/*     *roll = kf->state.pData[1] * 180.0f / PI; */
/* } */
/*  */
