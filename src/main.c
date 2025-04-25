#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"

// Преобразование радиан в градусы
#define RAD_TO_DEG (180.0f / PI)
#define DEG_TO_RAD (PI / 180.0f)


typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} Mag;

typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} Gyro;

typedef struct {
    float32_t pitch;
    float32_t roll;
} Angles;

typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} Acceleration;

Acceleration convertToAcceleration(const DataEntry* entry) {
    Acceleration acc;
    acc.x = entry->acc_x;
    acc.y = entry->acc_y;
    acc.z = entry->acc_z;
    return acc;
}
Mag convertToMag(const DataEntry* entry) {
    Mag mag;
    mag.x = entry->mag_x;
    mag.y = entry->mag_y;
    mag.z = entry->mag_z;
    return mag;
}
Gyro convertToGyro(const DataEntry* entry) {
    Gyro gyro;
    gyro.x = entry->gyro_x;
    gyro.y = entry->gyro_y;
    gyro.z = entry->gyro_z;
    return gyro;
}
Angles convertToAngles(const DataEntry* entry) {
    Angles angles;
    angles.pitch = entry->pitch_sensor;
    angles.roll = entry->roll_sensor;
    return angles;
}

void printStruct(const Acceleration* sensor){
  printf("Triad: (%.2f, %.2f, %.2f)\n",
   sensor->x,
   sensor->y,
   sensor->z);
}
void printAngles(const Angles* sensor){
  printf("Angles: (%.2f, %.2f)\n",
   sensor->pitch,
   sensor->roll);
}

float32_t calculateRoll(const Acceleration* acc) { // DEG
    const float32_t n = acc->y;
    const float32_t d = acc->z;
    return (atan2f(n, d) * RAD_TO_DEG);
}

float32_t calculatePitch(const Acceleration* acc) { // DEG

    const float32_t n = acc->x;
    const float32_t vec[2] = {acc->y, acc->z};
    float32_t sum_squares;
    arm_dot_prod_f32(&vec, &vec, 2, &sum_squares);
    float32_t d;
    arm_sqrt_f32(sum_squares, &d);
    return -(atan2f(n, d) * RAD_TO_DEG);
}

Angles calculateAngles(const Acceleration* acc) { //DEG
  Angles angles;
  angles.pitch = calculatePitch(acc);
  angles.roll = calculateRoll(acc);
  return angles;
}

float32_t calculateAzimuth(
    const float32_t pitch_deg, 
    const float32_t roll_deg, 
    const Mag* mag
) {
  
    const float32_t sin_pitch, cos_pitch;
    arm_sin_cos_f32(pitch_deg, &sin_pitch, &cos_pitch);

    const float32_t sin_roll, cos_roll;
    arm_sin_cos_f32(roll_deg, &sin_roll, &cos_roll);

    const float32_t n = 
        -(mag->y * cos_roll - mag->z * sin_roll);
    
    const float32_t d = 
        (mag->y * sin_roll + mag->z * cos_roll) * sin_pitch + 
        mag->x * cos_pitch;
    
    // Вычисление азимута с корректировкой квадрантов
    float32_t azimuth = atan2f(n, d);
    if (n >= 0 && d >= 0) {azimuth += 0;} ;
    if (n >= 0 && d < 0) {azimuth += PI;} ;
    if (n < 0 && d >= 0) {azimuth +=  2 * PI;} ;
    if (n < 0 && d < 0) {azimuth += PI;} ;
    //
    // Преобразование в градусы
    return azimuth * RAD_TO_DEG;
}




int main() {
    // Инициализация коллекции данных
    DataCollection dataCollection;
    initDataCollection(&dataCollection, 10); // Начальная емкость 10

    // Чтение данных из файла
    int readResult = readDataFile(&dataCollection);
    if (readResult != 0) {
        printf("Ошибка чтения файла: %d\n", readResult);
        freeDataCollection(&dataCollection);
        return 1;
    }
    // Вывод первых трех строк
    printf("Первые три строки из файла:\n");
    printFirstNRows(&dataCollection, 3);

    for (int i = 0; i < 3; i++) {
  
      Mag mm = convertToMag(&dataCollection.entries[i]);
      Acceleration aa = convertToAcceleration(&dataCollection.entries[i]);
      Angles pitchRoll = calculateAngles(&aa);
      float az = calculateAzimuth(pitchRoll.pitch, pitchRoll.roll, &mm);
      /* Gyro gg = convertToGyro(&dataCollection.entries[i]); */
      /* Angles pitchRoll = convertToAngles(&dataCollection.entries[i]); */
      /* printStruct(&mm); */
      /* printStruct(&aa); */
      /* printStruct(&gg); */
      printAngles(&pitchRoll);
      printf("Az: %.2f\n",az);
      /* printf("\nEND\n"); */
}

    /* printf("All collection:\n"); */
    /* printInputData(&dataCollection); */

    // Обработка данных (здесь будет математическая обработка)
    /* ProcessedData results; */
    /* int processResult = processData(&dataCollection, &results); */
    /* if (processResult != 0) { */
    /*     printf("Ошибка обработки данных: %d\n", processResult); */
    /*     freeDataCollection(&dataCollection); */
    /*     return 2; */
    /* } */
    /*  */
    /* // Вывод результатов обработки */
    /* printf("\nРезультаты обработки (первые 3):\n"); */
    /* printProcessedData(&results, 3); */
    /*  */
    /* // Освобождение памяти */
    /* freeProcessedData(&results); */
    /* freeDataCollection(&dataCollection); */
    //
    // Инициализация матриц
    float32_t A_data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};  // Матрица 2x3
    float32_t B_data[6] = {7.0, 8.0, 9.0, 10.0, 11.0, 12.0};  // Матрица 3x2
    float32_t C_data[4];  // Результат матричного умножения 2x2


    float32_t rad = 1.2; 
    float32_t sin_pitch = sinf(rad);
    float32_t cos_pitch = cosf(rad);
    printf("%.2f   %.2f \n", sin_pitch, cos_pitch);
    arm_sin_cos_f32(RAD_TO_DEG * rad, &sin_pitch, &cos_pitch);
    printf("%.2f   %.2f \n", sin_pitch, cos_pitch);
  

    /* const float32_t sin_roll, cos_roll; */
    /* arm_sin_cos_f32(roll_rad, &sin_roll, &cos_roll); */

    return 0;
}

/* int main() { */
/*     // Инициализация матриц */
/*     float32_t A_data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};  // Матрица 2x3 */
/*     float32_t B_data[6] = {7.0, 8.0, 9.0, 10.0, 11.0, 12.0};  // Матрица 3x2 */
/*     float32_t C_data[4];  // Результат матричного умножения 2x2 */
/*  */
/*     arm_matrix_instance_f32 A; */
/*     arm_matrix_instance_f32 B; */
/*     arm_matrix_instance_f32 C; */
/*  */
/*     // Инициализация структур матриц */
/*     arm_mat_init_f32(&A, 2, 3, A_data); */
/*     arm_mat_init_f32(&B, 3, 2, B_data); */
/*     arm_mat_init_f32(&C, 2, 2, C_data); */
/*  */
/*     // Умножение матриц A * B = C */
/*     arm_status status = arm_mat_mult_f32(&A, &B, &C); */
/*  */
/*     // Проверка статуса и вывод результата */
/*     if (status == ARM_MATH_SUCCESS) { */
/*         printf("Результат умножения матриц:\n"); */
/*         for (int i = 0; i < 2; i++) { */
/*             for (int j = 0; j < 2; j++) { */
/*                 printf("%f ", C.pData[i * 2 + j]); */
/*             } */
/*             printf("\n"); */
/*         } */
/*     } else { */
/*         printf("Ошибка при умножении матриц\n"); */
/*     } */
/*  */
/*     return 0; */
/* } */
