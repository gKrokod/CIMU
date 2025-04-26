#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"
#include "angles.h"

#define DELTA_T = 0.04f
#define ALPHA_LOW_PASS = 0.081f //10 Hz
#define OUTPUT_FILE "Data/output.txt"

DataEntry averageFirstNEntries(const DataCollection* collection, const uint32_t n) {
    DataEntry avgEntry = {0};
    
    if(collection->count < n || n < 1) {
        printf("Недостаточно данных для усреднения\n");
        return avgEntry;
    }

    // Инициализация временных буферов
    float32_t magX[n], magY[n], magZ[n], time[n];
    float32_t accX[n], accY[n], accZ[n];
    float32_t gyroX[n], gyroY[n], gyroZ[n];
    
    // Заполнение буферов
    for(int i=0; i<n; i++) {
        time[i] = collection->entries[i].time;

        magX[i] = collection->entries[i].mag_x;
        magY[i] = collection->entries[i].mag_y;
        magZ[i] = collection->entries[i].mag_z;
        
        accX[i] = collection->entries[i].acc_x;
        accY[i] = collection->entries[i].acc_y;
        accZ[i] = collection->entries[i].acc_z;
        
        gyroX[i] = collection->entries[i].gyro_x;
        gyroY[i] = collection->entries[i].gyro_y;
        gyroZ[i] = collection->entries[i].gyro_z;
    }

    // Вычисление средних значений 
    arm_mean_f32(time, n, &avgEntry.time);

    arm_mean_f32(magX, n, &avgEntry.mag_x);
    arm_mean_f32(magY, n, &avgEntry.mag_y);
    arm_mean_f32(magZ, n, &avgEntry.mag_z);
    
    arm_mean_f32(accX, n, &avgEntry.acc_x);
    arm_mean_f32(accY, n, &avgEntry.acc_y);
    arm_mean_f32(accZ, n, &avgEntry.acc_z);
    
    arm_mean_f32(gyroX, n, &avgEntry.gyro_x);
    arm_mean_f32(gyroY, n, &avgEntry.gyro_y);
    arm_mean_f32(gyroZ, n, &avgEntry.gyro_z);

    return avgEntry;
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
    
    // Открываем файл для записи результатов
    FILE *outputFile = fopen(OUTPUT_FILE, "w");
    if (outputFile == NULL) {
        printf("Ошибка открытия файла для записи: %s\n", OUTPUT_FILE);
        freeDataCollection(&dataCollection);
        return 1;
    }
    
    // Записываем заголовок в файл
    fprintf(outputFile, "Time\tPitchSensor\tRollSensor\tAzimuthSensor\n");
    
    // Усреднение первых 10 записей
    DataEntry averagedEntry = averageFirstNEntries(&dataCollection, 10);
    // Расчет углов на усредненных данных
    Acceleration avg_acc = convertToAcceleration(&averagedEntry);
    Mag avg_mag = convertToMag(&averagedEntry);
    Angles avg_angles = calculateAngles(&avg_acc);
    float avg_azimuth = calculateAzimuth(avg_angles.pitch, avg_angles.roll, &avg_mag);
    // Записываем результаты в файл
    fprintf(outputFile, "%6.1f\t%15.10f\t%15.10f\t%15.10f\n", 
                averagedEntry.time, 
                avg_angles.pitch, 
                avg_angles.roll, 
                avg_azimuth);

    // Для каждой строки входного файла выполняем расчеты и записываем в файл
    for (int i = 10; i < dataCollection.count; i++) {
        // Извлекаем данные из текущей записи
        Mag mag = convertToMag(&dataCollection.entries[i]);
        Acceleration acc = convertToAcceleration(&dataCollection.entries[i]);
        // Вычисляем углы тангажа и крена
        Angles pitchRoll = calculateAngles(&acc);
        // Вычисляем магнитный азимут
        float32_t azimuth = calculateAzimuth(pitchRoll.pitch, pitchRoll.roll, &mag);
        
        // Записываем результаты в файл
        fprintf(outputFile, "%6.1f\t%15.10f\t%15.10f\t%15.10f\n", 
                dataCollection.entries[i].time, 
                pitchRoll.pitch, 
                pitchRoll.roll, 
                azimuth);
    }
    
    // Закрываем файл
    fclose(outputFile);
    printf("Результаты успешно записаны в файл: %s\n", OUTPUT_FILE);
    
    // Освобождаем память
    freeDataCollection(&dataCollection);
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
