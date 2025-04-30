#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"
#include "file_writer.h"
#include "filters/iir.h"
#include "filters/kalman.h"
#include "angles.h"

#define DELTA_TT 0.04f

int main() {
    // Инициализация коллекции данных
    printf("startNEW\n");
    DataCollection dataCollection;
    initDataCollection(&dataCollection, 10); // Начальная емкость 10

    // Чтение данных из файла
    int readResult = readDataFile(&dataCollection);
    // Открываем файл для записи результатов
    FILE *outputFile = fopen(OUTPUT_FILE, "w");
    // write Title and average value
    // усредняяем здесь первые значения для исходных данных.
    DataEntry dataAverage = file_writer_title(outputFile, &dataCollection);
    IIRFilter iir;
    initialFilter (&iir, &dataAverage);

    /* KalmanFilter kf; //  */
    /* Acceleration avg_acc = convertToAcceleration(&dataAverage); */
    /* Mag avg_mag = convertToMag(&dataAverage); */
    /* Angles avg_angles = calculateAngles(&avg_acc); */
    /* #<{(|  |)}># */
    /* Kalman_Init(&kf, avg_angles.pitch, avg_angles.roll); */
    /*  */

    // Для каждой строки входного файла выполняем расчеты и записываем в файл
    for (int i = MAX_AVERAGE_SAMPLES; i < dataCollection.count; i++) {
      // сырые данные
        Mag mag = convertToMag(&dataCollection.entries[i]);
        Acceleration acc = convertToAcceleration(&dataCollection.entries[i]);
        Angles pitchRoll = calculateAngles(&acc);
        float32_t azimuth = calculateAzimuth(pitchRoll.pitch, pitchRoll.roll, &mag);
        
// iir фильтр
        DataEntry data_i = filterStep (&iir,&dataCollection.entries[i]);
        Mag iir_mag = convertToMag(&data_i);
        Acceleration iir_acc = convertToAcceleration(&data_i);
        Angles iir_pitchRoll = calculateAngles(&iir_acc);
        float32_t iir_azimuth = calculateAzimuth(iir_pitchRoll.pitch, iir_pitchRoll.roll, &iir_mag);
        
// Kalman фильтр
        /* float32_t kalman_pitch, kalman_roll; */
        /* Kalman_GetAngles(&kf, &kalman_pitch, &kalman_roll); */
        /* #<{(| printf("%.2f %.2f \t ", kalman_pitch, kalman_roll); |)}># */
        /*  */
        /* const float32_t sin_tettha, cos_tettha; */
        /* arm_sin_cos_f32(kalman_pitch, &sin_tettha, &cos_tettha); */
        /* const float32_t sin_phi, cos_phi; */
        /* arm_sin_cos_f32(kalman_roll, &sin_phi, &cos_phi); */
        /*  */
        /* Gyro gyro = convertToGyro(&dataCollection.entries[i]); */
        /*  */
        /* float32_t wE = gyro.y * cos_phi - gyro.z * sin_phi; // градус в секунду */
        /* float32_t wN = gyro.x * cos_tettha + sin_tettha * (gyro.z * cos_phi + gyro.y * sin_phi); // градус в секунду */

        /* printf("we and wn : %.2f %.2f\t ", wE, wN); */
        /* printf("%.2f %.2f  %.2f\n", gyro.x, gyro.y, gyro.z); */
        /* // Прогноз Калмана */
        /* Kalman_Predict(&kf, DELTA_TT, wN, wE); */
        /*  */
        /* // Обновление Калмана */
        /* Kalman_Update(&kf, pitchRoll.pitch, pitchRoll.roll); */
        /*  */
        /* // Получение отфильтрованных углов */
        /* Kalman_GetAngles(&kf, &kalman_pitch, &kalman_roll); */

        /* float32_t kalman_azimuth = calculateAzimuth(kalman_pitch, kalman_roll, &iir_mag); */
        /* printf(" :: %.2f %.2f \n ", kalman_pitch, kalman_roll); */

        /* fprintf(outputFile, "%6.1f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\n", */
        /*         dataCollection.entries[i].time, */
        /*         pitchRoll.pitch, */
        /*         pitchRoll.roll, */
        /*         azimuth, */
        /*         kalman_pitch, */
        /*         kalman_roll, */
        /*         kalman_azimuth, */
        /*         iir_pitchRoll.pitch, */
        /*         iir_pitchRoll.roll, */
        /*         iir_azimuth); */

    }
    // Закрываем файл
    fclose(outputFile);
    printf("\nРезультаты успешно записаны в файл: %s\n", OUTPUT_FILE);
    
    // Освобождаем память
    freeDataCollection(&dataCollection);
    return 0;
}
