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

    KalmanFilter kf; // todo zadaj
    /*                  // */
    Acceleration avg_acc = convertToAcceleration(&dataAverage);
    Mag avg_mag = convertToMag(&dataAverage);
    Angles avg_angles = calculateAngles(&avg_acc);
    /*  */
    Kalman_Init(&kf, avg_angles.pitch, avg_angles.roll);
    /*  */
    printf("%.2f %.2f ", avg_angles.pitch, avg_angles.roll);

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
        Gyro gyro = convertToGyro(&dataCollection.entries[i]);
        float32_t wN = 0.0f; // Реализуйте расчет wN и wE по данным гироскопа
        float32_t wE = 0.0f; // согласно вашей логике
        /*  */
        /* // Прогноз Калмана */
        Kalman_Predict(&kf, DELTA_TT, wN, wE);
        /*  */
        /* // Обновление Калмана */
        Kalman_Update(&kf, pitchRoll.pitch, pitchRoll.roll);
        /*  */
        /* // Получение отфильтрованных углов */
        float32_t kalman_pitch, kalman_roll;
        Kalman_GetAngles(&kf, &kalman_pitch, &kalman_roll);
        float32_t kalman_azimuth = calculateAzimuth(kalman_pitch, kalman_roll, &iir_mag);

        fprintf(outputFile, "%6.1f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\n",
                dataCollection.entries[i].time,
                pitchRoll.pitch,
                pitchRoll.roll,
                azimuth,
                iir_pitchRoll.pitch,
                iir_pitchRoll.roll,
                iir_azimuth,
                kalman_pitch,
                kalman_roll,
                kalman_azimuth);

    }
    // Закрываем файл
    fclose(outputFile);
    printf("\nРезультаты успешно записаны в файл: %s\n", OUTPUT_FILE);
    
    // Освобождаем память
    freeDataCollection(&dataCollection);
    return 0;
}
