#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"
#include "file_writer.h"
#include "filters/iir.h"
#include "filters/kalman.h"
#include "angles.h"

int main() {
    // Инициализация коллекции данных
    DataCollection dataCollection;
    initDataCollection(&dataCollection, 10); // Начальная емкость 10

    // Чтение данных из файла
    readDataFile(&dataCollection);
    // Открываем файл для записи результатов
    FILE *outputFile = fopen(OUTPUT_FILE, "w");
    // write Title and average value
    // усредняяем здесь первые значения для исходных данных.
    DataEntry dataAverage = file_writer_title(outputFile, &dataCollection);
    IIRFilter iir;
    initialFilter (&iir, &dataAverage);

    Acceleration avg_acc = convertToAcceleration(&dataAverage);
    Angles avg_angles = calculateAngles(&avg_acc);
    KalmanFilter_t kf;
    kalman_init(&kf, DEG_TO_RAD * avg_angles.pitch, DEG_TO_RAD * avg_angles.roll);

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

        kalman_step(&kf, &pitchRoll, &gyro); 

        float32_t kalman_azimuth = calculateAzimuth(RAD_TO_DEG * kf.vec_X[0], RAD_TO_DEG * kf.vec_X[1], &iir_mag);

// Output file 
        fprintf(outputFile, "%6.1f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\n",
                dataCollection.entries[i].time,
                pitchRoll.pitch,
                pitchRoll.roll,
                azimuth,
                RAD_TO_DEG * kf.vec_X[0],
                RAD_TO_DEG * kf.vec_X[1],
                kalman_azimuth,
                iir_pitchRoll.pitch,
                iir_pitchRoll.roll,
                iir_azimuth);

    }
    // Закрываем файл
    fclose(outputFile);
    printf("\nРезультаты успешно записаны в файл: %s\n", OUTPUT_FILE);
    
    // Освобождаем память
    freeDataCollection(&dataCollection);
    return 0;
}
