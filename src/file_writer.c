#include "file_writer.h"
#include "file_reader.h"
#include <stdio.h>
#include "arm_math.h"
#include "angles.h"

/* #define FILE_OUTPUT_DATA "Data/output.txt" */

/* #define MAX_AVERAGE_SAMPLES 10 */

static DataEntry averageFirstNEntries(const DataCollection* collection, const uint32_t n) {
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

void file_writer_title(const FILE* file, const DataCollection* collection) {
    // Записываем заголовок в файл
    fprintf(file, "Time\tPitchSensor\tRollSensor\tAzimuthSensor\n");
    // Усреднение первых 10 записей
    DataEntry averagedEntry = averageFirstNEntries(collection, MAX_AVERAGE_SAMPLES);
    // Расчет углов на усредненных данных
    Acceleration avg_acc = convertToAcceleration(&averagedEntry);
    Mag avg_mag = convertToMag(&averagedEntry);
    Angles avg_angles = calculateAngles(&avg_acc);
    float avg_azimuth = calculateAzimuth(avg_angles.pitch, avg_angles.roll, &avg_mag);
    // Записываем результаты в файл
    fprintf(file, "%6.1f\t%15.10f\t%15.10f\t%15.10f\n", 
                averagedEntry.time, 
                avg_angles.pitch, 
                avg_angles.roll, 
                avg_azimuth);
};
