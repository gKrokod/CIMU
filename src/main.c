#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"
#include "file_writer.h"
#include "angles.h"

#define DELTA_T 0.04f
#define ALPHA_LOW_PASS 0.081f //10 Hz

// simple IIR Filter
typedef struct {
    float32_t mag_x;
    float32_t mag_y;
    float32_t mag_z;
    float32_t acc_x;
    float32_t acc_y;
    float32_t acc_z;
} IIRFilter;

void initialFilter (IIRFilter * filter, DataEntry * data){
  filter->mag_x  = data->mag_x;
  filter->mag_y  = data->mag_y;
  filter->mag_z  = data->mag_z;
  filter->acc_x  = data->acc_x;
  filter->acc_y  = data->acc_y;
  filter->acc_z  = data->acc_z;

}
// берем новые данные, параметры фильтра с прошлого шага и получаем новые 
// параметры фильтра и данные для расчета углов
// вызывает интерес функция arm_iir_lattice_f32
DataEntry filterStep (IIRFilter * filter, const DataEntry * data ) {
  DataEntry filteredData;

  filter->mag_x  = ALPHA_LOW_PASS * filter->mag_x + ( 1 - ALPHA_LOW_PASS) * data->mag_x;
  filter->mag_y  = ALPHA_LOW_PASS * filter->mag_y + ( 1 - ALPHA_LOW_PASS) * data->mag_y;
  filter->mag_z  = ALPHA_LOW_PASS * filter->mag_z + ( 1 - ALPHA_LOW_PASS) * data->mag_z;
  filter->acc_x  = ALPHA_LOW_PASS * filter->acc_x + ( 1 - ALPHA_LOW_PASS) * data->acc_x;
  filter->acc_y  = ALPHA_LOW_PASS * filter->acc_y + ( 1 - ALPHA_LOW_PASS) * data->acc_y;
  filter->acc_z  = ALPHA_LOW_PASS * filter->acc_z + ( 1 - ALPHA_LOW_PASS) * data->acc_z;

  filteredData.time = data->time;
  filteredData.time_step = data->time_step;
  filteredData.mag_x = filter->mag_x;
  filteredData.mag_y = filter->mag_y;
  filteredData.mag_z = filter->mag_z;
  filteredData.acc_x = filter->acc_x;
  filteredData.acc_y = filter->acc_y;
  filteredData.acc_z = filter->acc_z;
  filteredData.gyro_x = data->gyro_x;
  filteredData.gyro_y = data->gyro_y;
  filteredData.gyro_z = data->gyro_z;
  filteredData.temp = data->temp;
  // эти будут пересчитаны, они так считаны из входного файла
  filteredData.pitch_sensor = data->pitch_sensor;
  filteredData.roll_sensor = data->roll_sensor;
  filteredData.azimuth_sensor = data->azimuth_sensor;
  return filteredData; 
}


int main() {
    // Инициализация коллекции данных
    printf("start\n");
    DataCollection dataCollection;
    initDataCollection(&dataCollection, 10); // Начальная емкость 10

    // Чтение данных из файла
    int readResult = readDataFile(&dataCollection);
    // Открываем файл для записи результатов
    FILE *outputFile = fopen(OUTPUT_FILE, "w");
    // write Title and average value
    DataEntry dataAverage = file_writer_title(outputFile, &dataCollection);
    IIRFilter iir;
    initialFilter (&iir, &dataAverage);
    printf("%.2f", iir.acc_z);
    

    // Для каждой строки входного файла выполняем расчеты и записываем в файл
    for (int i = MAX_AVERAGE_SAMPLES; i < dataCollection.count; i++) {
        Mag mag = convertToMag(&dataCollection.entries[i]);
        Acceleration acc = convertToAcceleration(&dataCollection.entries[i]);
        Angles pitchRoll = calculateAngles(&acc);
        float32_t azimuth = calculateAzimuth(pitchRoll.pitch, pitchRoll.roll, &mag);

        DataEntry data_i = filterStep (&iir,&dataCollection.entries[i]);

        Mag iir_mag = convertToMag(&data_i);
        Acceleration iir_acc = convertToAcceleration(&data_i);
        Angles iir_pitchRoll = calculateAngles(&iir_acc);
        float32_t iir_azimuth = calculateAzimuth(iir_pitchRoll.pitch, iir_pitchRoll.roll, &iir_mag);
        
        // Записываем результаты в файл
        fprintf(outputFile, "%6.1f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\t%15.10f\n", 
                dataCollection.entries[i].time, 
                pitchRoll.pitch, 
                pitchRoll.roll, 
                azimuth,
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
