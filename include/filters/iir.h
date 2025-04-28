#ifndef IIR_H
#define IIR_H //guard for double include file
                 //
#include "file_reader.h"

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

void initialFilter (IIRFilter * filter, DataEntry * data);
// void initialFilter (IIRFilter * filter, DataEntry * data){
//   filter->mag_x  = data->mag_x;
//   filter->mag_y  = data->mag_y;
//   filter->mag_z  = data->mag_z;
//   filter->acc_x  = data->acc_x;
//   filter->acc_y  = data->acc_y;
//   filter->acc_z  = data->acc_z;
//
// }
// берем новые данные, параметры фильтра с прошлого шага и получаем новые 
// параметры фильтра и данные для расчета углов
// вызывает интерес функция arm_iir_lattice_f32
DataEntry filterStep (IIRFilter * filter, const DataEntry * data ); 

// DataEntry filterStep (IIRFilter * filter, const DataEntry * data ) {
//   DataEntry filteredData;
//
//   filter->mag_x  = ALPHA_LOW_PASS * filter->mag_x + ( 1 - ALPHA_LOW_PASS) * data->mag_x;
//   filter->mag_y  = ALPHA_LOW_PASS * filter->mag_y + ( 1 - ALPHA_LOW_PASS) * data->mag_y;
//   filter->mag_z  = ALPHA_LOW_PASS * filter->mag_z + ( 1 - ALPHA_LOW_PASS) * data->mag_z;
//   filter->acc_x  = ALPHA_LOW_PASS * filter->acc_x + ( 1 - ALPHA_LOW_PASS) * data->acc_x;
//   filter->acc_y  = ALPHA_LOW_PASS * filter->acc_y + ( 1 - ALPHA_LOW_PASS) * data->acc_y;
//   filter->acc_z  = ALPHA_LOW_PASS * filter->acc_z + ( 1 - ALPHA_LOW_PASS) * data->acc_z;
//
//   filteredData.time = data->time;
//   filteredData.time_step = data->time_step;
//   filteredData.mag_x = filter->mag_x;
//   filteredData.mag_y = filter->mag_y;
//   filteredData.mag_z = filter->mag_z;
//   filteredData.acc_x = filter->acc_x;
//   filteredData.acc_y = filter->acc_y;
//   filteredData.acc_z = filter->acc_z;
//   filteredData.gyro_x = data->gyro_x;
//   filteredData.gyro_y = data->gyro_y;
//   filteredData.gyro_z = data->gyro_z;
//   filteredData.temp = data->temp;
//   // эти будут пересчитаны, они так считаны из входного файла
//   filteredData.pitch_sensor = data->pitch_sensor;
//   filteredData.roll_sensor = data->roll_sensor;
//   filteredData.azimuth_sensor = data->azimuth_sensor;
//   return filteredData; 
// }
#endif // IIR_H
