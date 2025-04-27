#ifndef ANGLES_H
#define ANGLES_H //guard for double include file

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

Acceleration convertToAcceleration(const DataEntry* entry); 
// Acceleration convertToAcceleration(const DataEntry* entry) {
//     Acceleration acc;
//     acc.x = entry->acc_x;
//     acc.y = entry->acc_y;
//     acc.z = entry->acc_z;
//     return acc;
// }
Mag convertToMag(const DataEntry* entry);
// Mag convertToMag(const DataEntry* entry) {
//     Mag mag;
//     mag.x = entry->mag_x;
//     mag.y = entry->mag_y;
//     mag.z = entry->mag_z;
//     return mag;
// }
Gyro convertToGyro(const DataEntry* entry);
// Gyro convertToGyro(const DataEntry* entry) {
//     Gyro gyro;
//     gyro.x = entry->gyro_x;
//     gyro.y = entry->gyro_y;
//     gyro.z = entry->gyro_z;
//     return gyro;
// }
Angles convertToAngles(const DataEntry* entry);
// Angles convertToAngles(const DataEntry* entry) {
//     Angles angles;
//     angles.pitch = entry->pitch_sensor;
//     angles.roll = entry->roll_sensor;
//     return angles;
// }

void printStruct(const Acceleration* sensor);
// void printStruct(const Acceleration* sensor){
//   printf("Triad: (%.2f, %.2f, %.2f)\n",
//    sensor->x,
//    sensor->y,
//    sensor->z);
// }
void printAngles(const Angles* sensor);
// void printAngles(const Angles* sensor){
//   printf("Angles: (%.2f, %.2f)\n",
//    sensor->pitch,
//    sensor->roll);
// }

float32_t calculateRoll(const Acceleration* acc); // DEG
// float32_t calculateRoll(const Acceleration* acc) { // DEG
//     const float32_t n = acc->y;
//     const float32_t d = acc->z;
//     return (atan2f(n, d) * RAD_TO_DEG);
// }

float32_t calculatePitch(const Acceleration* acc); // DEG
// float32_t calculatePitch(const Acceleration* acc) { // DEG
//     const float32_t n = acc->x;
//     const float32_t vec[2] = {acc->y, acc->z};
//     float32_t sum_squares;
//     arm_dot_prod_f32(&vec, &vec, 2, &sum_squares);
//     float32_t d;
//     arm_sqrt_f32(sum_squares, &d);
//     return -(atan2f(n, d) * RAD_TO_DEG);
// }

Angles calculateAngles(const Acceleration* acc); //DEG
// Angles calculateAngles(const Acceleration* acc) { //DEG
//   Angles angles;
//   angles.pitch = calculatePitch(acc);
//   angles.roll = calculateRoll(acc);
//   return angles;
// }

float32_t calculateAzimuth(
    const float32_t pitch_deg, 
    const float32_t roll_deg, 
    const Mag* mag
); 
// float32_t calculateAzimuth(
//     const float32_t pitch_deg, 
//     const float32_t roll_deg, 
//     const Mag* mag
// ) {
//   
//     const float32_t sin_pitch, cos_pitch;
//     arm_sin_cos_f32(pitch_deg, &sin_pitch, &cos_pitch);
//
//     const float32_t sin_roll, cos_roll;
//     arm_sin_cos_f32(roll_deg, &sin_roll, &cos_roll);
//
//     const float32_t n = 
//         -(mag->y * cos_roll - mag->z * sin_roll);
//     
//     const float32_t d = 
//         (mag->y * sin_roll + mag->z * cos_roll) * sin_pitch + 
//         mag->x * cos_pitch;
//     
//     // Вычисление азимута с корректировкой квадрантов
//     float32_t azimuth = atan2f(n, d);
//     if (n >= 0 && d >= 0) {azimuth += 0;} ;
//     if (n >= 0 && d < 0) {azimuth += PI;} ;
//     if (n < 0 && d >= 0) {azimuth +=  2 * PI;} ;
//     if (n < 0 && d < 0) {azimuth += PI;} ;
//     //
//     // Преобразование в градусы
//     return azimuth * RAD_TO_DEG;
// }
//

#endif // ANGLES_H

