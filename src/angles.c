#include "angles.h"
#include "arm_math.h"

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
    float32_t roll;
    arm_atan2_f32(n, d, &roll);
    if (roll > PI / 2) {roll -= PI;}
    else if (roll < -PI / 2) {roll += PI;};
    return (roll * RAD_TO_DEG);
}

float32_t calculatePitch(const Acceleration* acc) { // DEG
    const float32_t n = acc->x;
    const float32_t vec[2] = {acc->y, acc->z};
    float32_t sum_squares;
    arm_dot_prod_f32(vec, vec, 2, &sum_squares);
    float32_t d;
    arm_sqrt_f32(sum_squares, &d);
    float32_t pitch;
    arm_atan2_f32(n, d, &pitch);//status check todo
    if (pitch > PI / 2) {pitch -= PI;} // otherwise [-p;p]
    else if (pitch < -PI / 2) {pitch += PI;};
    return -(pitch * RAD_TO_DEG);
}

Angles calculateAngles(const Acceleration* acc) { //DEG
  Angles angles;
  angles.pitch = calculatePitch(acc);
  angles.roll = calculateRoll(acc);
  return angles;
}

float32_t calculateAzimuth( //DEG
    const float32_t pitch_deg, 
    const float32_t roll_deg, 
    const Mag* mag
) {

    float32_t sin_pitch, cos_pitch;
    arm_sin_cos_f32(pitch_deg, &sin_pitch, &cos_pitch);

    float32_t sin_roll, cos_roll;
    arm_sin_cos_f32(roll_deg, &sin_roll, &cos_roll);

    const float32_t n = -(mag->y * cos_roll - mag->z * sin_roll);

    const float32_t d = (mag->y * sin_roll + mag->z * cos_roll) * sin_pitch + 
                        mag->x * cos_pitch;

    // Вычисление азимута с корректировкой квадрантов
    float32_t azimuth;
    arm_atan2_f32(n, d, &azimuth);//status check todo
    if (azimuth < 0.0f) {azimuth += 2 * PI;}                                  

    /* float32_t azimuth = atan(n/d); //robust */
    /* if (n >= 0 && d >= 0) {azimuth += 0;} ; */
    /* if (n >= 0 && d < 0) {azimuth += PI;} ; */
    /* if (n < 0 && d >= 0) {azimuth +=  2 * PI;} ; */
    /* if (n < 0 && d < 0) {azimuth += PI;} ; */
    //
    return (azimuth * RAD_TO_DEG);
}
