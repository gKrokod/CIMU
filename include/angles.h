#ifndef ANGLES_H
#define ANGLES_H 

#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"

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

Mag convertToMag(const DataEntry* entry);

Gyro convertToGyro(const DataEntry* entry);

Angles convertToAngles(const DataEntry* entry);

void printStruct(const Acceleration* sensor);

void printAngles(const Angles* sensor);

float32_t calculateRoll(const Acceleration* acc); // DEG

float32_t calculatePitch(const Acceleration* acc); // DEG

Angles calculateAngles(const Acceleration* acc); //DEG

float32_t calculateAzimuth(
    const float32_t pitch_deg, 
    const float32_t roll_deg, 
    const Mag* mag
); //DEG

#endif // ANGLES_H

