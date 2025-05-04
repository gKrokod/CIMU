#ifndef IIR_H
#define IIR_H 

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

DataEntry filterStep (IIRFilter * filter, const DataEntry * data ); 

#endif // IIR_H
