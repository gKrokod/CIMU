#ifndef FILE_READER_H
#define FILE_READER_H //guard for double include file

#include <stdio.h>
#include "arm_math.h"

// Структура для хранения данных из файла
typedef struct {
    float32_t time;
    float32_t time_step;
    float64_t mag_x;
    float64_t mag_y;
    float64_t mag_z;
    float64_t acc_x;
    float64_t acc_y;
    float64_t acc_z;
    float64_t gyro_x;
    float64_t gyro_y;
    float64_t gyro_z;
    float32_t temp;
    float64_t pitch_sensor;
    float64_t roll_sensor;
    float64_t azimuth_sensor;
} DataEntry;

// Структура для хранения коллекции записей
typedef struct {
    DataEntry* entries;  // Массив записей
    int count;           // Количество записей
    int capacity;        // Емкость массива
} DataCollection;

// Инициализация коллекции данных
void initDataCollection(DataCollection* collection, int initialCapacity);

// Освобождение памяти коллекции
void freeDataCollection(DataCollection* collection);

// Чтение файла и заполнение коллекции данных
int readDataFile(DataCollection* collection);

// Печать первых N строк из коллекции
void printFirstNRows(const DataCollection* collection, int n);

// Печать первых N строк из коллекции
void printInputData(const DataCollection* collection);

#endif // FILE_READER_H

