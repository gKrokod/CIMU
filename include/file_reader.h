#ifndef FILE_READER_H
#define FILE_READER_H //guard for double include file

#include <stdio.h>

// Структура для хранения данных из файла
typedef struct {
    // Предполагаем, что в файле содержатся данные об углах
    double timestamp;
    double angle_x;
    double angle_y;
    double angle_z;
    // ... другие поля по необходимости
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

#endif // FILE_READER_H

