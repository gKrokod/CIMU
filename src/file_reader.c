#include "file_reader.h"
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LENGTH 256
#define FILE_INPUT_DATA "Data/input.txt"

void initDataCollection(DataCollection* collection, int initialCapacity) {
    collection->entries = (DataEntry*)malloc(initialCapacity * sizeof(DataEntry));
    collection->count = 0;
    collection->capacity = initialCapacity;
}

void freeDataCollection(DataCollection* collection) {
    free(collection->entries);
    collection->entries = NULL;
    collection->count = 0;
    collection->capacity = 0;
}

void printInputData(const DataCollection* collection) {
    int rowsToPrint = collection->count;
    for (int i = 0; i < rowsToPrint; i++) {
        printf("Время: %.2f, Mag: (%.2f, %.2f, %.2f), Acc: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f), Temp: %.2f, Pitch-Roll-Azimuth: (%.2f, %.2f, %.2f)\n", 
               collection->entries[i].time,
               collection->entries[i].mag_x,
               collection->entries[i].mag_y,
               collection->entries[i].mag_z,
               collection->entries[i].acc_x,
               collection->entries[i].acc_y,
               collection->entries[i].acc_z,
               collection->entries[i].gyro_x,
               collection->entries[i].gyro_y,
               collection->entries[i].gyro_z,
               collection->entries[i].temp,
               collection->entries[i].pitch_sensor,
               collection->entries[i].roll_sensor,
               collection->entries[i].azimuth_sensor);
    }

};

// Вспомогательная функция для парсинга строки в запись данных
static int parseDataEntry(const char* line, DataEntry* entry) {
    // Реализация парсинга зависит от формата данных в input.txt
    // Предполагаем CSV-формат с разделителями-запятыми
    return sscanf(line, "%f\t%f\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%f\t%lf\t%lf\t%lf", 
                  &entry->time, 
                  &entry->time_step, 
                  &entry->mag_x, 
                  &entry->mag_y, 
                  &entry->mag_z,
                  &entry->acc_x, 
                  &entry->acc_y, 
                  &entry->acc_z,
                  &entry->gyro_x, 
                  &entry->gyro_y, 
                  &entry->gyro_z,
                  &entry->temp, 
                  &entry->pitch_sensor, 
                  &entry->roll_sensor,
                  &entry->azimuth_sensor);
}

int readDataFile(DataCollection* collection) {
    FILE* file = fopen(FILE_INPUT_DATA, "r");
    if (file == NULL) {
        return -1; // Ошибка открытия файла
    }
    
    char line[MAX_LINE_LENGTH];
    
    // Пропускаем заголовок (первую строку)
    if (fgets(line, MAX_LINE_LENGTH, file) == NULL) {
        fclose(file);
        return -2; // Ошибка чтения заголовка
    }
    
    // Читаем данные
    while (fgets(line, MAX_LINE_LENGTH, file)) {
        // Проверка, нужно ли увеличить массив
        if (collection->count >= collection->capacity) {
            int newCapacity = collection->capacity * 2;
            DataEntry* newEntries = (DataEntry*)realloc(collection->entries, 
                                                       newCapacity * sizeof(DataEntry));
            if (newEntries == NULL) {
                fclose(file);
                return -3; // Ошибка выделения памяти
            }
            collection->entries = newEntries;
            collection->capacity = newCapacity;
        }
        
        // Парсим строку
        if (parseDataEntry(line, &collection->entries[collection->count]) > 0) {
            collection->count++;
        }
    }
    
    fclose(file);
    return 0; // Успех
}

void printFirstNRows(const DataCollection* collection, int n) {
    int rowsToPrint = (collection->count < n) ? collection->count : n;
    
    for (int i = 0; i < rowsToPrint; i++) {
        printf("Время: %.2f, Mag: (%.2f, %.2f, %.2f), Acc: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f), Temp: %.2f, Pitch-Roll-Azimuth: (%.2f, %.2f, %.2f)\n", 
               collection->entries[i].time,
               collection->entries[i].mag_x,
               collection->entries[i].mag_y,
               collection->entries[i].mag_z,
               collection->entries[i].acc_x,
               collection->entries[i].acc_y,
               collection->entries[i].acc_z,
               collection->entries[i].gyro_x,
               collection->entries[i].gyro_y,
               collection->entries[i].gyro_z,
               collection->entries[i].temp,
               collection->entries[i].pitch_sensor,
               collection->entries[i].roll_sensor,
               collection->entries[i].azimuth_sensor);
    }
}

