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

// Вспомогательная функция для парсинга строки в запись данных
static int parseDataEntry(const char* line, DataEntry* entry) {
    // Реализация парсинга зависит от формата данных в input.txt
    // Предполагаем CSV-формат с разделителями-запятыми
    return sscanf(line, "%lf,%lf,%lf,%lf", 
                  &entry->timestamp, 
                  &entry->angle_x, 
                  &entry->angle_y, 
                  &entry->angle_z);
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
        printf("Строка %d: Время: %.2f, Углы: (%.2f, %.2f, %.2f)\n", 
               i + 1,
               collection->entries[i].timestamp,
               collection->entries[i].angle_x,
               collection->entries[i].angle_y,
               collection->entries[i].angle_z);
    }
}

