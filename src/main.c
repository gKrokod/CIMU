#include <stdio.h>
#include "arm_math.h"
#include "file_reader.h"




int main() {
    // Инициализация коллекции данных
    DataCollection dataCollection;
    initDataCollection(&dataCollection, 10); // Начальная емкость 10

    // Чтение данных из файла
    int readResult = readDataFile(&dataCollection);
    if (readResult != 0) {
        printf("Ошибка чтения файла: %d\n", readResult);
        freeDataCollection(&dataCollection);
        return 1;
    }

    // Вывод первых трех строк
    printf("Первые три строки из файла:\n");
    printFirstNRows(&dataCollection, 3);

    // Обработка данных (здесь будет математическая обработка)
    /* ProcessedData results; */
    /* int processResult = processData(&dataCollection, &results); */
    /* if (processResult != 0) { */
    /*     printf("Ошибка обработки данных: %d\n", processResult); */
    /*     freeDataCollection(&dataCollection); */
    /*     return 2; */
    /* } */
    /*  */
    /* // Вывод результатов обработки */
    /* printf("\nРезультаты обработки (первые 3):\n"); */
    /* printProcessedData(&results, 3); */
    /*  */
    /* // Освобождение памяти */
    /* freeProcessedData(&results); */
    /* freeDataCollection(&dataCollection); */

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
