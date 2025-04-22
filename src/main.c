#include <stdio.h>
#include "arm_math.h"

#define MAX_LINE_LENGTH 256

int main() {
  FILE *file;
  char line[MAX_LINE_LENGTH];
  int line_count = 0;

  file = fopen("Data/input.txt","r"); //open for reading
  if (file == NULL) {
    printf("can't be opened Data/input.txt\n");
    return 1;
  }

  while (fgets(line, MAX_LINE_LENGTH, file) && line_count < 3) {
    printf("%s", line);
    line_count++;
  }

  fclose(file); // release memory
  return 0;
}

/* int main() { */
/*     // Инициализация матриц */
/*     float32_t A_data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};  // Матрица 2x3 */
/*     float32_t B_data[6] = {7.0, 8.0, 9.0, 10.0, 11.0, 12.0};  // Матрица 3x2 */
/*     float32_t C_data[4];  // Результат матричного умножения 2x2 */
/*      */
/*     arm_matrix_instance_f32 A; */
/*     arm_matrix_instance_f32 B; */
/*     arm_matrix_instance_f32 C; */
/*      */
/*     // Инициализация структур матриц */
/*     arm_mat_init_f32(&A, 2, 3, A_data); */
/*     arm_mat_init_f32(&B, 3, 2, B_data); */
/*     arm_mat_init_f32(&C, 2, 2, C_data); */
/*      */
/*     // Умножение матриц A * B = C */
/*     arm_status status = arm_mat_mult_f32(&A, &B, &C); */
/*      */
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
/*      */
/*     return 0; */
/* } */
