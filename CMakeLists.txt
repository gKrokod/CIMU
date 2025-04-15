cmake_minimum_required(VERSION 3.10)

# название проекта и используемый язык
project(CIMU C)

# установка стандарта языка
set(CMAKE_C_STANDARD 11)

# включение предупреждений компилятора
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")


# добавление исполняемого файла из исходного кода
add_executable(CIMU src/main.c)

