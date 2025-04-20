cmake_minimum_required(VERSION 3.10)

# название проекта и используемый язык
project(CIMU C)

# установка стандарта языка
set(CMAKE_C_STANDARD 11)

# включение предупреждений компилятора
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

set(CMSISDSP ${CMAKE_SOURCE_DIR}/CMSIS-DSP)
set(CMSISCORE ${CMAKE_SOURCE_DIR}/CMSIS_5/CMSIS/Core)
include_directories(${CMSISCORE}/Include)
include_directories(${CMSISDSP}/Include)
add_subdirectory(${CMSISDSP}/Source bin_dsp)

# добавление исполняемого файла из исходного кода
add_executable(CIMU src/main.c)

target_link_libraries(CIMU PRIVATE CMSISDSP)
target_link_libraries(CIMU PRIVATE m)
