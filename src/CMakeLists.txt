# добавление исполняемого файла из исходного кода
add_executable(CIMU 
  main.c
  file_reader.c
  angles.c
)

target_link_libraries(CIMU PRIVATE CMSISDSP)
# # math standart
target_link_libraries(CIMU PRIVATE m)


#
target_include_directories (CIMU PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
