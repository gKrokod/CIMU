#ifndef FILE_WRITER_H
#define FILE_WRITER_H //guard for double include file

#include <stdio.h>
#include "file_reader.h"

#define MAX_AVERAGE_SAMPLES 10
#define OUTPUT_FILE "Data/output.txt"

DataEntry file_writer_title(const FILE* file, const DataCollection* collection );

#endif // FILE_WRITER
