/*
* SD_Interface.h
*
* This file contains useful functions for saving and reading from the SD card
*/

#ifndef SDINTERFACE_H
#define SDINTERFACE_H

#include "User_Types.h"
#include <SdFat.h>

#define TMP_FILE_NAME "tmpfile"
#define FILE_BLOCK_COUNT 256000

// Create Binary File - This is adapted from SdFat.h example
void create_bin_file(SdFat *sd, SdBaseFile *binFile);

// Write data - This function records and writes one sample at a time
void record_bin_file(SdFat *sd, SdBaseFile *binFile, uint32_t microDelay, uint32_t recordTime);

// Write data - This is adapted from SdFat.h example
// This function records and writes in 512kB blocks
void record_bin_file_block(SdFat *sd, SdBaseFile *binFile, uint32_t microDelay, uint32_t recordTime);

void record_file(SdFat *sd, char *Filename, uint32_t microDelay, uint32_t recordTime);

// Read data
void read_bin_file(SdFat *sd, SdBaseFile *binFile);

// Convert data to csv
void convert_bin_to_csv(SdFat *sd, SdBaseFile *binFile, char *Filename);

#endif
