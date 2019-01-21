#include "User_Types.h"
#include "SD_Interface.h"
#include "MPU6050_Interface.h"
#include <SdFat.h>

void create_bin_file(SdFat *sd, SdBaseFile *binFile) {
  // max number of blocks to erase per erase call
  const uint32_t ERASE_SIZE = 262144L;
  uint32_t bgnBlock, endBlock;
  
  // Delete old tmp file.
  if (sd->exists(TMP_FILE_NAME)) {
    Serial.println(F("Deleting tmp file " TMP_FILE_NAME));
    if (!sd->remove(TMP_FILE_NAME)) {
      Serial.println("Can't remove tmp file");
    }
  }
  // Create new file.
  Serial.println(F("\nCreating new file"));
  binFile->close();
  if (!binFile->createContiguous(TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
    Serial.println("createContiguous failed");
  }
  // Get the address of the file on the SD.
  if (!binFile->contiguousRange(&bgnBlock, &endBlock)) {
    Serial.println("contiguousRange failed");
  }
  // Flash erase all data in the file.
  Serial.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock) {
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock) {
      endErase = endBlock;
    }
    if (!sd->card()->erase(bgnErase, endErase)) {
      Serial.println("erase failed");
    }
    bgnErase = endErase + 1;
  }
  Serial.println(F("Erase Complete"));
}

void record_bin_file(SdFat *sd, SdBaseFile *binFile, uint16_t microDelay, uint32_t recordTime) {
	uint32_t nSamples = recordTime / microDelay;
	uint32_t logTime = micros();
	imu_sample_t record_sample;

  Serial.println(F("Starting Record"));
  Serial.println(nSamples);
  if (!sd->card()->writeStart(binFile->firstBlock())) {
      Serial.println("writeStart failed");
  }
  
	for ( int i = 0; i < nSamples; i++ ) {
    Serial.println("Timing");
		int32_t delta;
		do {
			delta = micros() - logTime;
		} while (delta < 0);
		
    Serial.println("Next Step");
		// Grab sample
		record_sample.time_stamp = micros();
		ReadSensor(&( record_sample.sample ));
    Serial.println("Read Data");
		
		// Write to SD
		sd->card()->writeData((uint8_t*) &record_sample);
    Serial.println("SD Write");
		
		// Update expected time for next 
		logTime += microDelay;
	}
}

void record_file(SdFat *sd, char *Filename, uint16_t microDelay, uint32_t recordTime) {
  uint32_t nSamples = recordTime / microDelay;
  uint32_t logTime = micros();
  uint32_t sample_time;
  String record_line;
  SdFile csvFile;
  imu_data_t record_sample;
  if (!csvFile.open(Filename, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println("open csvFile failed");
  }
  
  Serial.println(F("Starting Record"));
  Serial.println(nSamples);
  
  for ( int i = 0; i < nSamples; i++ ) {
    int32_t delta;
    do {
      delta = micros() - logTime;
    } while (delta < 0);
    
    // Grab sample
    sample_time = micros();
    ReadSensor(&( record_sample ));
    
    record_line = String( String(sample_time) + "," + String(record_sample.x_accel) + "," + String(record_sample.y_accel) + "," + String(record_sample.z_accel) + "," + String(record_sample.x_gyro) + "," + String(record_sample.y_gyro) + "," + String(record_sample.z_gyro) );
    // Write to SD
    csvFile.println(record_line);
    
    // Update expected time for next 
    logTime += microDelay;
  }
  csvFile.close();
}

void record_bin_file_block(SdFat *sd, SdBaseFile *binFile, uint16_t microDelay, uint32_t recordTime) {
	const uint8_t BUFFER_BLOCK_COUNT = 10;
  
	const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 1;
	// Index of last queue location.
	const uint8_t QUEUE_LAST = QUEUE_DIM - 1;

	uint32_t maxBlocks = recordTime / microDelay / SAMPLES_PER_BLOCK;
  Serial.print("Max Blocks: ");
  Serial.println(maxBlocks);
	if ( maxBlocks > FILE_BLOCK_COUNT ) {
		Serial.println("Desired File Size is Too Large");
	}
	
	// Allocate extra buffer space.
	imu_data_block_t block[BUFFER_BLOCK_COUNT - 1];
	imu_data_block_t* curBlock = 0;
	imu_data_block_t* emptyStack[BUFFER_BLOCK_COUNT];
	uint8_t emptyTop;
	uint8_t minTop;

	imu_data_block_t* fullQueue[QUEUE_DIM];
	uint8_t fullHead = 0;
	uint8_t fullTail = 0;  

	// Use SdFat's internal buffer.
  Serial.println("cacheClear");
	emptyStack[0] = (imu_data_block_t*)sd->vol()->cacheClear();
	if (emptyStack[0] == 0) {
		Serial.println("cacheClear failed");
	}
	// Put rest of buffers on the empty stack.
	for (int i = 1; i < BUFFER_BLOCK_COUNT; i++) {
		emptyStack[i] = &block[i - 1];
	}
	emptyTop = BUFFER_BLOCK_COUNT;
	minTop = BUFFER_BLOCK_COUNT;
  
	// Start a multiple block write.
  Serial.println("Prep Write");
	if (!sd->card()->writeStart(binFile->firstBlock())) {
		Serial.println("writeStart failed");
	}
	
	uint32_t bn = 0;  
	uint32_t maxLatency = 0;
	uint32_t overrunTotal = 0;
	uint32_t logTime = micros();
	uint8_t sample_count = 0;
	while(1) {
		// Time for next data record.
		logTime += microDelay;
		if (curBlock == 0 && emptyTop != 0) {
			curBlock = emptyStack[--emptyTop];
			if (emptyTop < minTop) {
				minTop = emptyTop;
			}
			sample_count = 0;
		}
		int32_t delta;
		do {
			delta = micros() - logTime;
		} while (delta < 0);
		if (curBlock == 0) {
			overrunTotal++;
		}
    else {
      curBlock->block_storage[sample_count].time_stamp = micros();
			ReadSensor(&( curBlock->block_storage[sample_count++].sample ));
			if (sample_count == SAMPLES_PER_BLOCK) {
				fullQueue[fullHead] = curBlock;
				fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
				curBlock = 0;
			}
		}
		if (fullHead != fullTail && !sd->card()->isBusy()) {
			// Get address of block to write.
			imu_data_block_t* pBlock = fullQueue[fullTail];
			fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
			// Write block to SD.
			uint32_t usec = micros();
			if (!sd->card()->writeData((uint8_t*)pBlock)) {
				Serial.println("write data failed");
			}
			usec = micros() - usec;
			if (usec > maxLatency) {
				maxLatency = usec;
			}
			// Move block to empty queue.
			emptyStack[emptyTop++] = pBlock;
			bn++;
			if (bn == FILE_BLOCK_COUNT || bn == maxBlocks) {
				// File full so stop
				break;
			}
		}
	}
	if (!sd->card()->writeStop()) {
		Serial.println("writeStop failed");
	}
	Serial.print(F("Min Free buffers: "));
	Serial.println(minTop);
	Serial.print(F("Max block write usec: "));
	Serial.println(maxLatency);
	Serial.print(F("Overruns: "));
	Serial.println(overrunTotal);
	// Truncate file if recording stopped early.
	if (bn != FILE_BLOCK_COUNT) {
		Serial.println(F("Truncating file"));
		if (!binFile->truncate(512L * bn)) {
			Serial.println("Can't truncate file");
		}
	}
}

void read_bin_file(SdFat *sd, SdBaseFile *binFile) {
}

void convert_bin_to_csv(SdFat *sd, SdBaseFile *binFile, char *Filename) {
	uint8_t lastPct = 0;
	imu_sample_t data_sample;
	uint32_t t0 = millis();
	uint32_t syncCluster = 0;
	SdFile csvFile;
  String record_line;

	if (!binFile->isOpen()) {
		Serial.println();
		Serial.println(F("No current binary file"));
		return;
	}

	if (!csvFile.open(Filename, O_WRONLY | O_CREAT | O_TRUNC)) {
		Serial.println("open csvFile failed");
	}
	binFile->rewind();
	Serial.print(F("Writing: "));
	Serial.println(Filename);
  uint16_t counter = 0;
	while (binFile->read(&data_sample, sizeof(imu_sample_t)) == sizeof(imu_sample_t)) {
//		csvFile.print( data_sample.time_stamp );
//		csvFile.print(",");
//		csvFile.print( ConvertAccel( data_sample.sample.x_accel ) );
//		csvFile.print( "," );
//		csvFile.print( ConvertAccel( data_sample.sample.y_accel ) );
//		csvFile.print( "," );
//		csvFile.print( ConvertAccel( data_sample.sample.z_accel ) );
//		csvFile.print(",");
//		csvFile.print( ConvertGyro( data_sample.sample.x_gyro ) );
//		csvFile.print( "," );
//		csvFile.print( ConvertGyro( data_sample.sample.y_gyro ) );
//		csvFile.print( "," );
//		csvFile.println( ConvertGyro( data_sample.sample.z_gyro ) );

    record_line = String( String(data_sample.time_stamp) + "," + String(ConvertAccel( data_sample.sample.x_accel )) + "," + String(ConvertAccel( data_sample.sample.y_accel )) + "," + String(ConvertAccel( data_sample.sample.z_accel )) + "," + String(ConvertGyro( data_sample.sample.x_gyro )) + "," + String(ConvertGyro( data_sample.sample.y_gyro )) + "," + String(ConvertGyro( data_sample.sample.z_gyro )) );
    // Write to SD
    csvFile.println(record_line);
    
   counter++;
   if ( counter == 1000 ) {
    Serial.println("1000 Samples Converted...");
    counter = 0;
    csvFile.flush();
   }
	}
	csvFile.close();
	Serial.println(F("Done: "));
}
