#include "Logger.h"
#include <stdio.h>
#include "Printer.h"
extern Printer printer;
String message;

Logger::Logger(void)
	: num_datasources(0)
{}

void Logger::include(DataSource * source_p) {
	sources[num_datasources] = source_p;
	++num_datasources;
}


void Logger::padding(int number, byte width, String & str) {
	int currentMax = 10;
	for (byte i = 1; i < width; ++i) {
		if (number < currentMax) {
			str.concat("0");
		}
		currentMax *= 10;
	}
	str.concat(number);
}

void Logger::init(void) {
	Serial.print("Initializing SD Card... ");
  if (!SD.begin()) {
    Serial.println("failed!");
    return;
  }
  Serial.println("done!");

	unsigned int number = 0;
	String numstr = "";
	padding(number, 3, numstr);
	String finalname = LOG_FILENAME_BASE + numstr + ".bin";
	finalname.toCharArray(logfilename, LOG_FILENAME_BUFFERLEN);

	while(SD.exists(logfilename)) {
		number++;
		numstr = "";
		padding(number, 3, numstr);
		finalname = LOG_FILENAME_BASE + numstr + ".bin";
		finalname.toCharArray(logfilename, LOG_FILENAME_BUFFERLEN);
	}

	finalname = HEADINGS_FILENAME_BASE + numstr + ".txt";
	finalname.toCharArray(headingfilename, LOG_FILENAME_BUFFERLEN);

	message = "Logger: Using log file name " + String(logfilename);
	printer.printMessage(message,30);

	String headingStr = sources[0]->csvVarNames;
	String dataTypeStr = sources[0]->csvDataTypes;
	for(size_t i = 1; i < num_datasources; ++i) {
		headingStr += ",";
		headingStr += sources[i]->csvVarNames;
		dataTypeStr += ",";
		dataTypeStr += sources[i]->csvDataTypes;
	}
	headingStr += "\n"+dataTypeStr;
	
	///// TESTING /////
    //printer.printMessage(headinfStr,20);
	///// TESTING /////

	file = SD.open(headingfilename, FILE_WRITE);

	// if the file exists, use it:
  if (file) {
    file.println(headingStr);
    file.close();
	}

	printer.printMessage("Creating log file",10);
	file = SD.open(logfilename, FILE_WRITE);
	if(!file) {
		message = "Logger: error creating " + String(logfilename);
		printer.printMessage(message,0);
	} else {
		file.close();
	}

	// if exiting without error
	keepLogging = true;
}

void Logger::log(void){
	// record data from sources
	size_t idx = 0;
	unsigned char buffer[BYTES_PER_BLOCK];
	for(size_t i = 0; i < num_datasources; ++i) {
		idx = sources[i]->writeDataBytes(buffer, idx);
		if (idx >= BYTES_PER_BLOCK) {
			printer.printMessage("Too much data per log. Increase BYTES_PER_BLOCK or reduce data", 2);
		}
	}

	// write data to SD
	if (writtenBlocks >= FILE_BLOCK_COUNT) {
		printer.printMessage("Current file size limit reached. Change FILE_BLOCK_COUNT to fix. Stopping logging for now.",0);
		keepLogging = false;
	}

	file = SD.open(logfilename, FILE_WRITE);
	if (file) {
		int bytes = file.write(&buffer[0],BYTES_PER_BLOCK);
		if (!bytes) {
			printer.printMessage("Logger: Error printing to SD",0);
		}
	}
	file.close();

	writtenBlocks++;
	keepLogging = true;
}

String Logger::printState(void){
	String printString = "Logger: ";
	if(keepLogging) {
		printString += "Just logged buffer " + String(writtenBlocks) + 
		" to file: " + String(logfilename);
	} else {
		printString += "LOGGING ERROR, LOGGING HAS STOPPED";
	}

	return printString;
}
