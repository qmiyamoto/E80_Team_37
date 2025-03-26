#ifndef __PRINTER_H__
#define __PRINTER_H__

#include <Arduino.h>

#define LONGEST_STRING 80 // characters

/*
 * Print controls the serial printing for all funtions in Default_Robot, so that
 * it doesn't become overwhelming
 */
class Printer {
public:
  Printer(void);

  void init(void);

  // Prints the value on the given row number until it's updated
  void printValue(int rowNumber, String value);

  // prints errors, refreshes timer if the message is already up
  void printMessage(String message, int messageTime);

  // actually displays the held information
  void printToSerial(void);

  int lastExecutionTime = -1;

private:
  // highest allowed row number (for both errors and values)
  static const int maxRow = 13;

  // stores all repeatedly printed information
  char printInfo[maxRow][LONGEST_STRING];

  // stores temporary messages
  char messages[maxRow][LONGEST_STRING];

  // stores number of displays left for each message
  int messageTimes[maxRow];
};

#endif
