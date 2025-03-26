#include "Printer.h"
#include <algorithm>

Printer::Printer(void) {
}

void Printer::init(void) {
  // initializes the serial port
  Serial.begin(115200);
  delay(3000);          // timeout during which serial port probably connects.  Allows disconnected startup.
  //while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only
  Serial.println("Serial connection started");

  for (int i = 0; i < maxRow; i++) {
    printInfo[i][0] = '\0';
    messages[i][0] = '\0';
  }
}

// Prints the value on the given row number until it's updated
void Printer::printValue(int rowNumber, String value) {
  if (rowNumber >= maxRow || rowNumber < 0) {
    String errorMessage = "Row" + String(rowNumber) + "is an invalid row number! Try again.";
    printMessage(errorMessage,15);
  } else {
    value.toCharArray(printInfo[rowNumber],LONGEST_STRING);
  }
}

// prints messages, refreshes timer if error is already up
// messages are added to the bottom and move up
// if messageTime = 0, message will stay indefinitely
void Printer::printMessage(String message, int messageTime) {
  char newMessages[maxRow][LONGEST_STRING];
  char oldMessage[LONGEST_STRING];
  int newTimes[maxRow];
  int oldTime;
  int row;
  int newRow = 0;

  message.toCharArray(newMessages[newRow],LONGEST_STRING);
  newTimes[newRow] = messageTime;
  newRow++;

  for (row = 0; row < maxRow; row++) {
    strcpy(oldMessage,messages[row]); // store current row
    oldTime = messageTimes[row];

    if(oldMessage[0] != '\0' && newRow < maxRow) {
      strcpy(newMessages[newRow],oldMessage);
      newTimes[newRow] = oldTime;
      newRow++;
    }
  }
  while (newRow < maxRow) {
    newMessages[newRow][0] = '\0';
    newRow++;
  }

  for(int i=0; i<maxRow-1; i++) {
    for(int j=0; j<LONGEST_STRING-1; j++){
      messages[i][j]=newMessages[i][j];
    }
    messageTimes[i]=newTimes[i];
  }
  //std::copy(newMessages, newMessages+maxRow, messages);
  //std::copy(newTimes, newTimes+maxRow, messageTimes);
}

// actually displays the held information
void Printer::printToSerial(void) {
  int currentTime = millis();
  String timeString =
    String(currentTime/1000) + "." +
    String((currentTime%1000-currentTime%100)/100) +
    String((currentTime%100-currentTime%10)/10) +
    String(currentTime%10);

  // print extra space to clear
  for (int i=1; i<20; i++) {
    Serial.println("");
  }
  Serial.println("Messages at " + timeString + " seconds:");

  int messageTime;
  // print messages
  for (int row = maxRow-1; row >= 0; row--) {
    Serial.print("  ");
    Serial.println(messages[row]);
    messageTime = messageTimes[row];

    // deal with aging messages
    if (messageTime > 0) {
      messageTimes[row] = messageTime-1;
    }
    if (messageTime == 1) {
      messages[row][0] = '\0'; // termination character
    }
  }

  // print values with current time info
  Serial.println("");
  Serial.println("Values at " + timeString + " seconds:");
  // Serial.println(freeMemory());

  for (int row = 0; row < maxRow; row++) {
    Serial.print("  ");
    Serial.println(printInfo[row]);
  }
  Serial.flush();
}
