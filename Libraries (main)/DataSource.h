#ifndef __DATA_SOURCE_H__
#define __DATA_SOURCE_H__

#define LONGEST_CSV 100

#include <Arduino.h>

class DataSource {
public:
  // writes raw bytes of data to buffer, starting at index idx
  // returns the idx of the next vacant byte in the buffer
  virtual size_t writeDataBytes(unsigned char * buffer, size_t idx) = 0;

  // contains comma separated headings for each datavalue the dataSource reports
  const char * csvVarNames;
  const char * csvDataTypes;

protected:
  DataSource(const char varNames[], const char dataTypes[]) : csvVarNames(varNames), csvDataTypes(dataTypes) {}
};

#endif
