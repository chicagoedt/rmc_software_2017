#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <fstream>
#include <string>
#include "Mutex.h"
#include "Exception.h"

class DataLogger
{
public:
    DataLogger(const std::string& basename, const std::string& directory);
    ~DataLogger();
    void println(const std::string& s);
    bool isGood();
private:
    static std::string getTimeStampFile();
    static std::string getTimeStampLog();
    Mutex mutex;
    std::ofstream stream;
};

class DataLoggerException:public Exception
{
public:
  DataLoggerException(const std::string& s);
  virtual ~DataLoggerException();
};

#endif
