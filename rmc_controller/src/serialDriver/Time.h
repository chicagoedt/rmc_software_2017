#ifndef TIME_H
#define TIME_H

#include <ctime>
#include "Mutex.h"
#include "Exception.h"

class Time
{
public:
  static void getTimeStamp(tm& t);
  static std::string getLoggerTimeStamp();
private:
    static Mutex mutex;
};

class TimeException:public Exception
{
public:
  TimeException(const std::string& s);
  virtual ~TimeException();
};

#endif
