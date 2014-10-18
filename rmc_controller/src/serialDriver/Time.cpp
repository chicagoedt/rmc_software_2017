#include <cstring>
#include <ctime>
#include "Time.h"

using namespace std;

Mutex Time::mutex;

const int EPOCH_YEAR = 1900;

void Time::getTimeStamp(tm& t)
{
  try
  {
    mutex.lock();
    const time_t time_now = time(NULL);
    if (time_now == -1)
    {
      mutex.unlock();
      throw TimeException("Cannot get local time");
    }
    else
    {
      tm* timeinfo = localtime(&time_now);
      memcpy(&t, timeinfo, sizeof(tm));
      t.tm_year += EPOCH_YEAR;
      t.tm_mon += 1;
    }
    mutex.unlock();
  }
  catch (MutexException& e)
  {
    throw TimeException(e.what());
  }
}

TimeException::TimeException(const std::string& s):
Exception(s)
{
}

TimeException::~TimeException()
{
}
