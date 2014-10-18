#include <ctime>
#include <cstring>
#include <string>
#include <sstream>
#include "DataLogger.h"
#include "Time.h"

using namespace std;

DataLogger::DataLogger(const string& basename, const string& directory):
mutex(),
stream()
{

  ostringstream filename;
  filename << directory << "/" << basename << "_" << getTimeStampFile() << ".log";
  stream.open(filename.str().c_str());
  if (!stream.is_open() || !stream.good())
  {
    throw DataLoggerException("Couldn't open data logger file");
  }
}

string DataLogger::getTimeStampFile()
{
  tm timeinfo;
  memset(&timeinfo, 0, sizeof timeinfo);
  try
  {
    Time::getTimeStamp(timeinfo);
    int year =  timeinfo.tm_year;
    int month =  timeinfo.tm_mon;
    int day =  timeinfo.tm_mday;
    int hour =  timeinfo.tm_hour;
    int minute =  timeinfo.tm_min;
    int second =  timeinfo.tm_sec;
    ostringstream oss;
    oss.fill('0');
    oss.width(4);
    oss << year << "_";
    oss.width(2);
    oss << month << "_";
    oss.width(2);
    oss << day << "_";
    oss.width(2);
    oss << hour << "_";
    oss.width(2);
    oss << minute << "_";
    oss.width(2);
    oss << second;
    return oss.str();
  }
  catch (TimeException& e)
  {
    throw DataLoggerException(e.what());
  }
}

string DataLogger::getTimeStampLog()
{
  tm timeinfo;
  memset(&timeinfo, 0, sizeof timeinfo);
  try
  {
    Time::getTimeStamp(timeinfo);
    int year =  timeinfo.tm_year;
    int month =  timeinfo.tm_mon;
    int day =  timeinfo.tm_mday;
    int hour =  timeinfo.tm_hour;
    int minute =  timeinfo.tm_min;
    int second =  timeinfo.tm_sec;
    ostringstream oss;
    oss.fill('0');
    oss.width(4);
    oss << year << "-";
    oss.width(2);
    oss << month << "-";
    oss.width(2);
    oss << day << " ";
    oss.width(2);
    oss << hour << ":";
    oss.width(2);
    oss << minute << ":";
    oss.width(2);
    oss << second;
    return oss.str();
  }
  catch (TimeException& e)
  {
    throw DataLoggerException(e.what());
  }
}

bool DataLogger::isGood()
{
  try
  {
    mutex.lock();
    bool result = stream.is_open() && stream.good();
    mutex.unlock();
    return result;
  }
  catch (MutexException& e)
  {
    throw DataLoggerException(e.what());
  }
}


void DataLogger::println(const string& s)
{
  try
  {
    mutex.lock();
    if (stream.is_open() && stream.good()) //not using isGood() because that locks mutex and we would doublelock.
    {
      stream << this->getTimeStampLog() << "-> " << s << "\n";
      stream.flush();
      if (!stream.good())
      {
	mutex.unlock();
	throw DataLoggerException("Writing to output file failed");
      }
      mutex.unlock();
    }
    else
    {
      mutex.unlock();
      throw DataLoggerException("Output file is not good for writing()");
    }
  }
  catch (MutexException& e)
  {
    throw DataLoggerException(e.what());
  }
}

DataLogger::~DataLogger()
{
  if (stream.is_open())
  {
    stream.close();
  }
}

DataLoggerException::DataLoggerException(const string& s):
Exception(s)
{
}

DataLoggerException::~DataLoggerException()
{
}
