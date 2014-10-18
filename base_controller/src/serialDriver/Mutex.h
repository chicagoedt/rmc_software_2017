#ifndef MUTEX_H
#define MUTEX_H

#include <pthread.h>
#include <string>
#include "Exception.h"

class Mutex
{
public:
  Mutex();
  ~Mutex();
  void lock();
  void unlock();
private:
    pthread_mutex_t mutex;
};

class MutexException:public Exception
{
public:
  MutexException(const std::string& s);
  virtual ~MutexException();
};

#endif
