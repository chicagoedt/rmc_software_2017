#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>
#include "Exception.h"

class Runnable
{
public:
  virtual void run() = 0;
  virtual void requestStop() = 0;
};

class Thread
{
public:
  Thread(Runnable& r);
  ~Thread();
  void start();
  void join();
private:
    Runnable& runnable;
    pthread_t thread;
    bool wasStarted;
};

class ThreadException:public Exception
{
public:
  ThreadException(const std::string& s);
  virtual ~ThreadException();
};


#endif
