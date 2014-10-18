#include <pthread.h>
#include "Thread.h"

void* thread_wrapper(void *ptr)
{
  Runnable* r = (Runnable*) ptr;
  r->run();
  return 0;
}

Thread::Thread(Runnable& r):
runnable(r),
wasStarted(false)
{
}

Thread::~Thread()
{
}

void Thread::start()
{
  if (!wasStarted)
  {
    const int i = pthread_create(&thread, NULL, thread_wrapper, (void*) &runnable);
    if (i != 0)
    {
      throw ThreadException("Couldn't start thread");
    }
    else
    {
      wasStarted = true;
    }
  }
  else
  {
    throw ThreadException("Cant start thread. It is already started.");
  }
}

void Thread::join()
{
  if (wasStarted == true)
  {
    const int i = pthread_join(thread, NULL);
    if (i != 0)
    {
      throw ThreadException("Couldn't join thread.");
    }
  }
  else
  {
    throw ThreadException("Can't join thread. It hasn't been started.");
  }
}

ThreadException::ThreadException(const std::string& s):
Exception(s)
{
}

ThreadException::~ThreadException()
{
}
