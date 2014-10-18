#include <pthread.h>
#include "Mutex.h"

void Mutex::lock()
{
  const int i = pthread_mutex_lock(&mutex);
  if (i != 0)
  {
    throw MutexException("Couldn't lock Mutex");
  }
}

void Mutex::unlock()
{
  const int i = pthread_mutex_unlock(&mutex);
  if (i != 0)
  {
    throw MutexException("Couldn't unlock Mutex");
  }
}

Mutex::Mutex()
{
  const int i = pthread_mutex_init(&mutex, NULL);
  if (i != 0)
  {
    throw MutexException("Couldn't create Mutex");
  }
}


Mutex::~Mutex()
{
  pthread_mutex_destroy(&mutex);
}

MutexException::MutexException(const std::string& s):
Exception(s)
{
}

MutexException::~MutexException()
{
}
