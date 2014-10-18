#ifndef REMOTECONSOLE_H
#define REMOTECONSOLE_H

#include <string>
#include <list>
#include "Thread.h"
#include "Mutex.h"
#include "Exception.h"

class RemoteConsoleServer:public Runnable
{
public:
  RemoteConsoleServer(const int port);
  ~RemoteConsoleServer();
  void run();
  void requestStop();
  void println(const std::string& s);
private:
  void addStream(const int socket);
  static std::string getTimeStamp();
  volatile bool stop;
  Mutex mutex;
  const int myPort;
  std::list<int> sockets;
};

class RemoteConsoleException:public Exception
{
public:
  RemoteConsoleException(const std::string& s);
  virtual ~RemoteConsoleException();
};

#endif
