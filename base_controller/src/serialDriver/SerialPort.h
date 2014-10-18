#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <vector>
#include "Exception.h"

class SerialPort
{
public:
  SerialPort();
  SerialPort(const std::string& port, int baud);
  ~SerialPort();
  int write(const void* data, const int numBytes);
  int read(void* data, const int numBytes);
  bool isOpen() const;
  void setPort(const std::string& port, int baud);
  static void enumeratePorts(std::vector<std::string>& vec);
private:
    int fd;
};

class SerialPortException:public Exception
{
public:
  SerialPortException(const std::string& s);
  virtual ~SerialPortException();
};

#endif
