#ifndef OUTPUTCONSOLE_H
#define OUTPUTCONSOLE_H

#include <string>
#include "Mutex.h"

class OutputConsole
{
public:
  static void printOut(const std::string& s);
  static void printErr(const std::string& s);
  static void printlnOut(const std::string& s = "");
  static void printlnErr(const std::string& s = "");
private:
    static Mutex mutex;
};

#endif
