#include <iostream>
#include "OutputConsole.h"

using namespace std;

Mutex OutputConsole::mutex;

void OutputConsole::printErr(const std::string& s)
{
  try
  {
    mutex.lock();
    cerr << s;
    mutex.unlock();
  }
  catch (MutexException& e)
  {

  }
}

void OutputConsole::printOut(const std::string& s)
{
  try
  {
    mutex.lock();
    cerr << s;
    mutex.unlock();
  }
  catch (MutexException& e)
  {

  }
}

void OutputConsole::printlnErr(const std::string& s)
{
  try
  {
    mutex.lock();
    cerr << s << endl;
    mutex.unlock();
  }
  catch (MutexException& e)
  {

  }
}

void OutputConsole::printlnOut(const std::string& s)
{
  try
  {
    mutex.lock();
    cout << s << endl;
    mutex.unlock();
  }
  catch (MutexException& e)
  {

  }
}
