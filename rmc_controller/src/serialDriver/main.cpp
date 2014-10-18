#include <iostream>
#include "DataLogger.h"
#include "Thread.h"
#include "SerialPort.h"
#include <cstring>
#include <unistd.h>
#include "RemoteConsole.h"
#include "OutputConsole.h"

using namespace std;

class LogTest:public Runnable
{
public:
  LogTest(DataLogger& l, string s);
  void run();
  void requestStop();
private:
  std::string str;
  DataLogger& log;
};


int main()
{
  DataLogger d("x", "/home/akotadia/test");
  d.println("Line 1");
  d.println("Line 2");
  d.println("Line 3");

  LogTest lt1(d, "The quick brown fox jumps over the lazy dog");
  Thread t1(lt1);
  LogTest lt2(d, "Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor ");
  Thread t2(lt2);
  t1.start();
  t2.start();
  t1.join(); 
  t2.join();

  OutputConsole::printlnOut("Now it is serial time");

  try
  {
    SerialPort port("/dev/pts/5", 57600);
    
    port.write("Terrorist\r\n", 8);
    for (int i = 0; i < 300; ++i) //run for 30 seconds
    {
      usleep(100000);
      char cc[21];
      memset(cc, 0, 21);
      int read = port.read(&cc[0], 20);
      if (read > 0)
      {
	cout << cc << endl;
	d.println(cc);
      }
    }
  }
  catch (SerialPortException& e)
  {
    cout << "Had a serial port problem: " << e.what() << endl;
  }


  OutputConsole::printlnOut("Now it is network time");
  OutputConsole::printlnOut();
  RemoteConsoleServer server(7000);
  Thread t3(server);
  t3.start();
  sleep(5);
  server.println("Hello network");
  sleep(5);
    server.println("Hello Anup");
  sleep(5);
    server.println("Hello Terrorist");
  sleep(5);
    server.println("Hello, what's up?");
  sleep(5);
    server.println("How are you?");
  sleep(5);
    server.println("You need more fiber in your diet.");
  sleep(5);
  server.println("Hello network");
  sleep(5);
    server.println("Hello Anup");
  sleep(5);
    server.println("Hello Terrorist");
  sleep(5);
    server.println("Hello, what's up?");
  sleep(5);
    server.println("How are you?");
  sleep(5);
    server.println("You need more fiber in your diet.");
  sleep(5);
  server.requestStop();
  t3.join();
  return 0;
}

LogTest::LogTest(DataLogger& l, string s):
str(s),
log(l)
{
}

void LogTest::requestStop()
{
}

void LogTest::run()
{
  for (int i = 0; i < 1000; i++)
  {
    log.println(str);
  }
}
