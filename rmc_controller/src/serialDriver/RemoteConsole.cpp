#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstring>
#include <sstream>
#include <unistd.h>
#include "RemoteConsole.h"
#include "DataLogger.h"
#include "Time.h"
#include "OutputConsole.h"

using namespace std;

const int BACKLOG = 10; //max number of pending requests
const int ACCEPT_WAIT_TIME = 100000; //number of microseconds to block when checking for incoming sockets

RemoteConsoleServer::RemoteConsoleServer(const int port):
stop(false),
mutex(),
myPort(port)
{
}

RemoteConsoleServer::~RemoteConsoleServer()
{
  if (!sockets.empty())
  {
    for (std::list<int>::iterator itr = sockets.begin(); itr != sockets.end(); ++itr)
    {
      close(*itr);
    }
  }
}

void RemoteConsoleServer::run()
{
    sockaddr_storage their_addr;
    socklen_t addr_size;
    addrinfo hints, *res;
    int sockfd;

    // !! don't forget your error checking for these calls !!

    // first, load up address structs with getaddrinfo():

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

    ostringstream ss;
    ss << myPort;

    try
    {
      int i = getaddrinfo(NULL, ss.str().c_str(), &hints, &res);
      if (i != 0)
      {
	throw RemoteConsoleException("Can't get Address Info");
      }

      // make a socket, bind it, and listen on it:
      sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
      if (sockfd == -1)
      {
	throw RemoteConsoleException("Can't get Socket");
      }

      i = bind(sockfd, res->ai_addr, res->ai_addrlen);
      if (i != 0)
      {
	throw RemoteConsoleException("Can't bind to the port");
      }

      i = listen(sockfd, BACKLOG);
      if (i != 0)
      {
	throw RemoteConsoleException("Can't listen to the port");
      }

      while (!stop)
      {
	timeval tval;
	tval.tv_sec = 0;
	tval.tv_usec = ACCEPT_WAIT_TIME;

	fd_set master;    // master file descriptor list
	FD_ZERO(&master);
	FD_SET(sockfd, &master);
	i = select(sockfd+1, &master, NULL, NULL, &tval);
	if (i != -1 && FD_ISSET(sockfd, &master))
	{
	  addr_size = sizeof their_addr;
	  int client = accept(sockfd, (struct sockaddr *)&their_addr, &addr_size);
	  if (client != -1)
	  {
	    addStream(client);
	  }
	}
      }
      close(sockfd);
      freeaddrinfo(res); // free the linked-list
    }
    catch (RemoteConsoleException& e)
    {
      OutputConsole::printlnOut(e.what());
    }
}

void RemoteConsoleServer::requestStop()
{
  stop = true;
}


void RemoteConsoleServer::println(const string& s)
{
 try
  {
    mutex.lock();
    if (!sockets.empty())
    {
      ostringstream oss;
      oss << getTimeStamp() << "-> " << s << "\r\n";
      for (std::list<int>::iterator itr = sockets.begin(); itr != sockets.end();)
      {
	  int len = oss.str().length();
	  int socket = *itr;
	  int bytesSent = 0;
	  while (bytesSent < len)
	  {
	    int bytesSentNow = send(socket, oss.str().c_str(), (len - bytesSent), MSG_NOSIGNAL); //need the MSG_NOSIGNAL or program will crash when client disconnects
	    if (bytesSentNow <= 0)
	    {
	      break;
	    }
	    else
	    {
	      bytesSent += bytesSentNow;
	    }
	  }
	  if (bytesSent < len)
	  {
	      close(*itr);
	      itr = sockets.erase(itr);
	      OutputConsole::printlnOut("RemoteConsole lost a socket connection.");
	  }
	  else
	  {
	    ++itr;
	  }
      }
    }
    mutex.unlock();
  }
  catch (MutexException& e)
  {
    OutputConsole::printlnOut("Error with writing to remote console");
  }
  catch (RemoteConsoleException& e)
  {
    mutex.unlock();
    OutputConsole::printlnOut("Error with writing to remote console");
  }
}

void RemoteConsoleServer::addStream(const int socket)
{
  try
  {
    mutex.lock();
    sockets.push_back(socket);
    mutex.unlock();
  }
  catch (MutexException& e)
  {
    close(socket);
  }
}

string RemoteConsoleServer::getTimeStamp()
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
    throw RemoteConsoleException(e.what());
  }
}

RemoteConsoleException::RemoteConsoleException(const string& s):
Exception(s)
{
}

RemoteConsoleException::~RemoteConsoleException()
{
}
