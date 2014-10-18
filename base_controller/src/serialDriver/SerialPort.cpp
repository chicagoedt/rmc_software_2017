#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "SerialPort.h"
#include <iostream>
using namespace std;
SerialPort::SerialPort(const string& port, int baud)
{


    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    // cout << fd << endl;
    if (!isOpen())
    {
      throw SerialPortException("Could not open Serial Port");
    }

    termios options;
    speed_t speed;

    switch (baud) //switch statement selects baud rate. if unsupported baud rate is chosen, port will be closed.
    {
      case 50:
	speed = B50;
	break;
      case 75:
	speed = B75;
	break;
      case 110:
	speed = B110;
	break;
      case 134:
	speed = B134;
	break;
      case 150:
	speed = B150;
	break;
      case 200:
	speed = B200;
	break;
      case 300:
	speed = B300;
	break;
      case 600:
	speed = B600;
	break;
      case 1200:
	speed = B1200;
	break;
      case 1800:
	speed = B1800;
	break;
      case 2400:
	speed = B2400;
	break;
      case 4800:
	speed = B4800;
	break;
      case 9600:
	speed = B9600;
	break;
      case 19200:
	speed = B19200;
	break;
      case 38400:
	speed = B38400;
	break;
      case 57600:
	speed = B57600;
	break;
      case 115200:
	speed = B115200;
	break;
      case 230400:
	speed = B230400;
	break;
      default:
	close(fd);
	fd = -1;
	throw SerialPortException("Invalid Serial Port baud rate chosen");
	break;
    }

    int i = tcgetattr(fd, &options);
    if (i != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Could not get serial port parameters.");
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcflag_t c_iflag_clr = IXON | IXOFF | IXANY | ICRNL | IGNCR | INLCR | ISTRIP | INPCK | PARMRK | IGNPAR | BRKINT | IGNBRK;
    tcflag_t c_iflag_set = 0;
    tcflag_t c_iflag_exact_msk = 0;
    tcflag_t c_iflag_exact = 0;

    tcflag_t c_oflag_clr = OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL;
    tcflag_t c_oflag_set = 0;
    tcflag_t c_oflag_exact_msk = 0;
    tcflag_t c_oflag_exact = 0;

    tcflag_t c_cflag_clr = PARENB | CSTOPB | CRTSCTS | PARODD | HUPCL;
    tcflag_t c_cflag_set = CREAD | CLOCAL;
    tcflag_t c_cflag_exact_msk = CSIZE;
    tcflag_t c_cflag_exact = CS8;

    tcflag_t c_lflag_clr = ICANON | ECHO | ECHOE | ISIG;
    tcflag_t c_lflag_set = 0;
    tcflag_t c_lflag_exact_msk = 0;
    tcflag_t c_lflag_exact = 0;

    options.c_iflag &= ~c_iflag_clr;
    options.c_iflag |= c_iflag_set;
    options.c_iflag &= ~c_iflag_exact_msk;
    options.c_iflag |= c_iflag_exact;

    options.c_oflag &= ~c_oflag_clr;
    options.c_oflag |= c_oflag_set;
    options.c_oflag &= ~c_oflag_exact_msk;
    options.c_oflag |= c_oflag_exact;

    options.c_cflag &= ~c_cflag_clr;
    options.c_cflag |= c_cflag_set;
    options.c_cflag &= ~c_cflag_exact_msk;
    options.c_cflag |= c_cflag_exact;

    options.c_lflag &= ~c_lflag_clr;
    options.c_lflag |= c_lflag_set;
    options.c_lflag &= ~c_lflag_exact_msk;
    options.c_lflag |= c_lflag_exact;

    i = tcsetattr(fd, TCSANOW, &options);
    if (i != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Could not set serial port parameters.");
    }

    //verify all of our settings were done;
    i = tcgetattr(fd, &options);
    if (i != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Could not get serial port parameters.");
    }

    if (speed != cfgetispeed(&options))
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Input baud rate set wrong.");
    }
    if (speed != cfgetospeed(&options))
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Output baud rate set wrong.");
    }

    if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 0))
    {
      close(fd);
      fd = -1;
      throw SerialPortException("VTIME/VMIN set wrong.");
    }

    //----------------------------------------

    if ((options.c_iflag & c_iflag_clr) != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_iflag set wrong.");
    }
    if ((options.c_iflag & c_iflag_set) != c_iflag_set)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_iflag set wrong.");
    }
    if ((options.c_iflag & c_iflag_exact_msk) != c_iflag_exact)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_iflag set wrong.");
    }

    //----------------------------------------------

    if ((options.c_oflag & c_oflag_clr) != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_oflag set wrong.");
    }
    if ((options.c_oflag & c_oflag_set) != c_oflag_set)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_oflag set wrong.");
    }
    if ((options.c_oflag & c_oflag_exact_msk) != c_oflag_exact)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_oflag set wrong.");
    }

    //-------------------------------------------------

    if ((options.c_cflag & c_cflag_clr) != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_cflag set wrong.");
    }
    if ((options.c_cflag & c_cflag_set) != c_cflag_set)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_cflag set wrong.");
    }
    if ((options.c_cflag & c_cflag_exact_msk) != c_cflag_exact)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_cflag set wrong.");
    }

    //--------------------------------------------------

    if ((options.c_lflag & c_lflag_clr) != 0)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_lflag set wrong.");
    }
    if ((options.c_lflag & c_lflag_set) != c_lflag_set)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_lflag set wrong.");
    }
    if ((options.c_lflag & c_lflag_exact_msk) != c_lflag_exact)
    {
      close(fd);
      fd = -1;
      throw SerialPortException("Serial Port c_lflag set wrong.");
    }
}


int SerialPort::write(const void* data, const int numBytes )
{
  int result = 0;
  if (isOpen())
  {
    result = ::write(fd, data, numBytes);
    //cout << result << endl;
    if (result < 0)
    {
      throw SerialPortException("Error writing to Serial Port.");
    }
  }
  else
  {
    throw SerialPortException("Can't write Serial Port. Port is not open.");
  }
  return result;
}

int SerialPort::read(void* data, const int numBytes )
{
  int result = 0;
  if (isOpen())
  {
    // cout << fd << "\n"<< endl;
    result = ::read(fd, data, numBytes);
 
    // cout << result << endl;
    if (result < 0)
    {
      throw SerialPortException("Error reading from Serial Port.");
    }
  }
  else
  {
    throw SerialPortException("Can't read Serial Port. Port is not open.");
  }
  return result;
}

SerialPort::~SerialPort()
{
  if (isOpen())
  {
    close(fd);
  }
}

bool SerialPort::isOpen() const
{
  return (fd != -1);
}

void SerialPort::enumeratePorts(vector<string>& vec)
{
  vec.clear();
  //TODO Actually write the code to enumerate the serial ports and put them in vec.

}


SerialPortException::SerialPortException(const string& s):
Exception(s)
{
}

SerialPortException::~SerialPortException()
{
}
