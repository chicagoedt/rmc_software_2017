#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <string>

class Exception
{
public:
  Exception(const std::string& s);
  virtual ~Exception();
  virtual const std::string& what() const;
private:
  const std::string str;
};

#endif
