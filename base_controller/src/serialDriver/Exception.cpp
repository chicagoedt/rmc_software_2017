#include "Exception.h"

Exception::Exception(const std::string& s):
str(s)
{
}

Exception::~Exception()
{
}


const std::string& Exception::what() const
{
  return str;
}
