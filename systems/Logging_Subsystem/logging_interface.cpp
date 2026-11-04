#include <iostream>
#include <string>

class logging
{
public:
    virtual std::string loggable(int code, void *structPtr) = 0;
    virtual ~logging() = default;
};