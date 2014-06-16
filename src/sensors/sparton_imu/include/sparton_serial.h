#ifndef SPARTON_SERIAL
#define SPARTON_SERIAL
#include <string>
#include <serial/serial.h>

class Device{
public:
    Device(const std::string port, const int baudrate)
        : port(port), baudrate(baudrate) {};
    ~Device(){}
    virtual void open();
private:
    std::string port;
    int baudrate;
};

#endif /* SPARTON_SERIAL */
