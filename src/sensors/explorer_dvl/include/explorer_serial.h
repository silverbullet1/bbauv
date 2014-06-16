#ifndef EXPLORER_SERIAL_H
#define EXPLORER_SERIAL_H

#include <boost/thread.hpp>
#include <string>
#include <serial/serial.h>


class Device{
    public:
        Device(std::string port, int baudrate) :
            port(port), baudrate(baudrate)
    {}
        ~Device(){}
    private:
        const std::string port;
        int baudrate;
        serial::Serial dev;

        void open()
        {
            try{
                dev.close();
                dev.setPort(port);
                dev.setBaudrate(baudrate);
                dev.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
                dev.open();
                return;
            } catch(std::exception &e){
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            }
        }

    public:
        void read_byte(uint8_t &b)
        {
            try{
                dev.read(&b, sizeof(b));
            } catch(std::exception &e){
                boost::this_thread::sleep(boost::posix_time::seconds(1));
                open();
            }
        }

};

#endif /* EXPLORER_SERIAL_H */
