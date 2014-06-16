#include <boost/thread.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include <serial/serial.h>
#include <boost/foreach.hpp>
#include <bbauv_msgs/explorer_dvl_data.h>


namespace explorer_dvl
{
class DVL : public nodelet::Nodelet
{
public:
    DVL() : running(true) {}
    ~DVL()
    {
        running = false;
        poller.join();
    }
private:
    virtual void onInit()
    {
        
        dev = boost::make_shared<serial::Serial>();
        getPrivateNodeHandle().param("port", port,
                                     std::string("/dev/tty.usbserial-A602H7YO"));
        getPrivateNodeHandle().param("baud", baudrate, int(115200));
        rate = new ros::Rate(20);
        raw_pub = getPrivateNodeHandle().advertise<bbauv_msgs::explorer_dvl_data>
            ("/explorer_raw_data", 100);
        poller = boost::thread(boost::bind(&DVL::poll, this));

    }

    virtual void decode(bbauv_msgs::explorer_dvl_data &e)
    {
        std::vector<boost::uint8_t> ensemble;
        std::vector<double> corr(4, nan(""));
        ensemble.resize(4);
        if(!read(ensemble[0])) return;
        if(!read(ensemble[1])) return;
        if(!(ensemble[0] == 0x7F)) return;
        if(!(ensemble[1] == 0x7F)) return;

        if(!read(ensemble[2])) return;
        if(!read(ensemble[3])) return;
        e.header.stamp = ros::Time::now();
        e.header.frame_id = "explorer_dvl_raw";

        uint16_t ensemble_size = getu16le(ensemble.data() + 2);
        //ROS_INFO("ensemble_size: %d", ensemble_size);
        ensemble.resize(ensemble_size);
        for(int i = 4; i < ensemble_size; i++)
            read(ensemble[i]);
        
        uint16_t rchecksum;
        read_short(rchecksum);
        //ROS_INFO("checksum: %d", rchecksum);
        uint16_t checksum = 0;
        BOOST_FOREACH(uint16_t b, ensemble) checksum += b;
        if(checksum != rchecksum){
            ROS_ERROR("checksums dont match.");
            return;
        }

        if(ensemble.size() < 6) return;
        for(int i = 0; i < ensemble[5]; i++)
        {
            int offset = getu16le(ensemble.data() + 6 + 2 * i);
            if(ensemble.size() - offset < 2) continue;
            uint16_t section_id = getu16le(ensemble.data() + offset);
            if(section_id == 0x5803){
                //ROS_INFO("high res output found");
                if(ensemble.size() - offset < 2 + 4 * 4) continue;
                for(int i = 0; i < 4; i++){
                    int32_t velocity = gets32le(ensemble.data() + offset + 2 + 4 * i);
                    if(velocity == -32768){
                        ROS_INFO("dvl bad velocity on beam %i", i);
                        e.beam_velocity[i] = nan("");
                    } else{
                        double vel = -velocity * .01e-3;
                        e.beam_velocity[i] = vel;
                    }
                    //beam velocity
                }
            }
            else if(section_id == 0x0600){
                for(int i = 0; i < 4; i++){
                    e.beam_correlation[i] = *(ensemble.data() + offset + 32 + i);
                }
            }
            else if(section_id == 0x5804){
                if(ensemble.size() - offset < 2 + 4 * 3) continue;
                if(gets32le(ensemble.data() + offset + 10) <= 0){
                    ROS_INFO("no bottom range");
                }
                e.altitude = gets32le(ensemble.data() + offset + 10) * 0.1e-3;
            }

        }

    }
    
    bool read_short(uint16_t &x) {
        uint8_t low; if(!read(low)) return false;
        uint8_t high; if(!read(high)) return false;
        x = 256 * high + low;
        return true;
    }

    virtual void poll()
    {
        while(running){
            bbauv_msgs::explorer_dvl_data data;
            decode(data);
            if(raw_pub){
                raw_pub.publish(data);
            }
        }
    }

    virtual bool read(uint8_t &buff){
        try{
            if(!dev->isOpen())
                open();
            dev->read(&buff, sizeof(buff));
            return true;
        } catch(std::exception &e){
            ROS_ERROR("error reading byte: %s", e.what());
            boost::this_thread::sleep(boost::posix_time::seconds(1));
            open();
            return false;
        }
    }

    virtual void open()
    {
        try{
            dev->close();
            dev->setPort(port);
            dev->setBaudrate(baudrate);
            dev->setTimeout(
                    serial::Timeout::max(), 500, 0, 500, 0
                );
            dev->open();
        } catch(std::exception &e){
            ROS_ERROR("error opening serial port [%s]: %d",
                      port.c_str(), baudrate);
            ROS_ERROR("retrying in 1 second");
            boost::this_thread::sleep(boost::posix_time::seconds(1));
        }
    }

    static uint16_t getu16le(uint8_t* i) { return *i | (*(i+1) << 8); }
    static int32_t gets32le(uint8_t* i) {
        return *i | (*(i+1) << 8) | (*(i+2) << 16) | (*(i+3) << 24);
    }

    volatile bool running;
    std::string port;
    int baudrate;
    ros::Rate *rate;
    ros::Publisher raw_pub;
    boost::shared_ptr<serial::Serial> dev;
    boost::thread poller;
};
PLUGINLIB_DECLARE_CLASS(explorer_dvl, DVL, explorer_dvl::DVL, nodelet::Nodelet)
}
