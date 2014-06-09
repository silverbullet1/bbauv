#include <boost/thread.hpp>
#include <serial/serial.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <queue>

namespace imu
{
    struct sensor_data{
        double timestamp;
        ros::Time ros_timestamp;
        float magp[3];
        float accelp[3];
        float gyror[3];
        float gyrop[3];
        float yawt;
        float temperature;
        float pitch;
        float roll;
    };

    class IMU : public nodelet::Nodelet
    {
        public:
            IMU(){}
            ~IMU(){
                dev->write("printtrigger 0 set drop\r\n");
                dev_->interrupt();
                collect_->interrupt();
                dev_->join();
                collect_->join();
            }
        private:
            virtual void onInit();
            virtual void devicePoll();
            virtual void collect();
            virtual std::string extract();
            virtual void parse(sensor_data*, std::string);
            virtual void process();

            boost::shared_ptr<serial::Serial> dev;
            boost::shared_ptr<boost::thread> dev_;
            boost::shared_ptr<boost::thread> collect_;
            boost::shared_ptr<ros::Rate> rate;

            std::string buffer;
            std::queue<sensor_data> pqueue;
            double oldtime;

            ros::Publisher serial_out;
            ros::Publisher imu_data_out;
            ros::Publisher euler_out;
    };

    inline float radians(float degrees)
    {
        return degrees * (M_PI / 180.0);
    }

    inline float degrees(float radians)
    {
        return radians * (180.0 / M_PI);
    }

PLUGINLIB_DECLARE_CLASS(imu, IMU, imu::IMU, nodelet::Nodelet)
}
