#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/raw_magnetics.h>
#include <bbauv_msgs/imu_data.h>

#define DEFAULT_BAUDRATE 115200
#define GYRSR 107.892105

namespace sparton_imu
{

class Imu : public nodelet::Nodelet
{
    public:
        Imu() : running(true) {}
        ~Imu()
        {
            device->write("printtrigger 0 set drop\r\n");
            running = false;
            poller.join();
        }
    private:
        virtual void onInit()
        {
            getPrivateNodeHandle().param("port", device_port,
                                         std::string("/dev/ttyAHRS"));
            getPrivateNodeHandle().param("device", device_name,
                                         std::string("/AHRS8"));
            getPrivateNodeHandle().param("baud", baudrate,
                                         int(DEFAULT_BAUDRATE));

            rate = new ros::Rate(20);

            device = boost::make_shared<serial::Serial>();
            poller = boost::thread(boost::bind(&Imu::poll, this));

            imu_data_pub = getPrivateNodeHandle().advertise<sensor_msgs::Imu>
                ("/imu_data_q", 100);
            imu_serial_raw = getPrivateNodeHandle().advertise<std_msgs::String>
                ("/imu_serial_raw", 100);
            euler_out = getPrivateNodeHandle().advertise<bbauv_msgs::compass_data>
                ("/euler", 100);
            magnetics_out = getPrivateNodeHandle().advertise<bbauv_msgs::raw_magnetics>
                ("/magnetics_raw", 100);
            data_e_out = getPrivateNodeHandle().advertise<bbauv_msgs::imu_data>
                ("/imu_data_e", 100);
        }

        virtual void device_open()
        {
            ROS_INFO("%s %d", device_port.c_str(), baudrate);
            if(running){
                try{
                    device->close();
                    device->setPort(device_port);
                    device->setBaudrate(baudrate);
                    device->setTimeout(
                            serial::Timeout::max(), 250, 0, 250, 0
                        );
                    device->open();

                    std::stringstream ss;
                    ss << "printmask yawt_trigger roll_trigger or "
                        << "pitch_trigger or magp_trigger or temp_trigger "
                        << "or accelp_trigger or gyrop_trigger "
                        << "or set drop\r\n"
                        << "printmodulus 1 set drop\r\n"
                        << "printtrigger printmask set drop\r\n";

                    device->write(ss.str());
                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                } catch(std::exception &e){
                    ROS_ERROR("error opening RS232: %s", e.what());
                    ROS_INFO("waiting 1 second before trying again");
                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                    poll();
                }
            }
        }

        virtual void poll()
        {
            device_open();
            while(running){
                try{
                    if(device->available()){
                        buffer.append(device->read(device->available()));
                        std::string input = extract();
                        /*
                         * slow function
                         * @TODO: change
                         */
                        boost::erase_all(input, " ");
                        process(input);
                    }
                } catch(std::exception &e){
                    ROS_ERROR("error polling serial port: %s", e.what());
                    ROS_INFO("waiting 1 second before retrying");
                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                    poll();
                }
                rate->sleep();
            }
        }

        std::string extract()
        {
            size_t first = buffer.find("P:");
            size_t last  = buffer.rfind("\r\n");
            if(first == buffer.npos)
                return "";
            if(last == first)
                return "";

            std::string e = buffer.substr(first, last - first);
            buffer = buffer.substr(last, last - first);
            return e;
        }

        virtual void process(std::string data)
        {
            std::vector<std::string> p;
            std::vector<std::string> tok;
            boost::split(p, data, boost::is_any_of("\r\n"));
            for(auto &i : p){
                if(i.size()){
                    boost::split(tok, i, boost::is_any_of(","));
                    if(tok.size() != 22)
                        return;
                    serial_raw.data = i;
                    if(imu_serial_raw)
                        imu_serial_raw.publish(serial_raw);

                    imu_data.header.frame_id = "imu_frame";
                    imu_data.header.stamp = ros::Time::now();

                    float yaw = strtof(tok.at(15).c_str(), NULL);
                    float pitch = strtof(tok.at(19).c_str(), NULL);
                    float roll = strtof(tok.at(21).c_str(), NULL);
                    float temp = strtof(tok.at(17).c_str(), NULL);

                    float mx = strtof(tok.at(3).c_str(), NULL) * 1e-7;
                    float my = strtof(tok.at(4).c_str(), NULL) * 1e-7;
                    float mz = strtof(tok.at(5).c_str(), NULL) * 1e-7;

                    imu_data.linear_acceleration.x = strtof(tok.at(7).c_str(),
                                                            NULL) * -1e-3 * 9.80665;
                    imu_data.linear_acceleration.y = strtof(tok.at(8).c_str(),
                                                            NULL) * -1e-3 * 9.80665;
                    imu_data.linear_acceleration.z = strtof(tok.at(9).c_str(),
                                                            NULL) * -1e-3 * 9.80665;
                    imu_data.angular_velocity.x = strtof(tok.at(11).c_str(),
                                                         NULL) * 57.2957 * GYRSR;
                    imu_data.angular_velocity.y = strtof(tok.at(12).c_str(),
                                                         NULL) * 57.2957 * GYRSR;
                    imu_data.angular_velocity.z = strtof(tok.at(13).c_str(),
                                                         NULL) * 57.2957 * GYRSR;
                    imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                            radians(roll), radians(pitch), radians(yaw)
                        );
                    imu_data.orientation_covariance[0] =
                        imu_data.orientation_covariance[4] =
                        imu_data.orientation_covariance[8] = 0.01 * 0.01;
                    imu_data.angular_velocity_covariance[0] =
                        imu_data.angular_velocity_covariance[4] =
                        imu_data.angular_velocity_covariance[8] = 0.03 * 0.03;
                    imu_data.linear_acceleration_covariance[0] =
                        imu_data.linear_acceleration_covariance[4] =
                        imu_data.linear_acceleration_covariance[8] = 0.02 * 0.02;
                    if(imu_data_pub)
                        imu_data_pub.publish(imu_data);

                    euler.roll = roll;
                    euler.pitch = pitch;
                    euler.yaw = yaw;
                    euler.Az = imu_data.linear_acceleration.z;
                    euler.Ay = imu_data.linear_acceleration.y;
                    euler.Az = imu_data.linear_acceleration.z;
                    euler.temperature = temp;
                    euler.ang_vel_z = imu_data.angular_velocity.z;

                    if(euler_out)
                        euler_out.publish(euler);

                    mags.mags.x = mx;
                    mags.mags.y = my;
                    mags.mags.z = mz;

                    if(magnetics_out)
                        magnetics_out.publish(mags);

                    data_e.angular_velocity = imu_data.angular_velocity;
                    data_e.linear_acceleration = imu_data.linear_acceleration;
                    data_e.orientation.x = degrees(roll);
                    data_e.orientation.y = degrees(pitch);
                    data_e.orientation.z = degrees(yaw);
                    if(data_e_out)
                        data_e_out.publish(data_e);
                }
            }
        }

        inline float radians(float degrees)
        {
            return degrees * (M_PI / 180.0);
        }

        inline float degrees(float radians)
        {
            return radians * (180.0 / M_PI);
        }
        
        std::string device_port;
        std::string device_name;

        int baudrate;
        volatile bool running;
        boost::shared_ptr<serial::Serial> device;
        std::string buffer;
        boost::thread poller;
        ros::Rate *rate;
        sensor_msgs::Imu imu_data;
        std_msgs::String serial_raw;
        bbauv_msgs::compass_data euler;
        bbauv_msgs::raw_magnetics mags;
        bbauv_msgs::imu_data data_e;
        ros::Publisher imu_data_pub;
        ros::Publisher imu_serial_raw;
        ros::Publisher euler_out;
        ros::Publisher magnetics_out;
        ros::Publisher data_e_out;
};

PLUGINLIB_DECLARE_CLASS(sparton_imu, Imu, sparton_imu::Imu, nodelet::Nodelet)
}
