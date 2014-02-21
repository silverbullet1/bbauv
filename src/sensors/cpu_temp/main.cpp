#include <stdio.h>
#include <ros/ros.h>
#include <bbauv_msgs/cpu_temperature.h>
#include <sensors/sensors.h>
#include <string>

#define NUMBER_OF_SENSORS 7 //socket and core +1

void get_temperature(bbauv_msgs::cpu_temperature *ct, double *data, FILE *logf)
{
    const sensors_chip_name *cn;
    int c = 0, s = 0, f = 0, counter = 0;
    double val;
    //double data[NUMBER_OF_SENSORS];

    while((cn = sensors_get_detected_chips(0, &c)) != 0)
    {
        const sensors_feature *features;
        while((features = sensors_get_features(cn, &f)) != 0)
        {
            const sensors_subfeature *subfeatures;
            while((subfeatures = sensors_get_all_subfeatures(cn, features, &s)) != 0)
            {
                if(subfeatures->flags & SENSORS_MODE_R)
                {
                    int rc = sensors_get_value(cn, subfeatures->number, &val);
                    data[counter] = val;
                    counter++;
                }
                break;
            }
        }
    }
    double acpi_ave = (data[0] + data[1]) / 2.0;
    double core_ave = (data[2] + data[2] + data[3]) / 3.0;
    ct->cores_ave = core_ave;
    ct->socket_ave = acpi_ave;
    fprintf(logf, "socket_average: %lf\ncore_ave: %lf\n-----", acpi_ave, core_ave);
}

int main(int argc, char **argv)
{
    const std::string cpu_temp_topic = "CPU_TEMP";
    double data[NUMBER_OF_SENSORS];
    bbauv_msgs::cpu_temperature cpu_temperature;
    ros::init(argc, argv, "cpu_temperature");

    FILE *logf  = fopen("/home/bbauvsbc1/cpu_temperature_log.txt", "w");

    ros::NodeHandle nodehandler;
    ros::Publisher publisher = nodehandler.advertise<bbauv_msgs::cpu_temperature>(cpu_temp_topic, 1000);
    ros::Rate loop_rate(0.2);

    //get_temperature(&cpu_temperature, data);
    sensors_init();

    while(ros::ok()){
        get_temperature(&cpu_temperature, data, logf);
        publisher.publish(cpu_temperature);
        loop_rate.sleep();
        ros::spinOnce();
    }

    fclose(logf);
    sensors_cleanup();

    return 0;
}
