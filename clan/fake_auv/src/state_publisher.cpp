#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <bbauv_msgs/manual_control.h>
#include <math.h>

using namespace std;

float x,y,z,yaw;

void transform (const bbauv_msgs::manual_control::ConstPtr& msg){
  x = msg->x;
  y = msg->y;
  z = msg->z;
  yaw = -msg->yaw;
  //ROS_INFO("x = %f, y= %f, z=%f, yaw= %f",x,y,z,yaw);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    //ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    //sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    const double degree= M_PI/180;
    double angle;
    angle = -M_PI/2;
    ros::Subscriber sub=n.subscribe<bbauv_msgs::manual_control>("monitor_controller",1000,transform);

    while (ros::ok()) {
        //update joint_state
        //joint_state.header.stamp = ros::Time::now();

        // update transform
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x += x*0.05*cos(angle+M_PI/2);
        odom_trans.transform.translation.y += x*0.05*sin(angle+M_PI/2);
        odom_trans.transform.translation.z += z*0.05;
	angle+=yaw*degree;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        //joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);
	ROS_INFO("x = %f, y= %f, z=%f, angle= %f",odom_trans.transform.translation.x,odom_trans.transform.translation.y,odom_trans.transform.translation.z,angle);
	ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}



