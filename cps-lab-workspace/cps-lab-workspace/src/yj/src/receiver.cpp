#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

unsigned int i;
unsigned int j;

void callback_odd(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data)
    i=i+1;
    // ROS_INFO("Recv: %d\n", msg->data);
    ROS_INFO("num of odd: %d\n", i);
}

void callback_even(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data)
    j=j+1;
    // ROS_INFO("Recv: %d\n", msg->data);
    ROS_INFO("num of even: %d\n", j);
    ROS_INFO("-------------------");
}

int main(int argc, char **argv) {
	
    ros::init(argc, argv, "receiver");
    ros::NodeHandle node;
    ros::Subscriber odd = node.subscribe("odd", 10, callback_odd);
    ros::Subscriber even = node.subscribe("even", 10, callback_even);
    ros::spin();
    return 0;
}
