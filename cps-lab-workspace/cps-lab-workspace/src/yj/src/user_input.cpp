#include "ros/ros.h"
#include "std_msgs/Int32.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "user_input");
    ros::NodeHandle node;

    ros::Publisher user_integer = node.advertise<std_msgs::Int32>("user_integer", 10);
    ros::Rate loop_rate(2);
    
    int message;
    std_msgs::Int32 msg;
    
    while (ros::ok()) {
        printf("Enter integer: ");
        scanf("%d", &message);
        msg.data = message;
        ROS_INFO("%d", msg.data);
        user_integer.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
