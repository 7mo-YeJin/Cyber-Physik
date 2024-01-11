#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "cstdlib" // generate random number

int main(int argc, char **argv) {
    ros::init(argc, argv, "emitter");
    ros::NodeHandle node;

    ros::Publisher odd = node.advertise<std_msgs::Bool>("odd", 10);
    ros::Rate loop_rate(2);
    
    ros::Publisher even = node.advertise<std_msgs::Bool>("even", 10);
    
    

    char message[50];
    std_msgs::Bool msg;
    srand(time(0));

    while (ros::ok()) {
		int random_number=rand();
		ROS_INFO("-------");
		
		
		bool is_odd=(random_number %2 !=0);
		
		msg.data= is_odd;
        ROS_INFO("%d", msg.data);
        odd.publish(msg);
        
        bool is_even=(random_number %2 ==0);
        msg.data= is_even;
        ROS_INFO("%d", msg.data);
        even.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

