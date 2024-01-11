#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <iostream>
#include <termios.h>

ros::Publisher key_pub;
ros::Publisher Ftg_pub;

char message;

ackermann_msgs::AckermannDriveStamped Keyboard_drive_st_msg;
ackermann_msgs::AckermannDriveStamped Follow_gap_drive_st_msg;

void callback_key(const ackermann_msgs::AckermannDriveStamped & msg) {
Keyboard_drive_st_msg.header=msg.header;
Keyboard_drive_st_msg.drive=msg.drive;
}

void callback_ftg(const ackermann_msgs::AckermannDriveStamped & msg) {
Follow_gap_drive_st_msg.header=msg.header;
Follow_gap_drive_st_msg.drive=msg.drive;
}

void callback_mode(const std_msgs::String::ConstPtr& msg) {
	if (msg->data == "k") 
    message='k';
    else if (msg->data == "f") 
    message='f';
}


int main(int argc, char **argv) {
	
    ros::init(argc, argv, "mux");
    ros::NodeHandle node;
    ros::Subscriber mode = node.subscribe("mode", 10, callback_mode);
    // ros::Subscriber follow_the_gap_drive = node.subscribe("Follow_gap_drive", 10, callback_ftg);
    ros::Subscriber follow_the_gap_drive = node.subscribe("Test_drive", 10, callback_ftg);
    ros::Subscriber keyboard_drive = node.subscribe("Keyboard_drive", 10, callback_key);
    key_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);
    Ftg_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);
 
 ros::Rate loop_rate(150);   
 while(ros::ok()) {
    if (message== 'k')
    key_pub.publish(Keyboard_drive_st_msg); 
    if (message== 'f')
    Ftg_pub.publish(Follow_gap_drive_st_msg);        

    ros::spinOnce(); 
    loop_rate.sleep();
 }
    return 0;
}
