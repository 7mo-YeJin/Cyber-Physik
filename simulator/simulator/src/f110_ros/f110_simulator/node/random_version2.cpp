#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <stdlib.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>


//                       0:          1:          2:          3:          4:
// float mapping[5][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 0.0}, {0.0, 0.0}};
// pi
const double PI = 3.1415;
float speed_limit = 1.8;
float angle_limit = 0.3;
float view_ang;
float largest_dist,largest_direction,smallst_direction, smallst_dist=50, gain;
int   length,loop=1, alpha_max,alpha_min, max_loop, min_loop;
bool  adjust_ang=false;

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
	
	view_ang= msg->angle_max-msg->angle_min;
	// ROS_INFO("-------%f---------", view_ang);
	length=ceil(view_ang/msg->angle_increment);
	// ROS_INFO("length is-----%d-------/n",length);
	
	 for (loop=floor(1.0/4*length)+1;loop<=floor(3.0/4*length)+1;loop++){
		if (largest_dist < msg->ranges[loop]) {
		largest_dist=msg->ranges[loop];
		max_loop=loop;
	    }
	    
	    if (smallst_dist > msg->ranges[loop]){
	    smallst_dist=msg->ranges[loop];
	    min_loop=loop;    
	    }
	 }
		
		if (smallst_dist<0.7) {
			adjust_ang=true;
			ROS_INFO("---------!!!!adjust angle!!!!---------");
			gain=1.0/smallst_dist;
			alpha_max=-gain*(max_loop-floor(1.0/2*length)); // direction can be changed
	        largest_direction=msg->angle_increment*alpha_max;
	        // reset
	        largest_dist=0; 
	        max_loop=0;
	        min_loop=0;
	        smallst_dist=50; 
		} else{
	           alpha_max=max_loop-floor(1.0/2*length); // direction can be changed
	           largest_direction=msg->angle_increment*alpha_max;
	           // reset
	           largest_dist=0; 
	           max_loop=0;
	           min_loop=0;
	           ROS_INFO("largest direction is     %f-------/n",largest_direction);
	           }
	 
     // ROS_INFO("angle_min is ----%f-------/n",msg->angle_min);
     // ROS_INFO("angle_max is ----%f-------",msg->angle_max);
	 
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "random");

  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);
  ros::Subscriber scan = n.subscribe("scan", 10, callback_scan);
  
  //n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
  //ROS_INFO("-------%f")
  

  float speed = 0.0;
  float angle = 0.0;
  srand(time(NULL));
  ros::Rate loop_rate(100);
  
  
  
  while(ros::ok()) {
    // unsigned index = rand()%5; 
    // speed = mapping[index][0];
    // angle = mapping[index][1];
    // ros::Subscriber scan = n.subscribe("scan", 10, callback_scan);
    speed = 1.0;
    angle = largest_direction;
   // ROS_INFO("-----%f-------",angle);
    
    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = angle * angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);
    ros::spinOnce(); // ask ROS if I received a message in "scan". If yes, I go in callback_scan. Otherwise, I go to the next line
    loop_rate.sleep();
  }
  return 0;
}
