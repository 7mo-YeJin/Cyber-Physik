#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

unsigned int counter;
unsigned int num;
float num_avg;
float i;
unsigned int num_geo=1;
unsigned int num_prod=1;
float num_geo_avg;
unsigned int num_har=1;
float num_sum;
float num_har_avg;


void callback_avg(const std_msgs::Int32::ConstPtr& msg) {
     counter=counter+1;   
     num=msg->data+num;
     num_avg=num/counter;
     ROS_INFO("avg is: %2f\n", num_avg);
}

void callback_geo_avg(const std_msgs::Int32::ConstPtr& msg) {
     i=i+1; 
     num_geo=msg->data;  
     num_prod *= num_geo;
     num_geo_avg=pow(num_prod,1/i);
     ROS_INFO("geo_avg is: %2f\n", num_geo_avg);
}

void callback_har_avg(const std_msgs::Int32::ConstPtr& msg) {
     counter=counter+1; 
     num_har=msg->data;  
     num_sum += 1/num_har;
     num_har_avg=counter/num_sum;
     ROS_INFO("harmonic avg is: %2f\n", num_har_avg);
}





int main(int argc, char **argv) {
	
    ros::init(argc, argv, "arithms_avg");
    ros::NodeHandle node;
    ros::Subscriber arith_avg = node.subscribe("user_integer", 10, callback_avg);
ros::Subscriber arith_geo_avg = node.subscribe("user_integer", 10,callback_geo_avg);
ros::Subscriber arith_har_avg = node.subscribe("user_integer", 10,callback_har_avg);
    ros::spin();
    return 0;
}
