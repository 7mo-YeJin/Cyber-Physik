#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

float safety_shutdown_distance = 0.3; // m
std::vector<float> safety_shutdown_measurement_angles = {-30 * M_PI / 180, 0, 30 * M_PI / 180};
bool safety_shutdown = false;

sensor_msgs::LaserScan latest_scan;
void lidar_update(sensor_msgs::LaserScan msg)
{
    latest_scan = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aeb");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/scan", 1, lidar_update);

    ros::Publisher drive_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/keyboard_drive", 1);
    ros::Publisher mode_pub = node.advertise<std_msgs::String>("/mode", 1);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        if (latest_scan.ranges.size() > 0)
        {
            safety_shutdown = false;
            for (auto angle : safety_shutdown_measurement_angles)
            {
                int idx = std::max(0,
                                   std::min((int)latest_scan.ranges.size() - 1,
                                            (int)((angle - latest_scan.angle_min) / latest_scan.angle_increment)));
                if (latest_scan.ranges[idx] < safety_shutdown_distance)
                {
                    ROS_INFO("AEB engaged at angle %f, distance %f", angle * 180 / M_PI, latest_scan.ranges[idx]);
                    safety_shutdown = true;
                    break;
                }
            }
        }
        else
        {
            ROS_INFO("AEB engaged because no lidar data present");
            safety_shutdown = true;
        }

        if (safety_shutdown)
        {
            std_msgs::String mode_msg;
            mode_msg.data = "k";
            mode_pub.publish(mode_msg);

            ackermann_msgs::AckermannDriveStamped command_msg;
            command_msg.header.stamp = ros::Time::now();
            drive_pub.publish(command_msg);
        }

        loop_rate.sleep();
    }
}