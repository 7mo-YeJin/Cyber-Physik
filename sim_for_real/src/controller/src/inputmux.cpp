#define _GNU_SOURCE

#include <chrono>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// std ::string drive_topic = "/vesc/low_level/ackermann_cmd_mux/input/teleop";
std ::string drive_topic = "/drive";

ros::Publisher command_pub;

char mode = 'k';
auto operation_resume_time = std::chrono::system_clock::now();

void keyboard_callback(const ackermann_msgs::AckermannDriveStamped &msg)
{
    if (mode == 'k')
        command_pub.publish(msg);
};
void follow_gap_callback(const ackermann_msgs::AckermannDriveStamped &msg)
{
    if (mode == 'f')
        command_pub.publish(msg);
};
void pid_callback(const ackermann_msgs::AckermannDriveStamped &msg)
{
    if (mode == 'p')
        command_pub.publish(msg);
};
void mode_callback(const std_msgs::String &msg)
{
    if (msg.data.size() > 1)
        return;
    mode = msg.data.c_str()[0];
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "inputmux");

    ros::NodeHandle n;

    command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    ros::Subscriber k_sub = n.subscribe("/keyboard_drive", 10, keyboard_callback);
    ros::Subscriber f_sub = n.subscribe("/follow_gap_drive", 10, follow_gap_callback);
    ros::Subscriber aeb_sub = n.subscribe("/pid_drive", 10, pid_callback);
    ros::Subscriber m_sub = n.subscribe("/mode", 10, mode_callback);

    ros::spin();
}
