#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

std::string drive_topic = "/pid_drive";

float start_angle = M_PI / 2; //Point that we deem to be on the wall we want to follow in any case
float wall_jump_thresh = 0.8; //m

float lookahead_distance_offset = -0.1;
float lookahead_distance_speed_factor = 0.5;
float minimal_lookahead = 0.4;

float speed_limit = 2.5; // m/s
float speed_factor = 1;  // 1/s

float d_goal = 0.5; //m

float lidar_hardcrop = 120 * M_PI / 180;

float kp_kd_factor = 0.1;
float last_err = 0;

float control_frequency = 100;

sensor_msgs::LaserScan latest_scan;
void lidar_update(sensor_msgs::LaserScan msg)
{
    latest_scan = msg;
}

nav_msgs::Odometry latest_odom;
void odom_update(nav_msgs::Odometry msg)
{
    latest_odom = msg;
}

void speed_limit_update(std_msgs::Float32 msg)
{
    speed_limit = msg.data;
}

void speed_factor_update(std_msgs::Float32 msg)
{
    speed_factor = msg.data;
}

void ld_offset_update(std_msgs::Float32 msg)
{
    lookahead_distance_offset = msg.data;
}

void ld_speed_f_update(std_msgs::Float32 msg)
{
    lookahead_distance_speed_factor = msg.data;
}

void minimal_ld_update(std_msgs::Float32 msg)
{
    minimal_lookahead = msg.data;
}

void kp_kd_update(std_msgs::Float32 msg)
{
    kp_kd_factor = msg.data;
}

float idx_to_angle(int idx, const sensor_msgs::LaserScan &msg)
{
    return (msg.angle_min + idx * msg.angle_increment);
}

float idx_to_angle_deg(int idx, const sensor_msgs::LaserScan &msg)
{
    return idx_to_angle(idx, msg) * 180 / M_PI;
}

float distance_at_lookahead(float current_distance, float measurement_angle, float lookahead_distance)
{
    return std::sqrt(std::pow(lookahead_distance, 2) + std::pow(current_distance, 2) - 2 * lookahead_distance * current_distance * std::cos(measurement_angle));
}

float calculate_lookahead(float v, float max_lookahead)
{
    float speed_based_lookahead = lookahead_distance_offset + lookahead_distance_speed_factor * v;
    return std::max(minimal_lookahead,
                    std::min(speed_based_lookahead,
                             max_lookahead));
    // return std::max(minimal_lookahead, speed_based_lookahead);
}

void find_wall_idx(const std::vector<float> &ranges, int start_idx, int pos_end_idx, int neg_end_idx, float wall_jump_thresh, int &wall_start_idx, int &wall_end_idx)
{
    float d_i_minus_1 = ranges[start_idx];

    for (int i = start_idx; i <= pos_end_idx; i++)
    {
        float d_curr = latest_scan.ranges[i];

        // Make sure we stay on the same wall segment
        // Has to be done based on current lidar distance,
        // as things get complicated when deducing the threshold for a lookahead point
        if (std::abs(d_i_minus_1 - d_curr) > wall_jump_thresh)
        {
            break;
        }
        wall_end_idx = i;
        d_i_minus_1 = d_curr;
    }

    //Redo the same as above, just other direction
    d_i_minus_1 = latest_scan.ranges[start_idx];

    for (int i = start_idx; i >= neg_end_idx; i--)
    {
        float d_curr = latest_scan.ranges[i];
        if (std::abs(d_i_minus_1 - d_curr) > wall_jump_thresh)
        {
            break;
        }
        wall_start_idx = i;
        d_i_minus_1 = d_curr;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle node;

    ros::Publisher drive_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    ros::Publisher d_min_curr_pub = node.advertise<std_msgs::Float32>("/controller/d_min_curr", 10);
    ros::Publisher d_min_curr_angle_pub = node.advertise<std_msgs::Float32>("/controller/d_min_curr_angle_deg", 10);
    ros::Publisher d_min_lookahead_pub = node.advertise<std_msgs::Float32>("/controller/d_min_lookahead", 10);
    ros::Publisher d_min_lookahead_angle_pub = node.advertise<std_msgs::Float32>("/controller/d_min_lookahead_angle_deg", 10);
    ros::Publisher wall_start_pub = node.advertise<std_msgs::Float32>("/controller/wall_start_angle_deg", 10);
    ros::Publisher wall_end_pub = node.advertise<std_msgs::Float32>("/controller/wall_end_angle_deg", 10);
    ros::Publisher lookahead_pub = node.advertise<std_msgs::Float32>("/controller/lookahead_distance", 10);
    ros::Publisher max_lookahead_pub = node.advertise<std_msgs::Float32>("/controller/max_lookahead", 10);
    ros::Publisher err_pub = node.advertise<std_msgs::Float32>("/controller/err", 10);

    ros::Subscriber sub = node.subscribe("/scan", 1, lidar_update);
    ros::Subscriber sub_odom = node.subscribe("/odometry/filtered", 1, odom_update);
    ros::Subscriber sub_speed_lim = node.subscribe("/controller/speed_limit", 1, speed_limit_update);
    ros::Subscriber sub_speed_factor = node.subscribe("/controller/speed_factor", 1, speed_factor_update);
    ros::Subscriber sub_ldo = node.subscribe("/controller/lookahead_distance_offset", 1, ld_offset_update);
    ros::Subscriber sub_ldf = node.subscribe("/controller/lookahead_distance_speed_factor", 1, ld_speed_f_update);
    ros::Subscriber sub_min_ld = node.subscribe("/controller/min_lookahead", 1, minimal_ld_update);
    ros::Subscriber sub_kpkd = node.subscribe("/controller/kp_kd_factor", 1, kp_kd_update);
    ros::Rate loop_rate(control_frequency);

    while (ros::ok())
    {
        ros::spinOnce();

        if (latest_scan.angle_increment == 0)
        {
            //still in boot-up phase
            loop_rate.sleep();
            continue;
        }

        int start_idx = (start_angle - latest_scan.angle_min) / latest_scan.angle_increment;
        int pos_end_idx = std::min((int)latest_scan.ranges.size() - 1,
                                   (int)((lidar_hardcrop - latest_scan.angle_min) / latest_scan.angle_increment));
        int neg_end_idx = std::max(0,
                                   (int)((-lidar_hardcrop - latest_scan.angle_min) / latest_scan.angle_increment));

        int zero_idx = (0 - latest_scan.angle_min) / latest_scan.angle_increment;
        float d_0 = latest_scan.ranges[zero_idx];

        float d_min_lookhead = latest_scan.range_max;
        int d_min_lookahead_idx = -1;
        float d_min_curr = latest_scan.range_max;
        int d_min_curr_idx = -1;

        int wall_start_idx;
        int wall_end_idx;
        find_wall_idx(latest_scan.ranges, start_idx, pos_end_idx, neg_end_idx, wall_jump_thresh, wall_start_idx, wall_end_idx);

        // based on the lidar readings:
        // we can't look areound a corner so we shouldn't pretend to do so
        // by having a longer lookahead distance than the edge of the corner that we see
        float max_lookahead = std::cos(idx_to_angle(wall_start_idx, latest_scan)) * latest_scan.ranges[wall_start_idx];
        float v = latest_odom.twist.twist.linear.x;
        float lookahead_distance = calculate_lookahead(v, std::min(max_lookahead, d_0));

        for (int i = wall_start_idx; i <= wall_end_idx; i++)
        {
            float d_curr = latest_scan.ranges[i];
            float d_lookahead = distance_at_lookahead(d_curr, idx_to_angle(i, latest_scan), lookahead_distance);

            // Find the minimal distance from the lookahead point
            if (d_lookahead < d_min_lookhead)
            {
                d_min_lookhead = d_lookahead;
                d_min_lookahead_idx = i;
            }

            // For evaluation reasons find current shortest distance
            if (d_curr < d_min_curr)
            {
                d_min_curr = d_curr;
                d_min_curr_idx = i;
            }
        }

        float err = d_min_lookhead - d_goal;
        float ddt_err = (err - last_err) * control_frequency;

        // Result of several linearizations
        float K_P = 1 / lookahead_distance;
        float K_I = 0;
        float K_D = kp_kd_factor * K_P;

        ackermann_msgs::AckermannDriveStamped msg;
        msg.drive.speed = std::min(speed_limit, speed_factor * d_0);
        msg.drive.steering_angle = K_P * err + K_D * ddt_err;
        msg.header.stamp = ros::Time::now();
        drive_pub.publish(msg);

        std_msgs::Float32 d_min_lookahead_msg;
        d_min_lookahead_msg.data = d_min_lookhead;
        d_min_lookahead_pub.publish(d_min_lookahead_msg);
        std_msgs::Float32 d_min_lookahead_angle_msg;
        d_min_lookahead_angle_msg.data = idx_to_angle_deg(d_min_lookahead_idx, latest_scan);
        d_min_lookahead_angle_pub.publish(d_min_lookahead_angle_msg);

        std_msgs::Float32 d_min_curr_msg;
        d_min_curr_msg.data = d_min_curr;
        d_min_curr_pub.publish(d_min_curr_msg);
        std_msgs::Float32 d_min_curr_angle_msg;
        d_min_curr_angle_msg.data = idx_to_angle_deg(d_min_curr_idx, latest_scan);
        d_min_curr_angle_pub.publish(d_min_curr_angle_msg);

        std_msgs::Float32 wall_start_msg;
        wall_start_msg.data = idx_to_angle_deg(wall_start_idx, latest_scan);
        wall_start_pub.publish(wall_start_msg);
        std_msgs::Float32 wall_end_msg;
        wall_end_msg.data = idx_to_angle_deg(wall_end_idx, latest_scan);
        wall_end_pub.publish(wall_end_msg);

        std_msgs::Float32 ld_msg;
        ld_msg.data = lookahead_distance;
        lookahead_pub.publish(ld_msg);
        std_msgs::Float32 mld_msg;
        mld_msg.data = max_lookahead;
        max_lookahead_pub.publish(mld_msg);

        std_msgs::Float32 err_msg;
        err_msg.data = err;
        err_pub.publish(err_msg);

        loop_rate.sleep();
    }

    return 0;
}