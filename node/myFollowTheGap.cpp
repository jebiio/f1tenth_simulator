#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

// TODO: include ROS msg type headers and libraries
#include "f1tenth_simulator/precompute.hpp"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Bool.h>

using namespace racecar_simulator;

class Safety
{
  // The class that handles emergency braking
private:
  ros::NodeHandle n;
  double speed;
  double prev_angle = 0.0; // previous desired steering angle
  double error = 0;
  const double DESIRED_IDSTANCE = 0.5;
  const double KP = 1;
  double steering_angle = 0;
  std::vector<double> cosines; // precompute cosines of scan angles
  std::vector<double> car_distances; // precompute distance from lidar to edge of car for each beam

  // Sub.s
  ros::Subscriber odom_sub;
  ros::Subscriber laser_sub;

  // Pub.s
  ros::Publisher drive_pub;

public:
  Safety()
  {
    n = ros::NodeHandle();
    speed = 0.0;

    // Initialize the node handle
    n = ros::NodeHandle("~");

    // get topic names
    std::string drive_topic, odom_topic, scan_topic;

    n.getParam("new_drive_topic", drive_topic);
    n.getParam("odom_topic", odom_topic);
    n.getParam("scan_topic", scan_topic);

    // get car parameters
    // n.getParam("max_speed", max_speed);
    // n.getParam("max_steering_angle", max_steering_angle);

    // Get params for precomputation and collision detection
    int scan_beams;
    double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
    // n.getParam("ttc_threshold", ttc_threshold);
    n.getParam("scan_beams", scan_beams);
    n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
    n.getParam("width", width);
    n.getParam("wheelbase", wheelbase);
    n.getParam("scan_field_of_view", scan_fov);
    scan_ang_incr = scan_fov / scan_beams;

    // Precompute cosine and distance to car at each angle of the laser scan

    cosines = Precompute::get_cosines(scan_beams, -scan_fov / 2.0, scan_ang_incr);
    car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width,
                                                  scan_distance_to_base_link, -scan_fov / 2.0, scan_ang_incr);
    // Make a publisher for drive messages
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

    // Start a subscriber to listen to odom messages
    laser_sub = n.subscribe(scan_topic, 1, &Safety::scan_callback, this);
    odom_sub = n.subscribe(odom_topic, 1, &Safety::odom_callback, this);
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
  {
    speed = odom_msg->twist.twist.linear.x;
  }

  void scan_callback(const sensor_msgs::LaserScan &scan_msg)
  {
    unsigned int radius = 25;
    // find index_min
    std::vector<float> ranges = scan_msg.ranges;
    double range_min = scan_msg.ranges[scan_msg.ranges.size()/3];
    unsigned int index_min = 0;
    for (unsigned int i = scan_msg.ranges.size()/3; i < 2*scan_msg.ranges.size()/3; i++)
    {
      if (scan_msg.ranges[i] < range_min)
      {
        range_min = scan_msg.ranges[i];
        index_min = i;
      }
    }
    ROS_INFO("found index_min : %d", index_min);

    if (index_min == 0)
    {
      return ;
    }

    // set ranges in bubble to 0
    unsigned int index_min_temp = index_min;
    while ((index_min_temp >= scan_msg.ranges.size() / 3) && (index_min - index_min_temp < radius))
    {
      // ROS_INFO("%d", index_min_temp);
      ranges[index_min_temp] = 0;
      index_min_temp -= 1;
    }
    index_min_temp = index_min;
    while ((index_min_temp <= 2 * scan_msg.ranges.size() / 3) && (index_min_temp - index_min < radius))
    {
      // ROS_INFO("%d", index_min_temp);
      ranges[index_min_temp] = 0;
      index_min_temp += 1;
    }
    // ROS_INFO("set ranges in bubble to 0");

    // find largest gap
    double range_max;
    unsigned int index_max;
    if (index_min <= scan_msg.ranges.size() / 2)
    {
      range_max = scan_msg.ranges[index_min + radius];
      index_max = index_min + radius;
      for (unsigned int i = index_max; i < 2 * scan_msg.ranges.size() / 3; i++)
      {
        if (ranges[i] > range_max)
        {
          range_max = ranges[i];
          index_max = i;
        }
      }
    }
    else
    {
      range_max = scan_msg.ranges[index_min - radius];
      index_max = index_min - radius;
      for (unsigned int i = index_max; i > scan_msg.ranges.size() / 3; i--)
      {
        if (ranges[i] > range_max)
        {
          range_max = ranges[i];
          index_max = i;
        }
      }
    }
    ROS_INFO("found index_max : %d\n", index_max);
    // ROS_INFO("found range_max : %f", range_max);

    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.steering_angle = scan_msg.angle_min + scan_msg.angle_increment*index_max;
    ROS_INFO("steering_angle = %f", 180*drive_msg.steering_angle/M_PI);
    drive_msg.speed = 1.0;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    drive_st_msg.header = header;

    // set drive message in drive stamped message
    drive_st_msg.drive = drive_msg;

    // publish AckermannDriveStamped message to drive topic
    drive_pub.publish(drive_st_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "safety_node");
  Safety sn;
  ros::spin();
  return 0;
}