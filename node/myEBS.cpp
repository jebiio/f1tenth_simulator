#include <ros/ros.h>
#include <ros/package.h>

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
  // TODO: create ROS subscribers and publishers

  // Listen for odom messages
  ros::Subscriber odom_sub;
  ros::Subscriber laser_sub;

  // Publish drive data
  ros::Publisher brake_pub;
  ros::Publisher brake_bool_pub;

  // previous desired steering angle
  double prev_angle = 0.0;

  // precompute cosines of scan angles
  std::vector<double> cosines;

  // precompute distance from lidar to edge of car for each beam
  std::vector<double> car_distances;

public:
  Safety()
  {
    n = ros::NodeHandle();
    speed = 0.0;
    /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

    // TODO: create ROS subscribers and publishers
    // Initialize the node handle
    n = ros::NodeHandle("~");

    // get topic names
    std::string brake_topic, odom_topic, scan_topic, brake_bool_topic;

    n.getParam("brake_drive_topic", brake_topic);
    n.getParam("odom_topic", odom_topic);
    n.getParam("scan_topic", scan_topic);
    n.getParam("brake_bool_topic", brake_bool_topic);

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
    brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(brake_topic, 10);
    brake_bool_pub = n.advertise<std_msgs::Bool>(brake_bool_topic, 10);

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
    for (size_t i = 0; i < scan_msg.ranges.size(); i++)
    {
      // calculate projected velocity
      double proj_velocity = speed * cosines[i];
      double ttc = (scan_msg.ranges[i] - car_distances[i]) / proj_velocity;

      // if it's small, there's a collision
      if ((ttc < 0.5) && (ttc >= 0.0))
      {
        // Make and publish message
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        std_msgs::Header header;
        drive_msg.speed = 0.0;
        drive_msg.steering_angle = 0.0;
        header.stamp = ros::Time::now();

        drive_st_msg.header = header;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        brake_pub.publish(drive_st_msg);

        std_msgs::Bool brake_bool_msg;
        brake_bool_msg.data = true;

        brake_bool_pub.publish(brake_bool_msg);
        ROS_INFO("mynode : brake!");
        return;
      }
    }
    //
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "safety_node");
  Safety sn;
  ros::spin();
  return 0;
}