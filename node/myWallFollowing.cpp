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

    n.getParam("drive_topic", drive_topic);
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
    if(scan_msg.angle_min < 0)
    {
      // compute error
      // right wall
      // double range_zero = scan_msg.ranges[(int)(-scan_msg.angle_min / scan_msg.angle_increment)];
      // double range_nonzero = scan_msg.ranges[(int)(-scan_msg.angle_min / scan_msg.angle_increment) + (int)((M_PI / 4) / scan_msg.angle_increment)];

      // left wall
      double range_zero = scan_msg.ranges[3 * scan_msg.ranges.size() / 4];
      double range_nonzero = scan_msg.ranges[5 * scan_msg.ranges.size() / 8];
      double alpha = atan( (range_nonzero*cos(M_PI/4) - range_zero) / range_nonzero*sin(M_PI/4) );
      double distance = range_zero * cos(M_PI/4);
      double future_distance = distance + (speed*0.017*sin(alpha));
      double error = DESIRED_IDSTANCE - future_distance;

      // compute pid output
      ackermann_msgs::AckermannDriveStamped drive_st_msg;
      ackermann_msgs::AckermannDrive drive_msg;

      drive_msg.steering_angle = -fmod(error*KP, M_PI);

      if ((fabs(drive_msg.steering_angle) >= 0) && (fabs(drive_msg.steering_angle) <= (M_PI / 36)))
      {
        drive_msg.speed = 1.5;
      }
      else if ((fabs(drive_msg.steering_angle) >= (M_PI / 36)) && (fabs(drive_msg.steering_angle) <= (M_PI / 18)))
      {
        drive_msg.speed = 1.0;
      }
      else
      {
        drive_msg.speed = 0.5;
      }
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      drive_st_msg.header = header;

      // set drive message in drive stamped message
      drive_st_msg.drive = drive_msg;

      // publish AckermannDriveStamped message to drive topic
      drive_pub.publish(drive_st_msg);

      // ROS_INFO("range_size = %d", (int)(scan_msg.ranges.size()));
      // ROS_INFO("range_zero = %f", range_zero);
      // ROS_INFO("range_nonzero = %f", range_nonzero);
      // ROS_INFO("distance = %f", distance);
      // ROS_INFO("future_distance = %f", future_distance);
      ROS_INFO("error = %f", error);
      ROS_INFO("steering_angle = %f\n", (180*drive_msg.steering_angle)/M_PI);
      // ROS_INFO("speed = %f\n", drive_msg.speed);

      return;
    }

    else
    {
      ROS_INFO("\nmynode : scan_msg.angle_min is smaller than 0.\n");
      return;
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