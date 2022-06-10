#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries

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
  ros::Publisher drive_pub;

  // previous desired steering angle
  double prev_angle = 0.0;

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
    std::string drive_topic, odom_topic, scan_topic;
    n.getParam("new_drive_topic", drive_topic);
    n.getParam("odom_topic", odom_topic);
    n.getParam("scan_topic", scan_topic);

    // get car parameters
    // n.getParam("max_speed", max_speed);
    // n.getParam("max_steering_angle", max_steering_angle);

    // Make a publisher for drive messages
    // drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

    // Start a subscriber to listen to odom messages
    odom_sub = n.subscribe(scan_topic, 1, &Safety::odom_callback, this);
    odom_sub = n.subscribe(odom_topic, 1, &Safety::scan_callback, this);
  }
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
  {
    // TODO: update current speed
    speed = 0.0;
  }

  void scan_callback(const sensor_msgs::LaserScan &scan_msg)
  {
    // for (size_t i = 0; i < scan_msg.ranges.size(); i++)
    // {
    //   double angle = scan_msg.angle_min + i * scan_msg.angle_increment;

    //   // calculate projected velocity
    //   double proj_velocity = state.velocity * cosines[i];
    //   double ttc = (scan_msg.ranges[i] - car_distances[i]) / proj_velocity;

    //   // if it's small, there's a collision
    //   if ((ttc < ttc_threshold) && (ttc >= 0.0))
    //   {
    //     // Send a blank mux and write to file
    //     collision_helper();

    //     in_collision = true;

    //     collision_count++;
    //     collision_file << "Collision #" << collision_count << " detected:\n";
    //     collision_file << "TTC: " << ttc << " seconds\n";
    //     collision_file << "Angle to obstacle: " << angle << " radians\n";
    //     collision_file << "Time since start of sim: " << (ros::Time::now().toSec() - beginning_seconds) << " seconds\n";
    //     collision_file << "\n";
    //     return;
    //   }
    // }
    // // if it's gone through all beams without detecting a collision, reset in_collision
    // in_collision = false;
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "safety_node");
  Safety sn;
  ros::spin();
  return 0;
}