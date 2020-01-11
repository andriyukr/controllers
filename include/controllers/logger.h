#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_msgs/Actuators.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector3d;
using Eigen::Vector4d;

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber true_odometry_subscriber;
ros::Subscriber velocity_subscriber;
ros::Subscriber motor_speed_subscriber;
ros::Subscriber fnn_subscriber;

// Data
Vector4d pose;
Vector4d pose_d;
Vector4d pose_r;
Vector3d velocity;
Vector3d angular_velocity;
Vector4d velocity_d;
VectorXd omega(6);
Vector4d fnn;

// File
ofstream results;

class Logger{
        public:
          Logger(int, char**);
          ~Logger();
          void run();
};
