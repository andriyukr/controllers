#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <aruco_mapping/ArucoMarker.h>
#include <Eigen/Dense>

#define MAX_X           3
#define MAX_Y           3
#define MAX_Z           3
#define MAX_V           0.3
#define MAX_TILT        20
#define MAX_ROTATION    100
#define MAX_I           0.01

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

// Publishers
Publisher reset_publisher;
Publisher takeoff_publisher;
Publisher land_publisher;
Publisher velocity_publisher;
Publisher camera_publisher;
Publisher odometry_publisher;
Publisher noisy_odometry_publisher;

// Subsribers
Subscriber command_subscriber;
Subscriber command_velocity_subscriber;
Subscriber command_camera_subscriber;
Subscriber optitrack_subscriber;
Subscriber aruco_subscriber;
Subscriber noise_subscriber;
Subscriber gps_subscriber;
Subscriber odometry_subscriber;

// Velocity PID
Vector4d k_p;
Vector4d k_i;
Vector4d k_d;
Vector4d error;
Vector4d error_old;
Vector4d error_i;
Vector4d error_d;
Vector4d velocity;
Vector4d velocity_d;

// Variables
Point position;
Point orientation;
vector<Twist> velocities;
nav_msgs::Odometry noise;
Time time_old;
int sequence;
double yaw;
bool land;
bool marker_visibile;

class SafeBebop{
    public:
          SafeBebop(int, char**);
          ~SafeBebop();
          void run();
    private:
          void push(double vx, double vy, double vz, double p, double q, double r);
          Twist filter();
};
