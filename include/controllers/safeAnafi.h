#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setSafeAnafiConfig.h>
//#include <aruco_mapping/ArucoMarker.h>
#include <Eigen/Dense>
#include <olympe_bridge/ParrotCommand.h>

#define MAX_X           1
#define MAX_Y           1
#define MAX_Z           2
#define MAX_V           0.3
#define MAX_TILT        20
#define MAX_ROTATION    100

#define GIMBAL_PITCH_MIN 	-135
#define GIMBAL_PITCH_MAX 	100
#define GIMBAL_ROLL_MIN 	-38
#define GIMBAL_ROLL_MAX 	38
#define ZOOM_MIN 		1
#define ZOOM_MAX 		3

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

// Subsribers
Subscriber command_subscriber;
Subscriber command_keyboard_subscriber;
Subscriber command_camera_subscriber;
Subscriber skycontroller_subscriber;
Subscriber desired_velocity_subscriber;
Subscriber desired_attitude_subscriber;
Subscriber optitrack_subscriber;
Subscriber aruco_subscriber;
Subscriber noise_subscriber;
Subscriber gps_subscriber;
Subscriber odometry_subscriber;

// Publishers
Publisher emergency_publisher;
Publisher takeoff_publisher;
Publisher land_publisher;
Publisher offboard_publisher;
Publisher move_publisher;
Publisher rpyg_publisher;
Publisher camera_publisher;
Publisher euler_publisher;
Publisher odometry_publisher;
Publisher noisy_odometry_publisher;
Publisher desired_velocity_publisher;

// Velocity PID
Vector4d k_p;
Vector4d k_i;
Vector4d k_d;
Vector4d max_i;
Vector4d error;
Vector4d error_old;
Vector4d error_i;
Vector4d error_d;
Vector4d velocity;
Vector4d velocity_d;

Vector4d move_command;
Vector4d keyboard_command;
Vector4d skycontroller_command;
Vector4d desired_velocity;
Vector4d desired_attitude;

// Variables
Point position;
Point orientation;
vector<Twist> velocities;
nav_msgs::Odometry noise;
Time time_old;
int sequence;
double yaw = 0;
double initial_yaw = 1000*M_PI;
double gimbal_roll = 0;
double gimbal_pitch = 0;
double zoom = 1; 
bool arm = false;
bool land = true;
bool marker_visibile = true; // DANGEROUS!!!
int controller = 1;
bool global = false;

// Parameters
double max_tilt = 1, max_vertical_speed = 1, max_yaw_rotation_speed = 1;

class SafeAnafi{
    public:
          SafeAnafi(int, char**);
          ~SafeAnafi();
          void run();
    private:
          void push(double vx, double vy, double vz, double p, double q, double r);
          Twist filter();
};
