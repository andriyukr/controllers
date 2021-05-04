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
#include <Eigen/Dense>
#include <olympe_bridge/PilotingCommand.h>
#include <olympe_bridge/CameraCommand.h>

#define FILTER_SIZE 		3

#define GIMBAL_PITCH_MIN 	-135
#define GIMBAL_PITCH_MAX 	100
#define GIMBAL_ROLL_MIN 	-38
#define GIMBAL_ROLL_MAX 	38
#define ZOOM_MIN 		1
#define ZOOM_MAX 		3

#define BOUND3(x, min_x, max_x) (x > max_x ? max_x : x < min_x ? min_x : x)
#define BOUND2(x, min_max) BOUND3(x, -min_max, min_max)
#define GET_MACRO(_1, _2, _3, NAME, ...) NAME
#define BOUND(...) GET_MACRO(__VA_ARGS__, BOUND3, BOUND2)(__VA_ARGS__)

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
Subscriber desired_pose_subscriber;
Subscriber desired_velocity_subscriber;
Subscriber desired_attitude_subscriber;
Subscriber optitrack_subscriber;
Subscriber aruco_subscriber;
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
Publisher position_publisher;
Publisher desired_velocity_publisher;
Publisher acceleration_publisher;

// Flags
bool arm = false;
bool land = true;
bool marker_visibile = true; // DANGEROUS!!!
bool global = false;

// Pose
Vector3d position = Vector3d::Zero();
Vector3d orientation = Vector3d::Zero();
Vector4d pose_error = Vector4d::Zero();
Vector4d pose_error_i = Vector4d::Zero();
Vector4d pose_error_d = Vector4d::Zero();
double yaw = 0;
double relative_yaw = 0;
double initial_yaw = 1000*M_PI;
MatrixXd bounds(2, 3); // min_x, min_y, min_z; max_x, max_y, max_z

// Velocity
Vector3d velocity_error = Vector3d::Zero();
Vector3d velocity_error_i = Vector3d::Zero();
Vector3d velocity_error_d = Vector3d::Zero();
Vector3d velocity = Vector3d::Zero();
Vector3d velocity_old = Vector3d::Zero();
Vector3d velocity_d = Vector3d::Zero();
vector<Twist> velocities;
Vector3d rates = Vector3d::Zero();

// Acceleration
Vector3d acceleration = Vector3d::Zero();
MatrixXd accelerations = MatrixXd::Zero(FILTER_SIZE, 3);

// Commands
Vector4d move_command = Vector4d::Zero();
Vector4d keyboard_command = Vector4d::Zero();
Vector4d skycontroller_command = Vector4d::Zero();
Vector4d desired_pose = Vector4d::Zero();
Vector4d desired_velocity = Vector4d::Zero();
Vector4d desired_attitude = Vector4d::Zero();

// Gains
Vector4d k_p;
Vector4d k_i;
Vector4d k_d;
Vector4d max_i;

// Camera
double gimbal_roll = 0;
double gimbal_pitch = 0;
double zoom = 1; 

// Parameters
int controller = 0;
double max_tilt;
double max_vertical_speed;
double max_yaw_rotation_speed;

Time time_old;

class SafeAnafi{
    public:
    	SafeAnafi(int, char**);
        ~SafeAnafi();
        void run();
    private:

};
