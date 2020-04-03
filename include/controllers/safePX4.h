#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <controllers/RollPitchYawThrust.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/RCIn.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setSafePX4Config.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/BatteryState.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

/*
#define MAX_X   10
#define MAX_Y   4
#define MAX_Z   5
#define MAX_V   3
//#define MAX_RP  (M_PI / 10)
#define MAX_RP  (M_PI / 2)
*/
// Fast flight
#define MAX_X   110
#define MAX_Y   50
#define MAX_Z   10
#define MAX_V   3
//#define MAX_RP  (M_PI / 10)
#define MAX_RP  (M_PI)


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
Publisher odometry_publisher;
Publisher noisy_odometry_publisher;
Publisher attitude_publisher;
Publisher throttle_publisher;
Publisher position_publisher;
Publisher test_msg_publish;
Publisher att_com_publisher;
Publisher att_local_publisher;
Publisher raw_att_com_publisher;
Publisher raw_pos_com_publisher;

// Subsribers
Subscriber command_subscriber;
Subscriber command_position_subscriber;
Subscriber command_velocity_subscriber;
Subscriber command_attitude_subscriber;
Subscriber optitrack_subscriber;
Subscriber noise_subscriber;
Subscriber battery_subscriber;
Subscriber state_subsriber;
Subscriber velocity_actual_subscriber;
Subscriber local_pos_sub;
Subscriber RC_input_sub;

// Services
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;

//MavROS
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandTOL land_cmd;
mavros_msgs::RCIn RCinputs;

bool RCavailable =false;

Vector4d position_d;
Vector4d velocity_d;
Vector4d attitude_d;

// Variables
Point position;
Point orientation;
vector<Twist> velocities;
nav_msgs::Odometry noise;
Time time_old;
int sequence;
double yaw;
bool stop;
int controller;
double thrust_multiplier;

double error = 0.0, error_prev = 0.0;

class SafePX4{
    public:
          SafePX4(int, char**);
          ~SafePX4();
          void run();
    private:
          void push(double vx, double vy, double vz, double p, double q, double r);
          Twist filter();
};
