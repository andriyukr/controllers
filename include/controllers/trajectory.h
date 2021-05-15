#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setTrajectoryConfig.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>

#define ROUND(v) (((v.array()*1e3).round()/1e3).transpose())

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

// Subscribers
ros::Subscriber command_subscriber;
ros::Subscriber pose_subscriber;

// Publishers
ros::Publisher trajectory_publisher;
ros::Publisher velocity_publisher;

// Pose
Vector4d pose_d  = Vector4d::Zero();
Vector4d pose = Vector4d::Zero();

// Trajectory type
int trajectory_type;

// Trajectory speed
double speed;
double straight_speed;

// Trajectory size
double scale;

// Yaw angle
double yaw_d, initial_local_yaw, last_yaw;

// Waypoints
MatrixXd waypoints;
int waypoint = 0;
MatrixXd curve_parameters_pose;
MatrixXd curve_parameters_velocity;
VectorXd curve_time;

// Time
double t;
double t_straight;

class Trajectory{
        public:
          Trajectory(int, char**);
          ~Trajectory();
          void run();
        private:
          double denormalizeAngle(double a1);
          double distance(Vector4d v1, Vector4d v2);
          void readWaypoints(std::string fileName);
          void readWaypointsStamped(std::string fileName);
          void curveFitting(MatrixXd waypoints);
};
