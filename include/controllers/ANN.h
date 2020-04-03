#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setANNConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;

#define WEIGHTS     0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001
#define ALPHA       0.001
#define GAMMA       1.5

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;
ros::Subscriber pd_subscriber;

// Publishers
ros::Publisher velocity_publisher;
ros::Publisher ann_velocity_publisher;
ros::Publisher ann_params_publisher;

// Actual state
Vector4d pose;
Vector4d velocity;
Vector4d pose_d;
Vector4d velocity_d;

Vector4d u_c;
Vector4d u_f;

Vector4d e;
Vector4d e_d;
MatrixXd f(4, 9);
Vector4d q;

// Support
VectorXd W(9);
VectorXd W_bar(9);
VectorXd f_d(9);
double q_d;
double alpha_d;

// Gains
Vector4d alpha;
Vector4d gamma1;

Vector4d alpha_0;
Vector4d gamma1_0;

bool new_odometry;

int reset_time = 0;

class ANN{
        public:
          ANN(int, char**);
          ~ANN();
          void run();

        private:
          double update(short axis, double x1, double x2, double u_c, double dt);
          short sign(double x);
          double bound(double v, double min, double max);
          VectorXd bound(VectorXd v, double min, double max);
};
