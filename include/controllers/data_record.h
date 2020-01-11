#include <fstream>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setDataRecordConfig.h>

using namespace std;
using namespace geometry_msgs;
using namespace ros;
using namespace Eigen;

using Eigen::Vector3d;
using Eigen::Vector4d;

// Record start Publisher
ros::Publisher record_flag_pub;

// Command Subscribers
ros::Subscriber ref_traj_sub, com_att_sub, com_vel_sub, com_thr_sub;

// UAV feedback subscribers
ros::Subscriber local_pos_sub, local_att_sub, local_vel_sub;

// Other subscribers
ros::Subscriber pd_vel_sub, ann_vel_sub, fnn_vel_sub, motor_num_sub, traj_type_sub;

//ifstream read_name;
ofstream outputFile;

std_msgs::Int32 rec_start_flag;

double t, t_last, rec_time;
bool record_start = 0;
bool screen_msg_flag = 1;

int i, result;

// strings for naming the file
string trajectory_type, controller_type, setpoint_type, motor_fail_type, experiment_type, filename;

class data_record{
        public:
          data_record(int, char**);
          ~data_record();
          void run();
};
