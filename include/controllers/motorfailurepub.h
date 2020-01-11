#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "dynamic_reconfigure/server.h"
#include <controllers/setMotorFailureConfig.h>
#include <vector>

using namespace std;
using namespace ros;

// Subscribers
ros::Subscriber traj_type_sub;

// Publishers
ros::Publisher motor_failure_number_pub;
ros::Publisher record_start_flag_pub;

bool Fail_motor;
int motor_num, t_fail_max;
float t_stop = 0;
int t_motor_off = 6.5;

std_msgs::Int32 record_start, traj_type, motor_failure_num;
std_msgs::Bool motor_failure_flag;
