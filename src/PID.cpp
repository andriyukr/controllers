/** *************************** pid.cpp ***************************
 *
 * This cpp node is a simple PID controller.
 * It calculates the error in position and velocity from the actual
 * position and velocity estimates and the desired position and
 * velocity from the trajectory generator node.
 * This code uses two sets of P-I-D gains for xy and z axes.
 * Gains are initialzed in the code but later the values are taken
 * from the user.
 *
 * ***************************************************************/

#include "controllers/PID.h"

/* UAV odometry callback function
 * Subscribes to the UAV position, velocity and rate estimates [created in safe_y6]
 */
void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, yaw;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[PID] odom yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

/* Reference trajectory callback function
 * Subscribes to the commanded trajectory topic
 */
void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[PID] position_d: " << pose_d.transpose() << endl;
    //cout << "[PID] position_yaw: " << pose_d(3) * 180 / M_PI << endl;
}

/* Reference trajectory velocity callback function
 * Subscribes to the commanded trajectory velocity topic
 */
void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[PID] velocity_d: " << velocity_d.transpose() << endl;
}

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for P-I-D gains (separately for xy and z axes) from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setPIDConfig &config, uint32_t level){
    k_p_xy = config.k_p_xy;
    k_i_xy = config.k_i_xy;
    k_d_xy = config.k_d_xy;
    k_p_z = config.k_p_z;
    k_i_z = config.k_i_z;
    k_d_z = config.k_d_z;
    //k_p = config.k_p;
    //k_i = config.k_i;
    //k_d = config.k_d;
}

// Constructor
PID::PID(int argc, char** argv){
    ros::init(argc, argv, "PID");
    ros::NodeHandle node_handle;

    // Subscribers
    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);

    // Publishers
    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    if(argc == 7){
        k_p_xy = atof(argv[1]);
        k_i_xy = atof(argv[2]);
        k_d_xy = atof(argv[3]);
        k_p_z = atof(argv[4]);
        k_i_z = atof(argv[5]);
        k_d_z = atof(argv[6]);
    }
    else{
        k_p_xy = 2.0;
        k_i_xy = 0.1;
        k_d_xy = 0.6;
        k_p_z = 2.0;
        k_i_z = 0.1;
        k_d_z = 0.6;
    }

    error_i << 0, 0, 0, 0;

    //new_odometry = false;
}

// Destructor
PID::~PID(){
    ros::shutdown();
    exit(0);
}

// Function to denormalize the angle between -pi and pi
double PID::denormalizeAngle(double a1, double a2){
    if(abs(a2 - a1) > M_PI){
        if(a2 < a1)
            a1 -= 2 * M_PI;
        else
            a1 += 2 * M_PI;
    }
    return a1;
}

void PID::run(){

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setPIDConfig> server;
    dynamic_reconfigure::Server<controllers::setPIDConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    double time = 0;
    int c = 0;
    geometry_msgs::Quaternion velocity_msg;

    int controller_type;

    // Main loop
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        ros::param::get("/safe_y6/controller", controller_type);

        if(controller_type != 0 && new_odometry){ // command
            pose(3) = denormalizeAngle(pose(3), pose_d(3));

            //ros::Time begin = ros::Time::now();

            error = pose_d - pose;

            error_i += error * dt;
            error_d = velocity_d - velocity;

            // PID part
            velocity_msg.x = k_p_xy * error(0) + k_i_xy * error_i(0) + k_d_xy * error_d(0);
            velocity_msg.y = k_p_xy * error(1) + k_i_xy * error_i(1) + k_d_xy * error_d(1);
            velocity_msg.z = k_p_z * error(2) + k_i_z * error_i(2) + k_d_z * error_d(2);

            /*velocity_msg.x = k_p * error(0) + k_i * error_i(0) + k_d * error_d(0);
            velocity_msg.y = k_p * error(1) + k_i * error_i(1) + k_d * error_d(1);
            velocity_msg.z = k_p * error(2) + k_i * error_i(2) + k_d * error_d(2);*/

            //velocity_msg.w = 0; //k_p * error(3);
            velocity_msg.w = 0.5 * error(3);

            velocity_publisher.publish(velocity_msg);

            // Debug: calculate time taken to execute a loop
            //time += (ros::Time::now() - begin).toSec() * 1000;
            //c++;

            //cout << "[PID]: time = " << (time/c) << endl;
        }

        // Uncomment to not check for a new entry in the odometry
        //new_odometry = false;
    }
}

int main(int argc, char** argv){
    cout << "[PID] PID position controller is running..." << endl;

    PID* controller = new PID(argc, argv);

    controller->run();
}
