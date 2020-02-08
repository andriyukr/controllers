/** *************************** safeBebop.cpp ***************************
 *
 * This code collects all the messages:
 *      - Feedback from the UAV: state estimates for odometry
 *      - Reference trajectory: desired position and velocity from the trajectory generator
 *      - UAV commands: position, velocity and attitude commands from PID or ANN/FNN-PD
 *      - Keyboard input commands: commands from user by keyboard input
 *      - Constraints and safety: contains constraints and safety bounds for commands
 *      - Controller: lets the user choose between position/velocity/attitude commands
 *
 * *********************************************************************/

#include "controllers/safeBebop.h"

void push(double vx, double vy, double vz, double p, double q, double r){
    Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.linear.z = vz;
    t.angular.x = p;
    t.angular.y = q;
    t.angular.z = r;

    velocities.erase(velocities.begin());
    velocities.push_back(t);
}

Twist filter(){
    Twist t;
    t.linear.x = 0;
    t.linear.y = 0;
    t.linear.z = 0;
    t.angular.x = 0;
    t.angular.y = 0;
    t.angular.z = 0;
    for(vector<Twist>::iterator i = velocities.begin(); i != velocities.end(); ++i){
        t.linear.x += i->linear.x;
        t.linear.y += i->linear.y;
        t.linear.z += i->linear.z;
        t.angular.x += i->angular.x;
        t.angular.y += i->angular.y;
        t.angular.z += i->angular.z;
    }
    t.linear.x /= 10;
    t.linear.y /= 10;
    t.linear.z /= 10;
    t.angular.x /= 10;
    t.angular.y /= 10;
    t.angular.z /= 10;
    return t;
}

void optitrackCallback(const geometry_msgs::PoseStamped& optitrack_msg){
    Duration dt = optitrack_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = optitrack_msg.header.seq;
    odometry_msg.header.stamp = optitrack_msg.header.stamp;
    odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
    odometry_msg.child_frame_id = "bebop";

    odometry_msg.pose.pose.position.x = optitrack_msg.pose.position.x;
    odometry_msg.pose.pose.position.y = optitrack_msg.pose.position.y;
    odometry_msg.pose.pose.position.z = optitrack_msg.pose.position.z;

    tf::Quaternion q(optitrack_msg.pose.orientation.x, optitrack_msg.pose.orientation.y, optitrack_msg.pose.orientation.z, optitrack_msg.pose.orientation.w);
    //q.normalize();
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
    //q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    odometry_msg.pose.pose.orientation = optitrack_msg.pose.orientation;

    double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
    double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
    double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
    double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
    double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
    double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
    push(vx, vy, vz, omega_x, omega_y, omega_z);
    odometry_msg.twist.twist = filter();
    odometry_publisher.publish(odometry_msg);

    tf::Quaternion qc(noise.pose.pose.orientation.x, noise.pose.pose.orientation.y, noise.pose.pose.orientation.z, noise.pose.pose.orientation.w);
    tf::Quaternion rqc = q * qc;
    odometry_msg.pose.pose.position.x += noise.pose.pose.position.x;
    odometry_msg.pose.pose.position.y += noise.pose.pose.position.y;
    odometry_msg.pose.pose.position.z += noise.pose.pose.position.z;
    odometry_msg.pose.pose.orientation.x = rqc.getX(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.y = rqc.getY(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.z = rqc.getZ(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.w = rqc.getW(); // DO NOT WORK
    odometry_msg.twist.twist.linear.x += odometry_msg.twist.twist.linear.x;
    odometry_msg.twist.twist.linear.y += odometry_msg.twist.twist.linear.y;
    odometry_msg.twist.twist.linear.z += odometry_msg.twist.twist.linear.z;
    odometry_msg.twist.twist.angular.x += odometry_msg.twist.twist.angular.x;
    odometry_msg.twist.twist.angular.y += odometry_msg.twist.twist.angular.y;
    odometry_msg.twist.twist.angular.z += odometry_msg.twist.twist.angular.z;
    noisy_odometry_publisher.publish(odometry_msg);

    time_old = odometry_msg.header.stamp;
    position = odometry_msg.pose.pose.position;
    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = yaw;

    velocity << vx, vy, vz, omega_z;

    marker_visibile = true;

    //cout << "[SafeBebop] yaw = " << yaw << endl;
    /*tf::Quaternion q1(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    m1.getEulerZYX(z, y, x);
    cout << "[SafeBebop]:" << endl;
    cout << "x = " << odometry_msg.pose.pose.position.x << ",\t y =  " << odometry_msg.pose.pose.position.y << ",\t z = " << odometry_msg.pose.pose.position.z << endl;
    cout << "roll = " << x << ",\t pitch =  " << y << ",\t yaw = " << z << endl;
    cout << "vx = " << odometry_msg.twist.twist.linear.x << ",\t vy =  " << odometry_msg.twist.twist.linear.y << ",\t vz = " << odometry_msg.twist.twist.linear.z << endl;
    cout << "p = " << odometry_msg.twist.twist.angular.x << ",\t q =  " << odometry_msg.twist.twist.angular.y << ",\t r = " << odometry_msg.twist.twist.angular.z << endl;*/
}

/*void arucoCallback(const aruco_mapping::ArucoMarker& aruco_msg){
    marker_visibile = aruco_msg.marker_visibile;
    if(aruco_msg.marker_visibile){
        Duration dt = aruco_msg.header.stamp - time_old;
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.seq = aruco_msg.header.seq;
        odometry_msg.header.stamp = aruco_msg.header.stamp;
        odometry_msg.header.frame_id = aruco_msg.header.frame_id;
        odometry_msg.child_frame_id = "bebop";

        odometry_msg.pose.pose.position.x = -aruco_msg.global_camera_pose.position.z;
        odometry_msg.pose.pose.position.y = aruco_msg.global_camera_pose.position.y;
        odometry_msg.pose.pose.position.z = aruco_msg.global_camera_pose.position.x;

        tf::Quaternion q(aruco_msg.global_camera_pose.orientation.x, aruco_msg.global_camera_pose.orientation.y, aruco_msg.global_camera_pose.orientation.z, aruco_msg.global_camera_pose.orientation.w);
        //q.normalize();
        tf::Matrix3x3 m(q);
        double x, y, z, roll, pitch;
        m.getEulerZYX(z, y, x);
        roll = x;
        pitch = y;
        yaw = z;
        //q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        odometry_msg.pose.pose.orientation = aruco_msg.global_camera_pose.orientation;

        // Computes linear and angular velocities by the numerical derivation and filtering
        double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
        double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
        double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
        double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
        double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
        double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
        push(vx, vy, vz, omega_x, omega_y, omega_z);
        odometry_msg.twist.twist = filter();
        noisy_odometry_publisher.publish(odometry_msg);

        time_old = odometry_msg.header.stamp;
        position = odometry_msg.pose.pose.position;
        orientation.x = roll;
        orientation.y = pitch;
        orientation.z = yaw;

        velocity << vx, vy, vz, omega_z;
    }
}*/

void odometryCallback(const nav_msgs::Odometry odometry_msg){
    tf::Quaternion q(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
}

float bound(float v, float b){
    if(v < -b)
        return -b;
    if(v > b)
        return b;
    return v;
}

void commandCallback(const std_msgs::Int8& command_msg){
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist velocity_msg;
    switch(command_msg.data){
    case 1:
        velocity_msg.linear.x = 0;
        velocity_msg.linear.y = 0;
        velocity_msg.linear.z = 0;
        velocity_msg.angular.x = 0;
        velocity_msg.angular.y = 0;
        velocity_msg.angular.z = 0;
        velocity_publisher.publish(velocity_msg);
        velocity_publisher.publish(velocity_msg);
        land = true;
        break;
    case 2:
        takeoff_publisher.publish(empty_msg);
        takeoff_publisher.publish(empty_msg);
        Duration(5).sleep();
        land = false;
        break;
    case 3:
        velocity_msg.linear.x = 0;
        velocity_msg.linear.y = 0;
        velocity_msg.linear.z = 0;
        velocity_msg.angular.x = 0;
        velocity_msg.angular.y = 0;
        velocity_msg.angular.z = 0;
        velocity_publisher.publish(velocity_msg);
        velocity_publisher.publish(velocity_msg);
        land_publisher.publish(empty_msg);
        land_publisher.publish(empty_msg);
        land = true;
        break;
    case 4:
        reset_publisher.publish(empty_msg);
        reset_publisher.publish(empty_msg);
    }
}

void commandVelocityCallback(const geometry_msgs::Quaternion& command_msg){   
    float vx = bound(command_msg.x, MAX_V) / ((double)MAX_TILT * M_PI / 180);
    float vy = bound(command_msg.y, MAX_V) / ((double)MAX_TILT * M_PI / 180);
    float vz = bound(command_msg.z, MAX_V);
    float vw = bound(command_msg.w, MAX_V) / ((double)MAX_ROTATION * M_PI / 180);

    velocity_d << vx, vy, vz, vw;
}

void commandCameraCallback(const geometry_msgs::Vector3& command_msg){
    geometry_msgs::Twist camera_msg;
    camera_msg.linear.x = 0;
    camera_msg.linear.y = 0;
    camera_msg.linear.z = 0;
    camera_msg.angular.x = 0;
    camera_msg.angular.y = command_msg.y;
    camera_msg.angular.z = command_msg.z;
    camera_publisher.publish(camera_msg);
}

void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

// Constructor
SafeBebop::SafeBebop(int argc, char** argv){
    ros::init(argc, argv, "safeBebop");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    //aruco_subscriber = node_handle.subscribe("/aruco/odometry", 1, arucoCallback);
    odometry_subscriber = node_handle.subscribe("/bebop/odom", 1, odometryCallback);
    command_subscriber = node_handle.subscribe("/bebop/command", 1, commandCallback);
    command_velocity_subscriber = node_handle.subscribe("/bebop/command_velocity", 1, commandVelocityCallback);
    command_camera_subscriber = node_handle.subscribe("/bebop/command_camera", 1, commandCameraCallback);
    noise_subscriber = node_handle.subscribe("/bebop/noise", 1, noiseCallback);

    reset_publisher = node_handle.advertise<std_msgs::Empty>("/bebop/reset", 1);
    takeoff_publisher = node_handle.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    land_publisher = node_handle.advertise<std_msgs::Empty>("/bebop/land", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    camera_publisher = node_handle.advertise<geometry_msgs::Twist>("/bebop/camera_control", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/bebop/ground_truth/odometry", 1);
    noisy_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/bebop/odometry", 1);

    position.x = 0;
    position.y = 0;
    position.z = 0;

    k_p << 0.2, 0.2, 0.6, 0.9;
    k_i << 0.5, 0.5, 0.0, 0.0;
    k_d << 0.0, 0.0, 0.0, 0.0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;
    error_i << 0, 0, 0, 0;

    yaw = 0;

    land = true;
    marker_visibile = false;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);
}

// Destructor
SafeBebop::~SafeBebop(){
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist velocity_msg;
    velocity_msg.linear.x = 0;
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;
    velocity_msg.angular.z = 0;
    velocity_publisher.publish(velocity_msg);
    velocity_publisher.publish(velocity_msg);
    land_publisher.publish(empty_msg);
    land_publisher.publish(empty_msg);
    land = true;

    ros::shutdown();
    exit(0);
}

void SafeBebop::run(){
    double dt = (double)1/100;
    ros::Rate rate(100);

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        geometry_msgs::Twist velocity_msg;
        if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z
                && (velocity_d(0) != 0 || velocity_d(1) != 0 || velocity_d(2) != 0 || velocity_d(3) != 0) && marker_visibile){

            error = velocity_d - velocity;
            error = velocity_d; // FOR DEMO

            error_i += error * dt;
            error_i << bound(error_i(0), MAX_I), bound(error_i(1), MAX_I), bound(error_i(2), MAX_I), bound(error_i(3), MAX_I);
            error_d = (error - error_old) / dt;
            error_old = error;

            double vx = bound(k_p(0) * error(0) + k_i(0) * error_i(0) + k_d(0) * error_d(0), 1);
            double vy = bound(k_p(1) * error(1) + k_i(1) * error_i(1) + k_d(1) * error_d(1), 1);
            double vz = bound(k_p(2) * error(2) + k_i(2) * error_i(2) + k_d(2) * error_d(2), 1);
            double r  = bound(k_p(3) * error(3) + k_i(3) * error_i(3) + k_d(3) * error_d(3), 1);

            yaw = 0; // FOR DEMO

            velocity_msg.linear.x = cos(yaw) * vx + sin(yaw) * vy;
            velocity_msg.linear.y = -sin(yaw) * vx + cos(yaw) * vy;
            velocity_msg.linear.z = vz;
            velocity_msg.angular.x = 0;
            velocity_msg.angular.y = 0;
            velocity_msg.angular.z = r;
        }
        else{
            velocity_msg.linear.x = 0;
            velocity_msg.linear.y = 0;
            velocity_msg.linear.z = 0;
            velocity_msg.angular.x = 0;
            velocity_msg.angular.y = 0;
            velocity_msg.angular.z = 0;
        }
        if(!land)
            velocity_publisher.publish(velocity_msg);
    }
}

int main(int argc, char** argv){
    cout << "[SafeBebop] SafeBebop is running..." << endl;

    SafeBebop* bebop = new SafeBebop(argc, argv);

    bebop->run();
}
