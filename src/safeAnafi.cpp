/** *************************** safeAnafi.cpp ***************************
 *
 * This node collects all the messages:
 *      - Feedback from the UAV: state estimates for odometry
 *      - Reference trajectory: desired position and velocity from the trajectory generator
 *      - UAV commands: position, velocity and attitude commands from PID or ANN/FNN-PD
 *      - Keyboard input commands: commands from user by keyboard input
 *      - Constraints and safety: contains constraints and safety bounds for commands
 *      - Controller: lets the user choose between position/velocity/attitude commands
 *
 * *********************************************************************/

#include "controllers/safeAnafi.h"

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

// ********************** Callbacks **********************

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for controller type from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setSafeAnafiConfig &config, uint32_t level){
    controller = config.controller;
    global = config.global;

    k_p << config.groups.gains.k_p, config.groups.gains.k_p, 0, 0;
    k_i << config.groups.gains.k_i, config.groups.gains.k_i, 0, 0;
    k_d << config.groups.gains.k_d, config.groups.gains.k_d, 0, 0;
    max_i << config.groups.gains.max_i, config.groups.gains.max_i, 0, 0;
}

void optitrackCallback(const geometry_msgs::PoseStamped& optitrack_msg){
    Duration dt = optitrack_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = optitrack_msg.header.seq;
    odometry_msg.header.stamp = optitrack_msg.header.stamp;
    odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
    odometry_msg.child_frame_id = "body";

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

    if(initial_yaw == 1000*M_PI)
        initial_yaw = yaw;

    velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z, 0;

    geometry_msgs::Vector3 euler_msg;
    euler_msg.x = roll/M_PI*180;
    euler_msg.y = pitch/M_PI*180;
    euler_msg.z = yaw/M_PI*180;
    euler_publisher.publish(euler_msg);

    //ROS_INFO("odometryCallback");
}

float bound(float value, float boundary){
    return min(max(value, -boundary), boundary);
}

float bound(float value, float min_boundary, float max_boundary){
    return min(max(value, min_boundary), max_boundary);
}

void commandMetaCallback(const std_msgs::Int8& command_msg){
    std_msgs::Empty empty_msg;
    olympe_bridge::ParrotCommand rpyg_msg;
    std_msgs::Bool offboard_msg;
    switch(command_msg.data){
    case 1: // arm
        arm = true;
        break;
    case 2: // take-off
        if(arm){
            offboard_msg.data = false;
            offboard_publisher.publish(offboard_msg);
            takeoff_publisher.publish(empty_msg);
            Duration(1).sleep();
            land = false;
        }else
            ROS_WARN("Arm the drone before taking-off.");
        arm = false;
        break;
    case 3: // hower
        rpyg_publisher.publish(rpyg_msg);
        rpyg_publisher.publish(rpyg_msg);
        break;
    case 4: // land
        rpyg_publisher.publish(rpyg_msg);
        rpyg_publisher.publish(rpyg_msg);
        land_publisher.publish(empty_msg);
        land_publisher.publish(empty_msg);
        land = true;
        break;
    case 5: // switch off motors!
        emergency_publisher.publish(empty_msg);
        emergency_publisher.publish(empty_msg);
        land = true;
        break;
    case 101: // remote control!
        offboard_msg.data = false;
        offboard_publisher.publish(offboard_msg);
        break;
    case 102: // offboard control!
        offboard_msg.data = true;
        offboard_publisher.publish(offboard_msg);
        break;
    }
}

void commandKeyboardCallback(const geometry_msgs::TwistStamped& command_msg){
    ros::param::get("/anafi_bridge/max_tilt", max_tilt);
    ros::param::get("/anafi_bridge/max_vertical_speed", max_vertical_speed);
    ros::param::get("/anafi_bridge/max_yaw_rotation_speed", max_yaw_rotation_speed);
    if(controller == 1) // velocity control
            keyboard_command << command_msg.twist.linear.x*max_vertical_speed, command_msg.twist.linear.y*max_vertical_speed,
                    command_msg.twist.linear.z*max_vertical_speed, command_msg.twist.angular.z*max_yaw_rotation_speed;
    if(controller == 2) // attitude control
            keyboard_command << -command_msg.twist.linear.y*max_tilt, command_msg.twist.linear.x*max_tilt,
                    command_msg.twist.linear.z*max_vertical_speed, command_msg.twist.angular.z*max_yaw_rotation_speed;
}

void commandSkycontrollerCallback(const geometry_msgs::TwistStamped& command_msg){
    ros::param::get("/anafi_bridge/max_tilt", max_tilt);
    ros::param::get("/anafi_bridge/max_vertical_speed", max_vertical_speed);
    ros::param::get("/anafi_bridge/max_yaw_rotation_speed", max_yaw_rotation_speed);
    if(controller == 1) // velocity control
            skycontroller_command << command_msg.twist.angular.y/100*max_vertical_speed, -command_msg.twist.angular.x/100*max_vertical_speed,
                    command_msg.twist.linear.z/100*max_vertical_speed, -command_msg.twist.angular.z/100*max_yaw_rotation_speed;
    if(controller == 2) // attitude control
            skycontroller_command << command_msg.twist.angular.x/100*max_tilt, command_msg.twist.angular.y/100*max_tilt,
                    command_msg.twist.linear.z/100*max_vertical_speed, -command_msg.twist.angular.z/100*max_yaw_rotation_speed;
}

void desiredVelocityCallback(const geometry_msgs::TwistStamped& command_msg){
    desired_attitude << command_msg.twist.linear.x, command_msg.twist.linear.y, command_msg.twist.linear.z, command_msg.twist.angular.z;}

void desiredAttitudeCallback(const geometry_msgs::TwistStamped& command_msg){
    desired_attitude << command_msg.twist.angular.y, -command_msg.twist.angular.x, command_msg.twist.linear.z, command_msg.twist.angular.z;
}

void commandCameraCallback(const geometry_msgs::Twist& command_msg){
    geometry_msgs::Twist camera_msg;

    zoom = bound(zoom + command_msg.linear.x, ZOOM_MIN, ZOOM_MAX);   

    if(command_msg.angular.z == 1){
    	gimbal_roll = 0;
    	gimbal_pitch = 0;
    }else{
        gimbal_roll = bound(gimbal_roll + command_msg.angular.x, GIMBAL_ROLL_MIN, GIMBAL_ROLL_MAX);
    	gimbal_pitch = bound(gimbal_pitch + command_msg.angular.y, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    }

    camera_msg.linear.x = zoom;
    camera_msg.linear.y = 0;
    camera_msg.linear.z = command_msg.linear.z;
    camera_msg.angular.x = gimbal_roll;
    camera_msg.angular.y = gimbal_pitch;
    camera_msg.angular.z = 0;
    camera_publisher.publish(camera_msg);
}

void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

// Constructor
SafeAnafi::SafeAnafi(int argc, char** argv){
    ros::init(argc, argv, "safeBebop");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    //aruco_subscriber = node_handle.subscribe("/aruco/odometry", 1, arucoCallback);
    odometry_subscriber = node_handle.subscribe("/anafi/odometry", 1, odometryCallback);
    command_subscriber = node_handle.subscribe("/keyboard/command_meta", 1, commandMetaCallback);
    command_keyboard_subscriber = node_handle.subscribe("/keyboard/command_move", 1, commandKeyboardCallback);
    command_camera_subscriber = node_handle.subscribe("/keyboard/command_camera", 1, commandCameraCallback);
    skycontroller_subscriber = node_handle.subscribe("/skycontroller/command", 1, commandSkycontrollerCallback);
    desired_velocity_subscriber = node_handle.subscribe("/anafi/desired_velocity", 1, desiredVelocityCallback);
    desired_attitude_subscriber = node_handle.subscribe("/anafi/desired_attitude", 1, desiredAttitudeCallback);
    noise_subscriber = node_handle.subscribe("/anafi/noise", 1, noiseCallback);

    emergency_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/emergency", 1);
    takeoff_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/takeoff", 1);
    land_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/land", 1);
    offboard_publisher = node_handle.advertise<std_msgs::Bool>("/anafi/offboard", 1);
    move_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/anafi/cmd_move", 1);
    rpyg_publisher = node_handle.advertise<olympe_bridge::ParrotCommand>("/anafi/cmd_rpyt", 1);
    camera_publisher = node_handle.advertise<geometry_msgs::Twist>("/anafi/cmd_camera", 1);
    euler_publisher = node_handle.advertise<geometry_msgs::Vector3>("/anafi/euler", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/anafi/ground_truth/odometry", 1);
    noisy_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/anafi/odometry", 1);
    desired_velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/debug/desired_velocity", 1);

    position.x = 0;
    position.y = 0;
    position.z = 1;

    k_p << 0.2, 0.2, 0.6, 0.9;
    k_i << 0.5, 0.5, 0.0, 0.0;
    k_d << 0.0, 0.0, 0.0, 0.0;
    max_i << 0.1, 0.1, 0.0, 0.0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;
    error_i << 0, 0, 0, 0;

    move_command << 0, 0, 0, 0;
    keyboard_command << 0, 0, 0, 0;
    skycontroller_command << 0, 0, 0, 0;
    desired_velocity << 0, 0, 0, 0;
    desired_attitude << 0, 0, 0, 0;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);
}

// Destructor
SafeAnafi::~SafeAnafi(){
    ROS_INFO("SafeAnafi is stopping...");
    std_msgs::Empty empty_msg;
    olympe_bridge::ParrotCommand rpyg_msg;
    rpyg_publisher.publish(rpyg_msg);
    rpyg_publisher.publish(rpyg_msg);
    land_publisher.publish(empty_msg);
    land_publisher.publish(empty_msg);
    land = true;

    ros::shutdown();
    exit(0);
}

void SafeAnafi::run(){
    ROS_INFO("SafeAnafi is running...");

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setSafeAnafiConfig> server;
    dynamic_reconfigure::Server<controllers::setSafeAnafiConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);

    while(ros::ok()){
        ros::spinOnce();

        geometry_msgs::TwistStamped move_msg;
        olympe_bridge::ParrotCommand rpyg_msg;

        if(skycontroller_command(0) != 0 || skycontroller_command(1) != 0 || skycontroller_command(2) != 0 || skycontroller_command(3) != 0)
            move_command = skycontroller_command;
        else
            if(keyboard_command(0) != 0 || keyboard_command(1) != 0 || keyboard_command(2) != 0 || keyboard_command(3) != 0)
                move_command = keyboard_command;
            else{
                if(controller == 1) // velocity control
                        move_command << desired_velocity;
                if(controller == 2) // attitude control
                        move_command << desired_attitude;
            }

        geometry_msgs::TwistStamped desired_velocity_msg;
        desired_velocity_msg.header.stamp = ros::Time::now();
        desired_velocity_msg.twist.linear.x = move_command(0);
        desired_velocity_msg.twist.linear.y = move_command(1);
        desired_velocity_msg.twist.linear.z = move_command(2);
        desired_velocity_msg.twist.angular.z = move_command(3);

        if(//position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > 0 &&
                marker_visibile){
            switch(controller){
            case 0: // pose
                move_msg.twist.linear.x = 0;
                move_msg.twist.linear.y = 0;
                move_msg.twist.linear.z = 0;
                move_msg.twist.angular.x = 0;
                move_msg.twist.angular.y = 0;
                move_msg.twist.angular.z = 0;
                break;
            case 1: // velocity
                error = move_command - velocity;
                //error = move_command; // FOR DEMO

                error_i += error*dt;
                error_i << bound(error_i(0), max_i(0)), bound(error_i(1), max_i(1)), bound(error_i(2), max_i(2)), bound(error_i(3), max_i(3));
                error_d = (error - error_old)/dt;
                error_old = error;

                rpyg_msg.roll = -(k_p(1)*error(1) + k_i(1)*error_i(1) + k_d(1)*error_d(1));
                rpyg_msg.pitch =   k_p(0)*error(0) + k_i(0)*error_i(0) + k_d(0)*error_d(0);
                rpyg_msg.yaw = move_command(3);
                rpyg_msg.gaz = move_command(2);

                desired_velocity_publisher.publish(desired_velocity_msg);
                break;
            case 2: // attitude
                rpyg_msg.roll = move_command(0);
                rpyg_msg.pitch = move_command(1);
                rpyg_msg.gaz = move_command(3);
                rpyg_msg.gaz = move_command(2);
                break;
            }
            if(global && (controller == 1 || controller == 2)){ // rotate to global frame
                double roll = rpyg_msg.roll;
                double pitch = rpyg_msg.pitch;
                double relative_yaw = yaw - initial_yaw;
                rpyg_msg.roll = cos(relative_yaw)*roll + sin(relative_yaw)*pitch; // inverse rotation (from body to world)
                rpyg_msg.pitch = -sin(relative_yaw)*roll + cos(relative_yaw)*pitch; // inverse rotation (from body to world)
            }
        }
        //if(!land) // to prevent sending commands while landing
            switch(controller){
            case 0: // pose
                move_msg.header.stamp = ros::Time::now();
                move_msg.header.frame_id = "/body";
                move_publisher.publish(move_msg);
                break;
            case 1: // velocity
            case 2: // attitude
                rpyg_msg.header.stamp = ros::Time::now();
                rpyg_msg.header.frame_id = "/body";
                rpyg_publisher.publish(rpyg_msg);
                break;
            }
        rate.sleep();
    }
}

int main(int argc, char** argv){
    SafeAnafi* anafi = new SafeAnafi(argc, argv);
    anafi->run();
}
