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
 * TODO's:
 *      - Check velocities frame
 *      - Tune velocity controller
 *      - Filter accelerations
 *
 * *********************************************************************/

#include "controllers/safeAnafi.h"

Twist filter_mean_velocities(Twist v){
    velocities.erase(velocities.begin());
    velocities.push_back(v);

    Twist t;
    for(vector<Twist>::iterator i = velocities.begin(); i != velocities.end(); ++i){
        t.linear.x += i->linear.x/FILTER_SIZE;
        t.linear.y += i->linear.y/FILTER_SIZE;
        t.linear.z += i->linear.z/FILTER_SIZE;
        t.angular.x += i->angular.x/FILTER_SIZE;
        t.angular.y += i->angular.y/FILTER_SIZE;
        t.angular.z += i->angular.z/FILTER_SIZE;
    }
    return t;
}

Vector3d filter_mean_acceleration(Eigen::Ref<Eigen::VectorXd> a){
    accelerations.topRows(FILTER_SIZE - 1) = accelerations.bottomRows(FILTER_SIZE - 1);
    accelerations.row(FILTER_SIZE - 1) = a;

    return accelerations.colwise().sum()/FILTER_SIZE;
}

Vector3d filter_polinomial_acceleration(Eigen::Ref<Eigen::VectorXd> a){
    accelerations.topRows(FILTER_SIZE - 1) = accelerations.bottomRows(FILTER_SIZE - 1);
    accelerations.row(FILTER_SIZE - 1) = a;

    return accelerations.bottomRows(1);
}

// ********************** Callbacks **********************

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for controller type from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setSafeAnafiConfig &config, uint32_t level){
    controller = config.controller;
    global = config.global;

    k_p << config.k_p, config.k_p, 0, 0;
    k_i << config.k_i, config.k_i, 0, 0;
    k_d << config.k_d, config.k_d, 0, 0;
    max_i << config.max_i, config.max_i, 0, 0;
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
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
    odometry_msg.pose.pose.orientation = optitrack_msg.pose.orientation;

    Twist t;
    t.linear.x = (odometry_msg.pose.pose.position.x - position(0)) / (dt.toNSec() / pow(10, 9));
    t.linear.y = (odometry_msg.pose.pose.position.y - position(1)) / (dt.toNSec() / pow(10, 9));
    t.linear.z = (odometry_msg.pose.pose.position.z - position(2)) / (dt.toNSec() / pow(10, 9));
    t.angular.x = (roll - orientation(0)) / (dt.toNSec() / pow(10, 9));
    t.angular.y = (pitch - orientation(1)) / (dt.toNSec() / pow(10, 9));
    t.angular.z = (yaw - orientation(2)) / (dt.toNSec() / pow(10, 9));
    odometry_msg.twist.twist = filter_mean_velocities(t);
    odometry_publisher.publish(odometry_msg);

    position << odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z;
    orientation << roll, pitch, yaw;
    velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;
    rates << odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z;

    time_old = odometry_msg.header.stamp;
    marker_visibile = true;
}

void odometryCallback(const nav_msgs::Odometry odometry_msg){
    Duration dt = odometry_msg.header.stamp - time_old;

    tf::Quaternion q(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    if(initial_yaw == 1000*M_PI)
        initial_yaw = yaw;
    relative_yaw = yaw - initial_yaw;
        
    geometry_msgs::Vector3 euler_msg;
    euler_msg.x = roll/M_PI*180;
    euler_msg.y = pitch/M_PI*180;
    euler_msg.z = yaw/M_PI*180;
    euler_publisher.publish(euler_msg);
        
    velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;

    // TODO: replace with visual-inercial odometry
    position(0) += (cos(relative_yaw)*velocity(0) + sin(relative_yaw)*velocity(1))*(dt.toNSec()/pow(10, 9)); // simple dead reckoning sucks
    position(1) += (-sin(relative_yaw)*velocity(0) + cos(relative_yaw)*velocity(1))*(dt.toNSec()/pow(10, 9)); // simple dead reckoning sucks
    position(2) = odometry_msg.pose.pose.position.z; // barometer sucks
        
    acceleration << (velocity - velocity_old)/(dt.toNSec()/pow(10, 9));
    acceleration = filter_mean_acceleration(acceleration);

    velocity_old = velocity;            
    time_old = odometry_msg.header.stamp;
}

void commandMetaCallback(const std_msgs::Int8& command_msg){
    std_msgs::Empty empty_msg;
    olympe_bridge::PilotingCommand rpyg_msg;
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
    case 6: // reset position and heading
        ROS_INFO("Reseting position and heading");
        position << 0, 0, 0;
        initial_yaw = yaw;
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

void desiredPoseCallback(const geometry_msgs::PoseStamped& command_msg){
    desired_pose << BOUND(command_msg.pose.position.x, bounds(0,0), bounds(1,0)), BOUND(command_msg.pose.position.y, bounds(0,1), bounds(1,1)),
            BOUND(command_msg.pose.position.z, bounds(0,2), bounds(1,2)), command_msg.pose.orientation.z;
}

void desiredVelocityCallback(const geometry_msgs::TwistStamped& command_msg){
    ros::param::get("/anafi_bridge/max_vertical_speed", max_vertical_speed);
    desired_velocity << BOUND(command_msg.twist.linear.x, max_vertical_speed), BOUND(command_msg.twist.linear.y, max_vertical_speed), BOUND(command_msg.twist.linear.z, max_vertical_speed),
            command_msg.twist.angular.z;
}

void desiredAttitudeCallback(const geometry_msgs::TwistStamped& command_msg){
    desired_attitude << command_msg.twist.angular.x, command_msg.twist.angular.y, command_msg.twist.linear.z, command_msg.twist.angular.z;
}

void commandCameraCallback(const geometry_msgs::Twist& command_msg){
    gimbal_roll = command_msg.angular.z == 1 ? 0 : BOUND(gimbal_roll + command_msg.angular.x, GIMBAL_ROLL_MIN, GIMBAL_ROLL_MAX);
    gimbal_pitch = command_msg.angular.z == 1 ? 0 : BOUND(gimbal_pitch + command_msg.angular.y, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    zoom = BOUND(zoom + command_msg.linear.x, ZOOM_MIN, ZOOM_MAX);

    olympe_bridge::CameraCommand camera_msg;
    camera_msg.roll = gimbal_roll;
    camera_msg.pitch = gimbal_pitch;
    camera_msg.zoom = zoom;
    camera_msg.action = ((command_msg.linear.z == 1) << 0) | ((command_msg.linear.z == 2) << 1) | ((command_msg.linear.z == 3) << 2);
    camera_publisher.publish(camera_msg);
}

// Constructor
SafeAnafi::SafeAnafi(int argc, char** argv){
    ros::init(argc, argv, "safeBebop");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    odometry_subscriber = node_handle.subscribe("/anafi/odometry", 1, odometryCallback);
    command_subscriber = node_handle.subscribe("/keyboard/command_meta", 1, commandMetaCallback);
    command_keyboard_subscriber = node_handle.subscribe("/keyboard/command_move", 1, commandKeyboardCallback);
    command_camera_subscriber = node_handle.subscribe("/keyboard/command_camera", 1, commandCameraCallback);
    skycontroller_subscriber = node_handle.subscribe("/skycontroller/command", 1, commandSkycontrollerCallback);
    desired_pose_subscriber = node_handle.subscribe("/anafi/desired_pose", 1, desiredPoseCallback);
    desired_velocity_subscriber = node_handle.subscribe("/anafi/desired_velocity", 1, desiredVelocityCallback);
    desired_attitude_subscriber = node_handle.subscribe("/anafi/desired_attitude", 1, desiredAttitudeCallback);

    emergency_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/emergency", 1);
    takeoff_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/takeoff", 1);
    land_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/land", 1);
    offboard_publisher = node_handle.advertise<std_msgs::Bool>("/anafi/offboard", 1);
    move_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/anafi/cmd_move", 1);
    rpyg_publisher = node_handle.advertise<olympe_bridge::PilotingCommand>("/anafi/cmd_rpyt", 1);
    camera_publisher = node_handle.advertise<olympe_bridge::CameraCommand>("/anafi/cmd_camera", 1);
    euler_publisher = node_handle.advertise<geometry_msgs::Vector3>("/anafi/euler", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/anafi/ground_truth/odometry", 1);
    position_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("/debug/position", 1);
    desired_velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/debug/desired_velocity", 1);
    acceleration_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("/debug/acceleration", 1);

    k_p << 2.0, 2.0, 1.0, 1.0;
    k_i << 0.5, 0.5, 0.0, 0.0;
    k_d << 0.0, 0.0, 0.0, 0.0;
    max_i << 0.1, 0.1, 0.0, 0.0;

    ros::param::get("/safe_anafi/bounds/min_x", bounds(0,0));
    ros::param::get("/safe_anafi/bounds/min_y", bounds(0,1));
    ros::param::get("/safe_anafi/bounds/min_z", bounds(0,2));
    ros::param::get("/safe_anafi/bounds/max_x", bounds(1,0));
    ros::param::get("/safe_anafi/bounds/max_y", bounds(1,1));
    ros::param::get("/safe_anafi/bounds/max_z", bounds(1,2));
   
    Twist t; 
    velocities.clear();
    for(int i = 0; i < FILTER_SIZE; ++i)
        velocities.push_back(t);
}

// Destructor
SafeAnafi::~SafeAnafi(){
    ROS_INFO("SafeAnafi is stopping...");
    std_msgs::Empty empty_msg;
    olympe_bridge::PilotingCommand rpyg_msg;
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
        olympe_bridge::PilotingCommand rpyg_msg;

        if(!skycontroller_command.isZero())
            move_command = skycontroller_command;
        else
            if(!keyboard_command.isZero())
                move_command = keyboard_command;
            else{
                if(controller == 1) // velocity control
                        move_command << desired_velocity;
                if(controller == 2) // attitude control
                        move_command << desired_attitude;
            }

        geometry_msgs::Vector3Stamped position_msg; // FOR DEBUG
        position_msg.header.stamp = ros::Time::now(); // FOR DEBUG
        position_msg.vector.x = position(0); // FOR DEBUG
        position_msg.vector.y = position(1); // FOR DEBUG
        position_msg.vector.z = position(2); // FOR DEBUG
        position_publisher.publish(position_msg); // FOR DEBUG

        geometry_msgs::TwistStamped desired_velocity_msg; // FOR DEBUG
        desired_velocity_msg.header.stamp = ros::Time::now(); // FOR DEBUG
        desired_velocity_msg.twist.linear.x = move_command(0); // FOR DEBUG
        desired_velocity_msg.twist.linear.y = move_command(1); // FOR DEBUG
        desired_velocity_msg.twist.linear.z = move_command(2); // FOR DEBUG
        desired_velocity_msg.twist.angular.z = move_command(3); // FOR DEBUG
        
        geometry_msgs::Vector3Stamped acceleration_msg; // FOR DEBUG
        acceleration_msg.header.stamp = ros::Time::now(); // FOR DEBUG
        acceleration_msg.vector.x = acceleration(0); // FOR DEBUG
        acceleration_msg.vector.y = acceleration(1); // FOR DEBUG
        acceleration_msg.vector.z = acceleration(2); // FOR DEBUG
        acceleration_publisher.publish(acceleration_msg); // FOR DEBUG

        if(//position(0) < bounds(0,0) && position(0) > bounds(1,0) && position(1) < bounds(0,1) && position(1) > bounds(1,1) && position(2) < bounds(0,2) && position(2) > bounds(1,2) &&
                marker_visibile){

            if(global) // rotate command from world frame to body frame
                move_command << cos(relative_yaw)*move_command(0) + sin(relative_yaw)*move_command(1), -sin(relative_yaw)*move_command(0) + cos(relative_yaw)*move_command(1),
                        move_command(2), move_command(3);

            switch(controller){
            case 0: // pose
                /*pose_error = desired_pose.head(3) - position, desired_pose(3) - yaw;
                pose_error_i += pose_error*dt;
                pose_error_i << BOUND(pose_error_i(0), max_i(0)), BOUND(pose_error_i(1), max_i(1)), BOUND(pose_error_i(2), max_i(2));
                pose_error_d = desired_velocity.head(3) - velocity, desired_pose(3) - rates(2);

                rpyg_msg.roll = -(k_p(1)*pose_error(1) + k_i(1)*pose_error_i(1) + k_d(1)*pose_error_d(1)); // move_command(0)
                rpyg_msg.pitch =  k_p(0)*pose_error(0) + k_i(0)*pose_error_i(0) + k_d(0)*pose_error_d(0); // move_command(1)
                rpyg_msg.yaw =    k_p(2)*pose_error(3) + k_i(3)*pose_error_i(3) + k_d(3)*pose_error_d(3); // move_command(3)
                rpyg_msg.gaz =    k_p(2)*pose_error(2) + k_i(2)*pose_error_i(2) + k_d(2)*pose_error_d(2); // move_command(2)*/
                break; // TODO: CASCADE CONTROLLER
            case 1: // velocity
                velocity_error = move_command.head(3) - velocity;
                velocity_error_i += velocity_error*dt;
                velocity_error_i << BOUND(velocity_error_i(0), max_i(0)), BOUND(velocity_error_i(1), max_i(1)), BOUND(velocity_error_i(2), max_i(2));
                velocity_error_d = -acceleration;

                rpyg_msg.roll = -(k_p(1)*velocity_error(1) + k_i(1)*velocity_error_i(1) + k_d(1)*velocity_error_d(1)); // move_command(0)
                rpyg_msg.pitch =  k_p(0)*velocity_error(0) + k_i(0)*velocity_error_i(0) + k_d(0)*velocity_error_d(0); // move_command(1)
                rpyg_msg.yaw = move_command(3);
                rpyg_msg.gaz = move_command(2);

                desired_velocity_publisher.publish(desired_velocity_msg); // FOR DEBUG
                break; // TODO: CASCADE CONTROLLER
            case 2: // attitude
                rpyg_msg.roll = move_command(0);
                rpyg_msg.pitch = move_command(1);
                rpyg_msg.yaw = move_command(3);
                rpyg_msg.gaz = move_command(2);
                break;
            }
        }
        //if(!land) // to prevent sending commands while landing
            switch(controller){
            case 0: // pose
                //move_msg.header.stamp = ros::Time::now();
                //move_msg.header.frame_id = "/world";
                //move_publisher.publish(move_msg);
                //break;
            case 1: // velocity
            case 2: // attitude
                // TODO: bound commands here for the sliding control

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
