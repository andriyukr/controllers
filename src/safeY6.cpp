/** *************************** safeY6.cpp ***************************
 *
 * This code collects all the messages:
 *      - Feedback from the UAV: state estimates for odometry
 *      - Reference trajectory: desired position and velocity from the trajectory generator
 *      - UAV commands: position, velocity and attitude commands from PID or ANN/FNN-PD
 *      - Keyboard input commands: commands from user by keyboard input
 *      - Constraints and safety: contains constraints and safety bounds for commands
 *      - Controller: lets the user choose between position/velocity/attitude commands
 *
 * ******************************************************************/

#include "controllers/safeY6.h"


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
 * Subscribes to the values set by user for controller type and thrust multiplier from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setSafeY6Config &config, uint32_t level){
    controller = config.controller;
    thrust_multiplier = config.thrust_multiplier; // The hover thrust for the UAV will be (1/thrust_multiplier)*100%
}

/* UAV odometry callback function
 * Subscribes to UAV state estimates, creates and publishes odometry message
 */
void optitrackCallback(const geometry_msgs::PoseStamped& optitrack_msg){
    Duration dt = optitrack_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = optitrack_msg.header.seq;
    odometry_msg.header.stamp = optitrack_msg.header.stamp;
    odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
    odometry_msg.child_frame_id = "Y6";

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

}

// Function to bound the values of "v" between +/- "b"
float bound(float v, float b){
    if(v < -b)
        return -b;
    if(v > b)
        return b;
    return v;
}

// Function to bound command thrust between 0 and 1
float bound_T(float t){
    if(t < 0)
        return 0;
    if(t > 1)
        return 1;
    return t;
}

mavros_msgs::SetMode srv_setMode;
/* UAV command callback function
 * Subscribes to user keyboard input command
 */
void commandCallback(const std_msgs::Int8& command_msg){
    geometry_msgs::TwistStamped velocity_msg;
	velocity_msg.header.stamp = ros::Time::now();;
    switch(command_msg.data){
    case 1: // hover
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);
        break;
    case 2: // arm
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);
   		arm_cmd.request.value = true;
        arming_client.call(arm_cmd);
        stop = false;
        break;
    case 3: // land
        srv_setMode.request.custom_mode = "AUTO.LAND";
        set_mode_client.call(srv_setMode);
        stop = true;
        break;
    case 4: // disarm
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);
   		arm_cmd.request.value = false;
		arming_client.call(arm_cmd);
    stop = true;
    }
}

/* Reference trajectory callback function
 * Subscribes to the commanded trajectory topic
 */
void commandPositionCallback(const geometry_msgs::QuaternionStamped& command_msg){
    float x = bound(command_msg.quaternion.x, MAX_X);
    float y = bound(command_msg.quaternion.y, MAX_Y);
    float z = bound(command_msg.quaternion.z, MAX_Z);
    float psi = bound(command_msg.quaternion.w, M_PI);

    position_d << x, y, z, psi;
}

/* Command velocity callback function
 * Subscribes to the commanded velocity topic
 */
void commandVelocityCallback(const geometry_msgs::Quaternion& command_msg){
    float vx = bound(command_msg.x, MAX_V);
    float vy = bound(command_msg.y, MAX_V);
    float vz = bound(command_msg.z, MAX_V);
    float r = bound(command_msg.w, MAX_V);

    velocity_d << vx, vy, vz, r;
}

/* Command attitude callback function
 * Subscribes to the commanded attitude angles topic
 */
void commandAttitudeCallback(const geometry_msgs::Quaternion& command_msg){
    float roll = bound(-command_msg.y, MAX_RP);
    float pitch = bound(command_msg.x, MAX_RP);
    float yaw = bound(command_msg.w, M_PI);
    float thrust = bound(command_msg.z, MAX_V);

    attitude_d << roll, pitch, yaw, thrust;
}

/* External noise callback function
 * Subscribes to the noise topic
 */
void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

/* Battery status callback function
 * Subscribes to the battery status mavros topic
 */
void batteryCallback(const sensor_msgs::BatteryStateConstPtr& battery_msg){
    if(battery_msg->percentage < 0.1){
        stop = true;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);
        land_cmd.request.yaw = yaw;
        land_cmd.request.latitude = position_d(0);
        land_cmd.request.longitude = position_d(1);
        land_cmd.request.altitude = 0;
        land_client.call(land_cmd);
    }
}

/* Flight mode / State callback function
 * Subscribes to the UAV flight mode / State mavros topic
 */
void stateCallback(const mavros_msgs::State::ConstPtr& state_msg){
    current_state = *state_msg;
    if (current_state.mode == "ALTCTL")
    {
        srv_setMode.request.custom_mode = "ALTCTL";
        set_mode_client.call(srv_setMode);
    }
}

/* Local position callback function
 * Subscribes to the local position estimates topic
 */
geometry_msgs::PoseStamped local_pos;
void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
}

/* Local velocity callback function
 * Subscribes to the local velocity estimates topic
 */
geometry_msgs::TwistStamped actual_vel;
void actualVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr actual_msg){
    actual_vel = *actual_msg;
}

mavros_msgs::PositionTarget raw_target_pos_msg;
mavros_msgs::AttitudeTarget raw_target_att_msg;

// Function checks the RC remote for availability
void check_remote_input(){
    if(RCinputs.channels.size()>0){
        RCavailable=true;
    }
    else{
        RCavailable=false;
    }
}

// Constructor
SafeY6::SafeY6(int argc, char** argv){
    ros::init(argc, argv, "safeY6");
    ros::NodeHandle node_handle;

    // Subscribers
    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    command_subscriber = node_handle.subscribe("/y6/command", 1, commandCallback);
    command_position_subscriber = node_handle.subscribe("/y6/command_position", 1, commandPositionCallback);
    command_velocity_subscriber = node_handle.subscribe("/y6/command_velocity", 1, commandVelocityCallback);
    command_attitude_subscriber = node_handle.subscribe("/y6/command_attitude", 1, commandAttitudeCallback);
    noise_subscriber = node_handle.subscribe("/y6/noise", 1, noiseCallback);
    battery_subscriber = node_handle.subscribe("/mavros/battery", 1, batteryCallback);
    state_subsriber = node_handle.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    velocity_actual_subscriber = node_handle.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity",1,actualVelocityCallback);
    local_pos_sub = node_handle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, localPosCallback);

    // Publishers
    position_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
    attitude_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/target_attitude", 1);
    throttle_publisher = node_handle.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/y6/ground_truth/odometry", 1);
    noisy_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/y6/odometry", 1);
    test_msg_publish = node_handle.advertise<std_msgs::Float64>("/test_thrust",1);
    att_com_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/attitude_RPY/command", 1);
    att_local_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/attitude_RPY/local", 1);

    raw_pos_com_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    raw_att_com_publisher = node_handle.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

    // ROS service clients
    set_mode_client = node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client = node_handle.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");


    // Initializing parameters
    position.x = 0;
    position.y = 0;
    position.z = 0;

    position_d << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;
    attitude_d << 0, 0, 0, 0;

    yaw = 0;

    stop = true;

    controller = 0;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);

}

// Destructor
SafeY6::~SafeY6(){
    stop = true;

    ros::Rate rate(10);
    rate.sleep();

    ros::shutdown();
    exit(0);
}

void SafeY6::run(){

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setSafeY6Config> server;
    dynamic_reconfigure::Server<controllers::setSafeY6Config>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    long count = 0;

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped att_com_eul;
    geometry_msgs::PoseStamped att_local_eul;
    double roll_local, pitch_local, yaw_local;

    //send a few setpoints before starting
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    for(int i = 100; ros::ok() && i > 0; --i){
        position_publisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    std_msgs::Float64 test_thrust;
    double roll;
    double pitch;
    double yaw;
    double thrust;

    ros::Time last_request = ros::Time::now();

    // Main loop
    while(ros::ok()){

        rate.sleep();
        ros::spinOnce();


        // Real system
        float m = 2.206;
        float min_thrust = 0; // Min thrust [0 N] - worst case / free fall
        float max_thrust = thrust_multiplier * m * 9.81;

        check_remote_input();
        // Check if the state is not ALTCTL/OFFBOARD/AUTO.LAND and wait for 2 sec - only then request for OFFBOARD mode from flight controller
        if (current_state.mode != "ALTCTL" && current_state.mode != "OFFBOARD" && current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(2)))
        {
            //sleep(0.5);
            offb_set_mode.request.custom_mode = "OFFBOARD";
            set_mode_client.call(offb_set_mode);
            last_request = ros::Time::now();
        }


        // Publish the local_attitude angles to topic /attitude_RPY/local
        tf::Quaternion q_local(local_pos.pose.orientation.x, local_pos.pose.orientation.y, local_pos.pose.orientation.z, local_pos.pose.orientation.w);
        tf::Matrix3x3 m_local(q_local);
        m_local.getRPY(roll_local, pitch_local, yaw_local);

        att_local_eul.header.stamp = ros::Time::now();
        att_local_eul.header.seq = count;

        att_local_eul.pose.orientation.x = roll_local * (180/M_PI);
        att_local_eul.pose.orientation.y = pitch_local * (180/M_PI);
        att_local_eul.pose.orientation.z = yaw_local  * (180/M_PI);
        att_local_eul.pose.orientation.w = 0;
        att_local_publisher.publish(att_local_eul);


        geometry_msgs::PoseStamped position_msg;
        geometry_msgs::TwistStamped velocity_msg;
        geometry_msgs::PoseStamped attitude_msg;
        mavros_msgs::Thrust throttle_msg;

        /* ********************** Main Controller **********************
         *
         * Each controller has an if-else condition:
         * if the UAV moves out of bounds (MAX_X, MAX_Y, MAX_Z) or the flag "stop" becomes false then
         * UAV tries to hover at the last commanded position.
         *
         * ************************************************************/


        // ********************** Position setpoint **********************
        if(controller == 0){
            position_msg.header.stamp = ros::Time::now();
            position_msg.header.seq = count;
            position_msg.header.frame_id = "local_origin";

            // If the UAV goes out of the defined safety boundary ... set the desired conditions in the else [Change the coundary in header file]
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){
                // Send the position commands as it is from the trajectory topic.

                double x = position_d(0);
                double y = position_d(1);
                double z = position_d(2);
                double psi  = position_d(3);

                position_msg.pose.position.x = x;
                position_msg.pose.position.y = y;
                position_msg.pose.position.z = z;
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, psi);
                quaternionTFToMsg(q, position_msg.pose.orientation);
            }
            else{
                position_msg.pose.position.x = 0;
                position_msg.pose.position.y = 0;
                position_msg.pose.position.z = 0;
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
                quaternionTFToMsg(q, position_msg.pose.orientation);
            }

            position_publisher.publish(position_msg);
        }

        // ********************** Velocity setpoint **********************
        else if(controller == 1){
            velocity_msg.header.stamp = ros::Time::now();
            velocity_msg.header.seq = count;
            velocity_msg.header.frame_id = "local_origin";

            // If the UAV goes out of the defined safety boundary ... set the desired conditions in the else [Change the coundary in header file]
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){
                // Send the velocity commands as it is from the trajectory topic.


                double vx = velocity_d(0);
                double vy = velocity_d(1);
                double vz = velocity_d(2);
                double r  = velocity_d(3);

                velocity_msg.twist.linear.x = vx;
                velocity_msg.twist.linear.y = vy;
                velocity_msg.twist.linear.z = vz;
                velocity_msg.twist.angular.z = r;
            }
            else{
                velocity_msg.twist.linear.x = 0;
                velocity_msg.twist.linear.y = 0;
                velocity_msg.twist.linear.z = 0;
            }

            velocity_publisher.publish(velocity_msg);
        }

        // ********************** Attitude setpoint **********************
        else if(controller == 2){
            attitude_msg.header.stamp = ros::Time::now();
            attitude_msg.header.seq = count;
            attitude_msg.header.frame_id = "local_origin";
            throttle_msg.header.stamp = ros::Time::now();
            throttle_msg.header.frame_id = "local_origin";

            // If the UAV goes out of the defined safety boundary ... set the desired conditions in the else [Change the coundary in header file]
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){

                /* Attitude angles command
                 *
                 * yaw = desired yaw command from the user
                 * roll/pitch = calculated from the velocity scalars from the controller and transformed into global frame [to compensate yaw] accordingly
                 * [the velocity scalars are limited in the header file ..... bound the final angles commanded here]
                 *
                 * */
                yaw = position_d(3);
                roll = bound((cos(yaw) * attitude_d(0) + sin(yaw) * attitude_d(1))/M_PI, 5*M_PI/18);  // based on max velocity and max attitude angles
                pitch = bound((cos(yaw) * attitude_d(1) - sin(yaw) * attitude_d(0))/M_PI, 5*M_PI/18); // bound to 45 deg [M_PI/4] or 50 deg [5M_PI/18]


                tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
                quaternionTFToMsg(q, attitude_msg.pose.orientation);


                // Publish the command_attitude angles to topic /attitude_RPY/command
                att_com_eul.header.stamp = ros::Time::now();
                att_com_eul.header.seq = count;

                att_com_eul.pose.orientation.x = roll * (180/M_PI);
                att_com_eul.pose.orientation.y = pitch * (180/M_PI);
                att_com_eul.pose.orientation.z = yaw * (180/M_PI);
                att_com_eul.pose.orientation.w = 0;

                att_com_publisher.publish(att_com_eul);


                /* Thust command calculated by the formula:
                 *
                 *                              1                                                             1            1
                 *      thrust = ((( ----------------------- ) * (m*g - min_thrust)) + command*scale) * ( --------- + ----------- - 1 )
                 *                   max_thrust - min_thrust                                              cos (phi)   cos (theta)
                 *
                */
                thrust = ((((1) / (max_thrust - min_thrust)) * (m*9.81 - min_thrust)) + attitude_d(3)/7) * (1/(cos(roll)) + 1/(cos(pitch)) - 1); // based on available throttle
                throttle_msg.thrust = bound_T(thrust);       // Thrust bound between (0 & 1)


                //Debug: Publish thrust on test_thrust topic
                test_thrust.data = bound_T(thrust);     // Bound thrust between 0 & 1
                test_msg_publish.publish(test_thrust);

            }
            else{
                // Attitude commands
                roll = attitude_d(0);
                pitch = attitude_d(1);
                yaw = position_d(3);

                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
                quaternionTFToMsg(q, attitude_msg.pose.orientation);

                // Thrust command
                thrust = (((1 / (max_thrust - min_thrust)) * (m*9.81 - min_thrust)) + attitude_d(3)/7) * (1/(cos(roll)) + 1/(cos(pitch)) - 1); // real --
                //throttle_msg.thrust = thrust;
                throttle_msg.thrust = bound_T(thrust);  // Thrust bound from (0 - 1)


                // Publish the command_attitude angles to topic /attitude_RPY/command
                att_com_eul.header.stamp = ros::Time::now();
                att_com_eul.header.seq = count;

                att_com_eul.pose.orientation.x = roll * (180/M_PI);
                att_com_eul.pose.orientation.y = pitch * (180/M_PI);
                att_com_eul.pose.orientation.z = yaw * (180/M_PI);
                att_com_eul.pose.orientation.w = 0;

                att_com_publisher.publish(att_com_eul);

            }

            attitude_publisher.publish(attitude_msg);
            throttle_publisher.publish(throttle_msg);

        }

        /* Hybrid controller: Attitude commands + Vertical velocity command
         *
         *             ------------ TESTING ---------------
         *
         * [works for bebop / currently unsupported on PX4 firmware]
         * */
        else if(controller == 3){
          // Position and velocity command
            raw_target_pos_msg.header.stamp = ros::Time::now();
            raw_target_pos_msg.header.seq = count;
            raw_target_pos_msg.header.frame_id = "local_origin";
            raw_target_pos_msg.coordinate_frame = 1;

            /* Comment the following masks if using them. Ex- if user wants to publish x-y-z position commands
             * comment those and uncomment the remaining masks. Then assign the desired value to variables and publish
             * */
            raw_target_pos_msg.type_mask = 1 |      // position PX
                                           2 |      //          PY
                                           4 |      //          PZ
                                           //8 |      // velocity VX
                                           //16 |     //          VY
                                           //32 |     //          VZ
                                           64 |     // acceleration AFX
                                           128 |    //              AFY
                                           256 |    //              AFZ
                                           512 |    // force in af vector
                                           //1024 |   // yaw
                                           2048     // yaw_rate
                                           ;
            // Attitude command
              raw_target_att_msg.header.stamp = ros::Time::now();
              raw_target_att_msg.header.seq = count;
              raw_target_att_msg.header.frame_id = "local_origin";
              raw_target_att_msg.type_mask = 1 |                           //rollrate
                                             2 |                           //pitchrate
                                             4 |                           //yawrate
                                             64 //|                          //thrust
                                             //128                           //attitude
                                             ;


              if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){

                  // Calculate the attitude angles
                  yaw = position_d(3);
                  roll = bound((cos(yaw) * attitude_d(0) + sin(yaw) * attitude_d(1))/M_PI, 5*M_PI/18);  // based on max velocity and max attitude angles
                  pitch = bound((cos(yaw) * attitude_d(1) - sin(yaw) * attitude_d(0))/M_PI, 5*M_PI/18); // bound to 45 deg [M_PI/4] or 50 deg [5M_PI/18]

                  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
                  quaternionTFToMsg(q, raw_target_att_msg.orientation);

                  // Thrust
                  //thrust = ((((1) / (max_thrust - min_thrust)) * (m*9.81 - min_thrust)) + attitude_d(3)/7) * (1/(cos(roll)) + 1/(cos(pitch)) - 1); // test [based on available throttle]
                  //raw_target_att_msg.thrust = bound_T(thrust);


                  // Publish the command_attitude angles to topic /attitude_RPY/command

                  att_com_eul.pose.orientation.x = roll * (180/M_PI);
                  att_com_eul.pose.orientation.y = pitch * (180/M_PI);
                  att_com_eul.pose.orientation.z = yaw * (180/M_PI);
                  att_com_eul.pose.orientation.w = 0;

                  att_com_publisher.publish(att_com_eul);


                  // Vertical velocity command (Vz)
                  double vx = velocity_d(0);
                  double vy = velocity_d(1);
                  double vz = velocity_d(2);

                  raw_target_pos_msg.velocity.x = vx;
                  raw_target_pos_msg.velocity.y = vy;
                  raw_target_pos_msg.velocity.z = vz;

                  /*raw_target_pos_msg.position.x = 0;
                  raw_target_pos_msg.position.y = 0;
                  raw_target_pos_msg.position.z = 1;*/

                  raw_target_pos_msg.yaw = yaw;

              }
              else{
                  yaw = position_d(3);

                  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
                  quaternionTFToMsg(q, raw_target_att_msg.orientation);

                  raw_target_pos_msg.velocity.z = 0;
              }

              raw_pos_com_publisher.publish(raw_target_pos_msg);
              //raw_att_com_publisher.publish(raw_target_att_msg);
        }

        ++count;

    }
}

// Main function
int main(int argc, char** argv){
    cout << "[SafeY6] SafeY6 is running..." << endl;

    SafeY6* y6 = new SafeY6(argc, argv);

    y6->run();
}
