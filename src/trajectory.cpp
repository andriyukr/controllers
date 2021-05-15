/** *************************** trajectory.cpp ***************************
 *
 * This code is the trajectory generator. It generates different trajectories
 * based on user input. The user can also select the speed of trajectory.
 * It publishes the desired position and velocity, and the type of the
 * respective trajectory.
 *
 * **********************************************************************/

#include "controllers/trajectory.h"


// ********************** Callbacks **********************

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for desired_yaw, trajectory_type, and speed from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setTrajectoryConfig &config, uint32_t level){
    last_yaw = config.yaw_d/180*M_PI;
    trajectory_type = config.trajectory;
    if(level == 0 || level == 1){
        waypoint = 0;
        if(trajectory_type == 6)
            t = M_PI/2;
        else
            if(trajectory_type == 7)
                t = 1;
            else
                t = 0;
    }

    speed = config.speed;
    scale = config.scale;

    pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d/180*M_PI;
    yaw_d = initial_local_yaw + pose_d(3);
}

/* Keyboard command callback function
 * Subscribes to the keyboard input "h" to instantly hover at current pos
 */
void commandCallback(const std_msgs::Int8& command_msg){
    if(command_msg.data == 5)
        trajectory_type = 11;
}

// Pose callback function
void poseCallback(const geometry_msgs::PoseStamped& pose_msg){
    tf::Quaternion q(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, yaw;
}

// Constructor
Trajectory::Trajectory(int argc, char** argv){
    ros::init(argc, argv, "Trajectory");
    ros::NodeHandle node_handle;
    
    // Subscribers
    command_subscriber = node_handle.subscribe("/uav/command", 1, commandCallback);
    pose_subscriber = node_handle.subscribe("/uav/pose", 1, poseCallback);

    // Publishers
    trajectory_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/uav/trajectory", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/uav/trajectory_velocity", 1);

    // Initializing parameters
    pose_d << 0, 0, 0, 0;
    trajectory_type = 0;
    speed = 1;
    scale = 1;
    straight_speed = 1;

    yaw_d = 0;

    std::string file_waypoints, file_waypoints_stamped;
    if(node_handle.getParam("/trajectory/file_waypoints", file_waypoints))
        readWaypoints(file_waypoints);
    if(node_handle.getParam("/trajectory/file_waypoints_stamped", file_waypoints_stamped))
        readWaypointsStamped(file_waypoints_stamped);
}

// Destructor
Trajectory::~Trajectory(){
    ros::shutdown();
    exit(0);
}

// Function to denormalize angles from -pi to pi
double Trajectory::denormalizeAngle(double a1){
    if(a1 > M_PI)
        a1 -= 2*M_PI;
    else if(a1 < -M_PI)
        a1 += 2*M_PI;
    return a1;
}

// Reads the file with waypoints
void Trajectory::readWaypoints(std::string fileName){
    string line;
    ifstream myfile(fileName);
    if(myfile.is_open()){       
        int points = 0;
        while(getline(myfile, line))
            ++points;
        waypoints = MatrixXd(points, 4);

        ifstream myfile(fileName);
        getline(myfile, line); // reads out the header
        waypoints.row(0) = pose;
        for(int i = 1; getline(myfile, line); ++i){
            string delimiter = "\t";
            size_t pos = 0;
            for(int j = 0; (pos = line.find(delimiter)) != string::npos; ++j) {
                waypoints(i, j) = atof(line.substr(0, pos).c_str());
                line.erase(0, pos + delimiter.length());
            }
            waypoints(i, 3) = atof(line.c_str())/180*M_PI;
        }
        myfile.close();

        ROS_INFO_STREAM("File with waypoints loaded: " << fileName);
        ROS_DEBUG_STREAM("waypoints:\n" << waypoints);
    }
    else
        ROS_WARN_STREAM("Unable to open file with waypoints: " << fileName);
}

// Reads the file with stamped waypoints and calculates the fitting curve
void Trajectory::readWaypointsStamped(std::string fileName){
    string line;
    ifstream myfile(fileName);
    if(myfile.is_open()){
        int points = 0;
        while(getline(myfile, line))
            ++points;
        MatrixXd waypoints = MatrixXd(points, 5);

        ifstream myfile(fileName);
        getline(myfile, line); // reads out the header
        waypoints.row(0) << 0, pose.transpose();
        for(int i = 1; getline(myfile, line); ++i){
            string delimiter = "\t";
            size_t pos = 0;
            for(int j = 0; (pos = line.find(delimiter)) != string::npos; ++j) {
                waypoints(i, j) = atof(line.substr(0, pos).c_str());
                line.erase(0, pos + delimiter.length());
            }
            waypoints(i, 4) = atof(line.c_str())/180*M_PI;
        }
        myfile.close();

        ROS_INFO_STREAM("File with stamped waypoints loaded: " << fileName);
        ROS_DEBUG_STREAM("waypoints stamped:\n" << waypoints);
        curveFitting(waypoints);
    }
    else
        ROS_WARN_STREAM("Unable to open file with stamped waypoints: " << fileName);
}

void Trajectory::curveFitting(MatrixXd waypoints){
    int points = waypoints.rows();
    curve_parameters_pose = MatrixXd(points, 4);
    curve_parameters_velocity = MatrixXd(points - 1, 4);
    curve_time = waypoints.leftCols(1);
    
    MatrixXd times = MatrixXd(points, points);
    for(int i = 0; i < points; ++i)
        for(int j = 0; j < points; ++j)
            times(i,j) = pow(waypoints(i,0), j);

    ROS_DEBUG_STREAM("times.inverse():\n" << times.completeOrthogonalDecomposition().pseudoInverse());
    ROS_DEBUG_STREAM("waypoints.rightCols(4):\n" << waypoints.rightCols(4));

    curve_parameters_pose << times.completeOrthogonalDecomposition().pseudoInverse()*waypoints.rightCols(4);

    ROS_DEBUG_STREAM("curve_parameters_pose:\n" << curve_parameters_pose);
}

void Trajectory::run(){
    /* Take initial yaw angle
     *
     * This loop waits for the estimator to stabilize and send the yaw values
     * This value is taken as the initial yaw reference [denormalized] for the UAV to always point in the intitial desired direction
     * The UAV can go +180 deg to -179 deg from this initial yaw
     *
     * */

    ROS_INFO("Trajectory is running...");

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setTrajectoryConfig> server;
    dynamic_reconfigure::Server<controllers::setTrajectoryConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    Vector4d trajectory;
    Vector4d velocity;
    t = 0;
    t_straight = 0;
    double R = sqrt(2)/2;
    Vector4d w1;
    Vector4d w2;
    VectorXd times = VectorXd(curve_parameters_pose.rows());

    double var_speed = 0;
    double d = 0;
    double x, y;

    double dt = (double)1/100;
    ros::Rate rate(100);
    // Main loop
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        // Denormalize yaw
        yaw_d = denormalizeAngle(yaw_d);

        Matrix3d R1;
        Matrix3d R2;

        // Main switch case
        switch(trajectory_type){
        case 0: // no command
            trajectory << 0, 0, -100, yaw_d;
            velocity << 0, 0, 0, 0;
            break;
        case 1: // hover
            trajectory << 0, 0, 1, yaw_d;
            velocity << 0, 0, 0, 0;
            break;
        case 2: // user
            trajectory << pose_d(0), pose_d(1), pose_d(2), yaw_d;
            velocity << 0, 0, 0, 0;
            break;
        case 3: // waypoints
            if(t == 0)
                ROS_INFO_STREAM("waypoint #" << waypoint << ": " << ROUND(trajectory));
            if(waypoint < waypoints.rows() - 1){
                w1 << waypoints.row(waypoint).transpose();
                w2 << waypoints.row(waypoint + 1).transpose();
                double d = distance(w1, w2);
                trajectory << (1 - t/d)*w1 + t/d*w2;
                velocity << (w2 - w1)/d*speed;
                if(t >= d){
                    t = 0;
                    waypoint++;
                    ROS_INFO_STREAM("waypoint #" << waypoint << ": " << w2.transpose());
                }
            }
            else{
                trajectory << waypoints.bottomRows(1).transpose();
                velocity << 0, 0, 0, 0;
            }
            break;
        case 4: // stamped waypoints
            if((float)t <= curve_time(curve_time.size() - 1)){

                for(int i = 0; i < curve_parameters_pose.rows(); ++i)
                    times(i) = pow(t, i);
                trajectory <<  curve_parameters_pose.transpose()*times;

                for(int i = 0; i < curve_parameters_pose.rows(); ++i) // analytical derivative of time
                    times(i) = i*pow(t, i - 1);
                velocity << curve_parameters_pose.transpose()*times;

                if((float)t == curve_time(curve_time.size() - 1))
                    velocity << 0, 0, 0, 0;

                if((float)t >= curve_time(waypoint))
                    ROS_INFO_STREAM("t = " << t << " -> waypoint #" << waypoint++ << ": " << ROUND(trajectory));

                t -= speed*dt;
                t += dt;
            }
            break;
        case 5: // Circle
            trajectory << scale*sin(t/scale) + pose_d(0), scale*cos(t/scale) + pose_d(1), pose_d(2), pose_d(3);
            velocity << speed*cos(t/scale), -speed*sin(t/scale), 0, 0;
            break;
        case 6: // smooth 8
            trajectory << sqrt(2)/2*4/(3 - cos(2*t))*cos(t) + pose_d(0), sqrt(2)/2*4/(3 - cos(2*t))*cos(t) + pose_d(1), 4/(3 - cos(2*t))*sin(2*t)/2 + pose_d(2) + 0.5, pose_d(3);
            velocity << sqrt(2)/2*speed*((4*sin(t))/(cos(2*t) - 3) - (8*sin(2*t)*cos(t))/pow(cos(2*t) - 3, 2)), sqrt(2)/2*speed*((4*sin(t))/(cos(2*t) - 3) - (8*sin(2*t)*cos(t))/pow(cos(2*t) - 3, 2)), 0, 0;
            break;
        case 7: // aggressive 8
            t = fmod(t, 4 + 2*M_PI*R);
            if(t < 2){
                trajectory << (R - R*t), (R - R*t), 0, -3.0/4*M_PI + 2*M_PI;
                velocity << -R, -R, 0, 0;
            }else
                if(t < 2 + M_PI*R){
                    trajectory << (-R - sin((t - 2)/R)*R), (-cos((t - 2)/R)*R), 0, M_PI/2;
                    velocity << -cos((t - 2)/R)*R, sin((t - 2)/R), 0, 0;
                }else
                    if(t < 4 + M_PI*R){
                        trajectory << (-R + R*(t - (2 + M_PI*R))), (R - R*(t - (2 + M_PI*R))), 0, -M_PI/4 + 2*M_PI;
                        velocity << R, -R, 0, 0;
                    }else
                        if(t < 4 + 2*M_PI*R){
                            trajectory << (R + sin((t - (4 + M_PI*R))/R)*R), (-cos((t - (4 + M_PI*R))/R)*R), 0, M_PI/2;
                            velocity << cos((M_PI*R - t + 4)/R), -sin((M_PI*R - t + 4)/R), 0, 0;
                        }
            trajectory = scale*trajectory;
            velocity = speed*velocity;
            R1 << cos(-M_PI/4), -sin(-M_PI/4), 0, sin(-M_PI/4), cos(-M_PI/4), 0, 0, 0, 1;
            R2 << cos(M_PI/18), 0, sin(M_PI/18), 0, 1, 0, -sin(M_PI/18), 0, cos(M_PI/18);
            trajectory.head(3) = R1*R2*trajectory.head(3);
            velocity.head(3) = R1*R2*velocity.head(3);
            trajectory(0) += pose_d(0);
            trajectory(1) += pose_d(1);
            trajectory(2) += pose_d(2);
            trajectory(3) = trajectory(3)/scale - M_PI/4;//pose_d(3);
            break;
        case 8: // square
            t = fmod(t, 8);
            if(t < 2){
                trajectory << 1, 1 - t, 0, 0;
                velocity << 0, -1, 0, 0;
            }
            else
                if(t < 4){
                    trajectory << 1 - (t - 2), -1, 0, 0;
                    velocity << -1, 0, 0, 0;
                }
                else
                    if(t < 6){
                        trajectory << -1, -1 + (t - 4), 0, 0;
                        velocity << 0, 1, 0, 0;
                    }
                    else
                        if(t < 8){
                            trajectory << -1 + (t - 6), 1, 0, 0;
                            velocity << 1, 0, 0, 0;
                        }
            trajectory = scale*trajectory;
            velocity = speed*velocity;
            R1 << cos(M_PI/18), 0, sin(M_PI/18), 0, 1, 0, -sin(M_PI/18), 0, cos(M_PI/18);
            trajectory.head(3) = R1*trajectory.head(3);
            velocity.head(3) = R1*velocity.head(3);
            trajectory(0) += pose_d(0);
            trajectory(1) += pose_d(1);
            trajectory(2) += pose_d(2);
            trajectory(3) = pose_d(3);
            break;
        case 9: // waypoints-square
            t = fmod(t, 4*scale);
            if(t < 1*scale)
                trajectory << 1, -1, 0.2, 0;
            else
                if(t < 2*scale)
                    trajectory << -1, -1, -0.2, 0;
                else
                    if(t < 3*scale)
                        trajectory << -1, 1, -0.2, 0;
                    else
                        if(t < 4*scale)
                            trajectory << 1, 1, 0.2, 0;
            trajectory = scale*trajectory;
            velocity << 0, 0, 0, 0;
            trajectory(0) += pose_d(0);
            trajectory(1) += pose_d(1);
            trajectory(2) += pose_d(2);
            trajectory(3) = pose_d(3);
            break;
        case 10: // circle with variable speed
            t -= speed*dt;
            t += dt;
            if(fmod(t, 2*M_PI) < M_PI/2)
                var_speed += 1.0/(M_PI/2)*dt;
            else if(fmod(t, 2*M_PI) < M_PI)
                var_speed = 1.0;
            else if(fmod(t, 2*M_PI) < 3*M_PI/2)
                var_speed += 1.0/(M_PI/2)*dt;
            else
                var_speed -= 2.0/(M_PI/2)*dt;
            d += var_speed*dt;

            trajectory << 2*sin(d), 2*cos(d), pose_d(2), pose_d(3);
            velocity << 2*var_speed*cos(d), -2*var_speed*sin(d), 0, 0;
            break;
        case 11: // circle with variable yaw
            trajectory << 2*sin(t), 2*cos(t), pose_d(2), pose_d(3) - t + M_PI;
            velocity << speed*cos(t), -speed*sin(t), 0, -speed;
            break;
        case 12: // Vertical circle
            // Uncomment to change yaw while performing circle
            //yaw_d = fmod(t, 2 * M_PI) - M_PI;

            trajectory << trajectory(0), 2*cos(t), pose_d(2) + 2*sin(t), yaw_d;
            velocity << 0, -2*speed*sin(t), 2*speed*cos(t), 0;
            break;
        }

        t += speed*dt;

        double v = sqrt(velocity(0)*velocity(0) + velocity(1)*velocity(1));

        // Publish the reference trajectory
        geometry_msgs::PoseStamped trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        trajectory_msg.pose.position.x = trajectory(0);
        trajectory_msg.pose.position.y = trajectory(1);
        trajectory_msg.pose.position.z = trajectory(2);
        trajectory_msg.pose.orientation.z = trajectory(3);
        trajectory_publisher.publish(trajectory_msg);

        // Publish the corresponding reference trajectory velocity
        geometry_msgs::TwistStamped velocity_msg;
        velocity_msg.header.stamp = ros::Time::now();
        velocity_msg.twist.linear.x = velocity(0);
        velocity_msg.twist.linear.y = velocity(1);
        velocity_msg.twist.linear.z = velocity(2);
        velocity_msg.twist.angular.z = velocity(3);
        velocity_publisher.publish(velocity_msg);
    }
}

// Function to calculate distance between two waypoints
double Trajectory::distance(Vector4d v1, Vector4d v2){
    return sqrt(pow(v1(0) - v2(0), 2) + pow(v1(1) - v2(1), 2) + pow(v1(2) - v2(2), 2));
}

int main(int argc, char** argv){
    Trajectory* controller = new Trajectory(argc, argv);
    controller->run();
}
