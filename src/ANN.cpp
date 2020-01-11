/** *************************** ann.cpp ***************************
 *
 * This code is the implementation of an artificial neural network.
 * It receives the actual and desired poses of the UAV and computes the control signal to the UAV.
 * The user can chose the learning rate (alpha) and the update of the learning rates (gamma) from the GUI.
 *
 * ***************************************************************/

#include "controllers/ANN.h"

/* UAV odometry callback function
 * Subscribes to UAV state estimates, creates and publishes odometry message
 */
void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;

    new_odometry = true;
}

/* Reference trajectory callback function
 * Subscribes to the commanded trajectory topic
 */
void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
}

/* Reference trajectory velocity callback function
 * Subscribes to the commanded trajectory velocity topic
 */
void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
}

/* PD velocity callback function
 * Subscribes to the PD velocity command topic
 */
void pdCallback(const geometry_msgs::Quaternion& pd_msg){
    u_c << pd_msg.x, pd_msg.y, pd_msg.z, pd_msg.w;
}

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for learning & update rates from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setANNConfig &config, uint32_t level){
    alpha << config.alpha, config.alpha, config.alpha, config.alpha;
    gamma1 << config.gamma, config.gamma, config.gamma, config.gamma;
	
    // Store the initial values to reset later
    alpha_0 << config.alpha, config.alpha, config.alpha, config.alpha;
    gamma1_0 << config.gamma, config.gamma, config.gamma, config.gamma;
}

// Constructor
ANN::ANN(int argc, char** argv){
    ros::init(argc, argv, "ANN");
    ros::NodeHandle node_handle;

    // Subscribers
    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback); // for UAV's state
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback); // for desired trajectory
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback); // for desired velocity
    pd_subscriber = node_handle.subscribe("/uav/command_velocity_pd", 1, pdCallback); // for the command from PD controller

    // Publishers
    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity_ann", 1); // for the output from ANN (for debug)
    ann_velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/ann/command_velocity", 1); // for the command signal
    ann_params_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/ann/params", 1); // for learning rates in ANN (for debug)

	// Initializes the actual and desired states
    pose << 0, 0, 0, 0; // actual pose
    pose_d << 0, 0, 0, 0; // desired pose
    velocity << 0, 0, 0, 0; // actual velocity
    velocity_d << 0, 0, 0, 0; // desired velocity

    // Initializes gains
    if(argc > 1){
        alpha << atof(argv[1]), atof(argv[1]), atof(argv[1]), atof(argv[1]); // learning rates
        gamma1 << atof(argv[2]), atof(argv[2]), atof(argv[2]), atof(argv[2]); // updates of the learning rates
    }
    else{
        alpha << ALPHA, ALPHA, ALPHA, ALPHA;  // learning rates
        gamma1 << GAMMA, GAMMA, GAMMA, GAMMA; // updates of the learning rates
    }

	// Initializes the weights of the NN
    for(int i = 0; i < 9; ++i)
        f.col(i) << ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000;
}

// Destructor
ANN::~ANN(){
    ros::shutdown();
    exit(0);
}

/* Core function
 * Reads the data, computes the inputs to the NNs and publishes the outputs from the NNs
 */
void ANN::run(){
    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setANNConfig> server;
    dynamic_reconfigure::Server<controllers::setANNConfig>::CallbackType g;
    g = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(g);

	// Type of the controller: 1 - position controller, 2 - velocity controller, 3 - attitude controller
    int controller_type;

    bool check_flag = 1;

    double dt = (double)1/1000;
    ros::Rate rate(1000);

    double time = ros::Time::now().toSec();
	
    // Core loop
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        ros::param::get("/safe_y6/controller", controller_type);

        // Checks if a new odometry has arrived ... only then calculate the control command
        if(controller_type != 0 && new_odometry){ // command
            e = pose_d - pose;
            e_d = velocity_d - velocity;

            // Re-initializes the learning rates & Resets ANN every reset_time secs
            if(ros::Time::now().toSec() - time > reset_time && reset_time != 0) {
              alpha = alpha_0;
              gamma1 = gamma1_0;
              for(int i = 0; i < 9; ++i)
                  f.col(i) << ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000;
//              check_flag = 0;
              time = ros::Time::now().toSec();
            }

			// Updates the weights in the NNs
            u_f(0) = update(0, e(0), e_d(0), u_c(0), dt); // x axis
            u_f(1) = update(1, e(1), e_d(1), u_c(1), dt); // y axis
            u_f(2) = update(2, e(2), e_d(2), u_c(2), dt); // z axis
            //u_f(3) = update(3, e(3), e_d(3), u_c(3), dt); // yaw axis

			// Publishes the control signal
            geometry_msgs::Quaternion velocity_msg;
            velocity_msg.x = u_c(0) - u_f(0);
            velocity_msg.y = u_c(1) - u_f(1);
            velocity_msg.z = u_c(2) - u_f(2);
            velocity_msg.w = 0;//u_c(3) - u_f(3);
            velocity_publisher.publish(velocity_msg);

			// Publishes the output from ANN (for debugging)
            velocity_msg.x = -u_f(0);
            velocity_msg.y = -u_f(1);
            velocity_msg.z = -u_f(2);
            velocity_msg.w = 0;
            ann_velocity_publisher.publish(velocity_msg);

			// Publishes the update rates (for debugging)
            velocity_msg.x = alpha(0);
            velocity_msg.y = alpha(1);
            velocity_msg.z = alpha(2);
            velocity_msg.w = alpha(3);
            ann_params_publisher.publish(velocity_msg);
        }

        new_odometry = false;
    }
}

/* Update function
 * Updates the weights in the NN and computes the outputs from NN 
 */
double ANN::update(short axis, double x1, double x2, double u_c, double dt){
	// Bounds the inputs
    x1 = bound(x1, -1, 1); // position error
    x2 = bound(x2, -1, 1); // velocity error

	// Computes the input neurons values
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            W(3 * i + j) = bound(x1 + x2, -1, 1);

    if (W.sum() != 0) { // if there is an error
		// Normalizes the input neurons
        W_bar = W / W.sum();

		// Computes the update of the weights and of the learning rates
        f_d = -W_bar/(W_bar.transpose() * W_bar) * alpha(axis) * sign(u_c);
        alpha_d = 2 * gamma1(axis) * abs(u_c);
		
		// Updates the weights and the learning rates
        f.row(axis) += f_d * dt;
        alpha(axis) += alpha_d * dt;

		// Bounds the weights and the learning rates
        f.row(axis) = bound(f.row(axis), -10, 10);
        alpha(axis) = bound(alpha(axis), 0, 10);
    }

	// Computes the output
    return f.row(axis) * W_bar;
}

/* Sign function
 * Returns the sign of a number x, i.e., y = sing(x)
 */
short ANN::sign(double x){
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

/* Bound function
 * Bounds a number x between min and max values
 */
double ANN::bound(double v, double min, double max){
    return (v < min) ? min : ((v > max) ? max : v);
}

/* Bound vector function
 * Bounds all numbers in a vector v between min and max values
 */
VectorXd ANN::bound(VectorXd v, double min, double max){
    for(int i = 0; i < v.size(); ++i)
        v(i) = bound(v(i), min, max);
    return v;
}

// Main function
int main(int argc, char** argv){
    cout << "[ANN] ANN position controller is running..." << endl;

    ANN* controller = new ANN(argc, argv);

    controller->run();
}
