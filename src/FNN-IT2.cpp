/****************************** FNN-IT2.cpp ***************************
 *
 * This code is the implementation of a type-2 fuzzy neural network controller.
 * It receives the actual and desired poses of the UAV and computes the control signal to the UAV.
 * The user can chose the learning rate (alpha) and the update of the learning rates (gamma) from the GUI.
 *
 * ***************************************************************/

#include "controllers/FNN-IT2.h"

/* UAV odometry callback function
 * Subscribes to UAV state estimates, creates and publishes odometry message
 */
void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, yaw;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[FNN_IT2] yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

/* Reference trajectory callback function
 * Subscribes to the commanded trajectory topic
 */
void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[FNN_IT2] position: " << pose_d.transpose() << endl;
}

/* Reference trajectory velocity callback function
 * Subscribes to the commanded trajectory velocity topic
 */
void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[FNN_IT2] velocity_d: " << velocity_d.transpose() << endl;
}

/* PD velocity callback function
 * Subscribes to the PD velocity command topic
 */
void pdCallback(const geometry_msgs::Quaternion& pd_msg){
    u_c << pd_msg.x, pd_msg.y, pd_msg.z, pd_msg.w;
    //cout << "[FNN_IT2] tau_c: " << tau_c.transpose() << endl;
}

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for fnn_type, learning & update rates from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setFNN_IT2Config &config, uint32_t level){
    alpha << config.alpha, config.alpha, config.alpha, config.alpha;
    gamma1 << config.gamma, config.gamma, config.gamma, config.gamma;
}

// Constructor
FNN_IT2::FNN_IT2(int argc, char** argv){
    ros::init(argc, argv, "FNN_IT2");
    ros::NodeHandle node_handle;

	// Subscribers
    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback); // for UAV's state
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback); // for desired trajectory
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback); // for desired velocity
    pd_subscriber = node_handle.subscribe("/uav/command_velocity_pd", 1, pdCallback); // for the command from PD controller

	// Publishers
    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity_fnn", 1); // for the output from FNN (for debug)
    fnn_velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/fnn/command_velocity", 1); // for the command signal
    fnn_params_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/fnn/params", 1); // for learning rates in FNN (for debug)
	
	// Initializes the actual and desired states
    pose << 0, 0, 0, 0; // actual pose
    pose_d << 0, 0, 0, 0; // desired pose
    velocity << 0, 0, 0, 0; // actual velocity
    velocity_d << 0, 0, 0, 0; // desired velocity

	// Initialize parameters
    if(argc > 1){
        alpha << atof(argv[1]), atof(argv[1]), atof(argv[1]), atof(argv[1]); // learning rates
        gamma1 << atof(argv[2]), atof(argv[2]), atof(argv[2]), atof(argv[2]); // updates of the learning rates
    }
    else{
        alpha << ALPHA, ALPHA, ALPHA, ALPHA; // learning rates
        gamma1 << GAMMA, GAMMA, GAMMA, GAMMA; // updates of the learning rates
    }

	// Initializes the variables of the elliptic membership functions (check paper [6] for these parameters)
    a11 << A1, A1, A1, A1; 
    a12 << A2, A2, A2, A2;
    a21 << A1, A1, A1, A1;
    a22 << A2, A2, A2, A2;
    c1 << C, C, C, C;
    c2 << C, C, C, C;
    d1 << D, D, D, D;
    d2 << D, D, D, D;
    q << 0.5, 0.5, 0.5, 0.5;

	// Initializes the weights of the NN
    for(int i = 0; i < 9; ++i)
        f.col(i) << ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000;
}

// Destructor
FNN_IT2::~FNN_IT2(){
    ros::shutdown();
    exit(0);
}

/* Core function
 * Reads the data, computes the inputs to the NNs and publishes the outputs from the NNs
 */
void FNN_IT2::run(){
	// Sets up the dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setFNN_IT2Config> server;
    dynamic_reconfigure::Server<controllers::setFNN_IT2Config>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

	// Type of the controller: 1 - position controller, 2 - velocity controller, 3 - attitude controller
    int controller_type;

    double time = 0;
    int c = 0;

	// Core loop
    double dt = (double)1/1000;
    ros::Rate rate(1000);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        ros::param::get("/safe_y6/controller", controller_type);

		// Check if a new odometry has arrived ... only then calculate the error
        if(controller_type != 0 && new_odometry){ // command
            e = pose_d - pose;
            e_d = velocity_d - velocity;

			// Updates the weights in the FNNs
            u_f(0) = update(0, e(0), e_d(0), u_c(0), dt);
            u_f(1) = update(1, e(1), e_d(1), u_c(1), dt);
            u_f(2) = update(2, e(2), e_d(2), u_c(2), dt);
            u_f(3) = update(3, e(3), e_d(3), u_c(3), dt);

            //cout << "[FNN_IT2] tau_f: " << tau_f.transpose() << endl;

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
            fnn_velocity_publisher.publish(velocity_msg);

			// Publishes the update rates (for debugging)
            velocity_msg.x = alpha(0);
            velocity_msg.y = alpha(1);
            velocity_msg.z = alpha(2);
            velocity_msg.w = alpha(3);
            fnn_params_publisher.publish(velocity_msg);

            //cout << "[FNN-IT2]: time = " << (time/c) << endl;
        }

        new_odometry = false;
    }
}

/* Update function
 * Updates the weights in the FNN and computes the outputs from FNN 
 */
double FNN_IT2::update(short axis, double x1, double x2, double u_c, double dt){
	// Bounds the input variables
    x1 = bound(x1, -1, 1); // position error
    x2 = bound(x2, -1, 1); // velocity error

	// Elliptic fuzzification of the inputs
    for(int i = 0; i < 3; ++i){
        mu1_upper(i) = elliptic(a11(axis, i), c1(axis, i), d1(axis, i), x1);
        mu1_lower(i) = elliptic(a12(axis, i), c1(axis, i), d1(axis, i), x1);
        mu2_upper(i) = elliptic(a21(axis, i), c2(axis, i), d2(axis, i), x2);
        mu2_lower(i) = elliptic(a22(axis, i), c2(axis, i), d2(axis, i), x2);
    }

	// Firing levels
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j){
            W_upper(3 * i + j) = mu1_upper(i) * mu2_upper(j);
            W_lower(3 * i + j) = mu1_lower(i) * mu2_lower(j);
        }

	// Normalization
    W_upper_bar = W_upper / W_upper.sum();
    W_lower_bar = W_lower / W_lower.sum();

	// Update laws for the consequent part (weights and learning rates in the FNN)
    Pi = q(axis) * W_lower_bar + (1 - q(axis)) * W_upper_bar;
    f_d = -Pi/(Pi.transpose() * Pi) * alpha(axis) * sign(u_c);
    alpha_d = 2 * gamma1(axis) * abs(u_c);
    q_d = 0;//-1.0/(f(axis) * (W_lower_bar - W_upper_bar).transpose()) * alpha(axis) * abs(u_c);

	// Updates the variables
    f.row(axis) += f_d * dt;
    alpha(axis) += alpha_d * dt;
    q(axis) += q_d * dt;

	// Bounds the variables
    f.row(axis) = bound(f.row(axis), -10, 10);
    alpha(axis) = bound(alpha(axis), 0, 10);
    q(axis) = bound(q(axis), 0, 1);

	// Computes the output
    return (q(axis) * f.row(axis) * W_lower_bar + (1 - q(axis)) * f.row(axis) * W_upper_bar)(0);
}

/* Sign function
 * Returns the sign of a number x, i.e., y = sing(x)
 */
short FNN_IT2::sign(double x){
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

/* Gaussian function
 * Returns the value of the Gaussian function with center c and variance sigma, i.e., y = Gaussian(x)
 */
double FNN_IT2::Gaussian(double c, double sigma, double x){
    return exp(-pow(x - c, 2)/pow(sigma, 2));
}

/* Elliptic function
 * Returns the value of the elliptic function, i.e., y = elliptic(x)
 */
double FNN_IT2::elliptic(double a, double c, double d, double x){
    if(c - d < x && x < c + d)
        return pow(1 - pow(abs((x - c) / d), a), 1.0/a);
    return 0;
}

/* Bound function
 * Bounds a number x between min and max values
 */
double FNN_IT2::bound(double v, double min, double max){
    return (v < min) ? min : ((v > max) ? max : v);
}

/* Bound vector function
 * Bounds all numbers in a vector v between min and max values
 */
VectorXd FNN_IT2::bound(VectorXd v, double min, double max){
    for(int i = 0; i < v.size(); ++i)
        v(i) = bound(v(i), min, max);
    return v;
}

// Main function
int main(int argc, char** argv){
    cout << "[FNN_IT2] FNN_IT2 position controller is running..." << endl;

    FNN_IT2* controller = new FNN_IT2(argc, argv);

    controller->run();
}
