/** *************************** fnn.cpp ***************************
 *
 * This code is the implementation of a fuzzy neural network controller.
 * It receives the actual and desired poses of the UAV and computes the control signal to the UAV.
 * The user can select the type of update rules (sliding mode control or Levenberg–Marquardt),
 * and select the learning rate (alpha), the update of the learning rates (gamma) and 
 * the variation of the learning rates (nu) from the GUI.
 *
 * ***************************************************************/

#include "controllers/FNN.h"

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
    //cout << "[FNN] yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

/* Reference trajectory callback function
 * Subscribes to the commanded trajectory topic
 */
void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[FNN] position: " << pose_d.transpose() << endl;
}

/* Reference trajectory velocity callback function
 * Subscribes to the commanded trajectory velocity topic
 */
void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[FNN] velocity_d: " << velocity_d.transpose() << endl;
}

/* PD velocity callback function
 * Subscribes to the PD velocity command topic
 */
void pdCallback(const geometry_msgs::Quaternion& pd_msg){
    tau_c << pd_msg.x, pd_msg.y, pd_msg.z, pd_msg.w;
    //cout << "[FNN] tau_c: " << tau_c.transpose() << endl;
}

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for fnn_type, learning & update rates from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setFNNConfig &config, uint32_t level){
    fnn_type = config.fnn_type;
    alpha << config.alpha, config.alpha, config.alpha, config.alpha;
    gamma1 << config.gamma, config.gamma, config.gamma, config.gamma;
    nu << config.nu, config.nu, config.nu, config.nu;
}

// Constructor
FNN::FNN(int argc, char** argv){
    ros::init(argc, argv, "FNN");
    ros::NodeHandle node_handle;

    // Subscribers
    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback); // for UAV's state
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback); // for desired trajectory
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback); // for desired velocity
    pd_subscriber = node_handle.subscribe("/uav/command_velocity_pd", 1, pdCallback); // for the command from PD controller

    // Publishers
    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity_fnn", 1); // for the output from FNN (for debug)
    fnn_velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/fnn/command_velocity", 1); // for the command signal

	// Initializes the actual and desired states
    pose << 0, 0, 0, 0; // actual pose
    pose_d << 0, 0, 0, 0; // desired pose
    velocity << 0, 0, 0, 0; // actual velocity
    velocity_d << 0, 0, 0, 0; // desired velocity

    e_d_old = velocity_d - velocity;

    // Initialize parameters
    if(argc > 1){
        alpha << atof(argv[1]), atof(argv[1]), atof(argv[1]), atof(argv[1]); // learning rates
        gamma1 << atof(argv[2]), atof(argv[2]), atof(argv[2]), atof(argv[2]); // updates of the learning rates
        nu << atof(argv[3]), atof(argv[3]), atof(argv[3]), atof(argv[3]); // variation of the learning rates
    }
    else{
        alpha << ALPHA, ALPHA, ALPHA, ALPHA; // learning rates
        gamma1 << GAMMA, GAMMA, GAMMA, GAMMA; // updates of the learning rates
        nu << GAMMA, GAMMA, GAMMA, GAMMA; // variation of the learning rates
    }

	// FNN type: 1 - sliding mode control, 2 - Levenberg–Marquardt
    fnn_type = 1;

	// Initializes the variables of the Gaussian membership functions
    c1 << CENTER_E, CENTER_E, CENTER_E, CENTER_E; // centers for the first input
    c2 << CENTER_E_D, CENTER_E_D, CENTER_E_D, CENTER_E_D;  // centers for the second input
    sigma1 << VARIANCE_E, VARIANCE_E, VARIANCE_E, VARIANCE_E;  // variances for the first input
    sigma2 << VARIANCE_E_D, VARIANCE_E_D, VARIANCE_E_D, VARIANCE_E_D;  // variances for the second input

	// Initializes the weights of the NN
    for(int i = 0; i < 9; ++i)
        f.col(i) << ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000;

    //cout << "***** [FNN] alpha = " << alpha(0) << ", gamma1 = " << gamma1(0) << ", nu = " << nu(0) << endl;
}

// Destructor
FNN::~FNN(){
    ros::shutdown();
    exit(0);
}

/* Core function
 * Reads the data, computes the inputs to the NNs and publishes the outputs from the NNs
 */
void FNN::run(){

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setFNNConfig> server;
    dynamic_reconfigure::Server<controllers::setFNNConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

	// Type of the controller: 1 - position controller, 2 - velocity controller, 3 - attitude controller
    int controller_type;

    double dt = (double)1/1000;
    ros::Rate rate(1000);

    // Core loop
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        ros::param::get("/safe_y6/controller", controller_type);

        // Check if a new odometry has arrived ... only then calculate the error
        if(controller_type != 0 && new_odometry){ // if there is a command
            e = pose_d - pose;
            e_d = velocity_d - velocity;
            e_dd = (e_d - e_d_old) / dt;
            e_d_old = e_d;

            //cout << "[FNN] e = " << e(0) << ", e_d = " << e_d(0) << ", e_dd = " << e_dd(0) << ", tau_c = " << tau_c(0) << ", dt = " << dt << endl;

			// Computes the outputs from FNN and updates its weights
            if(fnn_type == 1){ // FNN with sliding mode control update rules
                tau_f(0) = SMC(0, e(0), e_d(0), e_d(0), e_dd(0), tau_c(0), dt);
                tau_f(1) = SMC(1, e(1), e_d(1), e_d(1), e_dd(1), tau_c(1), dt);
                tau_f(2) = SMC(2, e(2), e_d(2), e_d(2), e_dd(2), tau_c(2), dt);
                tau_f(3) = SMC(3, e(3), e_d(3), e_d(3), e_dd(3), tau_c(3), dt);
            }
            else if(fnn_type == 2){ // FNN with Levenberg–Marquardt update rules
                tau_f(0) = LM(0, e(0), e_d(0), e_d(0), e_dd(0), tau_c(0), dt);
                tau_f(1) = LM(1, e(1), e_d(1), e_d(1), e_dd(1), tau_c(1), dt);
                tau_f(2) = LM(2, e(2), e_d(2), e_d(2), e_dd(2), tau_c(2), dt);
                tau_f(3) = LM(3, e(3), e_d(3), e_d(3), e_dd(3), tau_c(3), dt);
            }
            else // no controller
                tau_f << 0, 0, 0, 0;

            //cout << "[FNN] tau_f: " << tau_f.transpose() << endl;

			// Publishes the control signal
            geometry_msgs::Quaternion velocity_msg;
            velocity_msg.x = tau_c(0) - tau_f(0);
            velocity_msg.y = tau_c(1) - tau_f(1);
            velocity_msg.z = tau_c(2) - tau_f(2);
            velocity_msg.w = tau_c(3) - tau_f(3);
            //velocity_msg.w = 0;//tau_c(3) - tau_f(3);
            velocity_publisher.publish(velocity_msg);

			// Publishes the output from ANN (for debugging)
            velocity_msg.x = -tau_f(0);
            velocity_msg.y = -tau_f(1);
            velocity_msg.z = -tau_f(2);
            velocity_msg.w = -tau_f(3);
            //velocity_msg.w = 0;
            fnn_velocity_publisher.publish(velocity_msg);
        }

        new_odometry = false;
    }
}

// Update rules for Sliding Mode Control
double FNN::SMC(short axis, double x1, double x1_d, double x2, double x2_d, double tau_c, double dt){
	// Bounds the input variables
    x1 = bound(x1, -1, 1); // position error
    x1_d = bound(x1_d, -1, 1); // derivative of the position error
    x2 = bound(x2, -1, 1); // velocity error
    x2_d = bound(x2_d, -1, 1); // derivative of the velocity error

	// Gaussian fuzzification of the inputs
    for(int i = 0; i < 3; ++i){
        mu1(i) = Gaussian(c1(axis, i), sigma1(axis, i), x1);
        mu2(i) = Gaussian(c2(axis, i), sigma2(axis, i), x2);
    }

	// Firing levels
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            W(3 * i + j) = mu1(i) * mu2(j);

	// Normalization
    W_bar = W / W.sum();

	// Update laws for the centers of the Gaussian membership functions
    c1_d << x1_d, x1_d, x1_d;
    c2_d << x2_d, x2_d, x2_d;

	// Update laws for the variances of the Gaussian membership functions
    for(int i = 0; i < 3; ++i){
            sigma1_d(i) = -pow(sigma1(axis, i), 3)/(pow(x1 - c1(axis, i), 2) + 0.001) * alpha(axis) * sign(tau_c);
            sigma2_d(i) = -pow(sigma2(axis, i), 3)/(pow(x2 - c2(axis, i), 2) + 0.001) * alpha(axis) * sign(tau_c);
    }
	
	// Update laws for the consequent part (weights in the FNN)
    f_d = -W_bar/(W_bar.transpose() * W_bar) * alpha(axis) * sign(tau_c);
	
	// Update law for learning rate
    alpha_d = gamma1(axis) * abs(tau_c) - gamma1(axis) * nu(axis) * alpha(axis);

	// Updates the variables
    c1.row(axis) += c1_d * dt;
    c2.row(axis) += c2_d * dt;
    sigma1.row(axis) += sigma1_d * dt;
    sigma2.row(axis) += sigma2_d * dt;
    f.row(axis) += f_d * dt;
    alpha(axis) += alpha_d * dt;

	// Bounding the system variables
    c1.row(axis) = bound(c1.row(axis), -1, 1);
    c2.row(axis) = bound(c2.row(axis), -1, 1);
    sigma1.row(axis) = bound(sigma1.row(axis), 0.001, 1);
    sigma2.row(axis) = bound(sigma2.row(axis), 0.001, 1);
    f.row(axis) = bound(f.row(axis), -10, 10);
    alpha(axis) = bound(alpha(axis), 0, 10);

	// Computes the output
    return (f.row(axis) * W_bar)(0);
}

double FNN::LM(short axis, double x1, double x1_d, double x2, double x2_d, double tau_c, double dt){
	// Bounds the input variables
    x1 = bound(x1, -1, 1); // position error
    x1_d = bound(x1_d, -1, 1); // derivative of the position error
    x2 = bound(x2, -1, 1); // velocity error
    x2_d = bound(x2_d, -1, 1); // derivative of the velocity error

	// Gaussian fuzzification of the inputs
    for(int i = 0; i < 3; ++i){
        mu1(i) = Gaussian(c1(axis, i), sigma1(axis, i), x1);
        mu2(i) = Gaussian(c2(axis, i), sigma2(axis, i), x2);
    }

	// Firing levels
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            W(3 * i + j) = mu1(i) * mu2(j);

	// Normalization
    W_bar = W / W.sum();

	// Update laws for the centers of the Gaussian membership functions
    c1_d << x1_d, x1_d, x1_d;
    c2_d << x2_d, x2_d, x2_d;

	// Update laws for the variances of the Gaussian membership functions
    for(int i = 0; i < 3; ++i){
            sigma1_d(i) = -pow(sigma1(axis, i), 3)/(pow(x1 - c1(axis, i), 2) + 0.001) * alpha(axis) * sign(tau_c);
            sigma2_d(i) = -pow(sigma2(axis, i), 3)/(pow(x2 - c2(axis, i), 2) + 0.001) * alpha(axis) * sign(tau_c);
    }
	
	// Update laws for the consequent part (weights in the FNN)
	float delta = 0;//max(W_bar.transpose() * W_bar, alpha);
	f_d = -gamma1 * (W_bar * W_bar.transpose() + delta * MatrixXd::Identity(3*3, 3*3)).inverse() * W_bar * sign(tau_c);
	
	// Update law for learning rate
    alpha_d = gamma1(axis) * abs(tau_c) - gamma1(axis) * nu(axis) * alpha(axis);

	// Updates the variables
    c1.row(axis) += c1_d * dt;
    c2.row(axis) += c2_d * dt;
    sigma1.row(axis) += sigma1_d * dt;
    sigma2.row(axis) += sigma2_d * dt;
    f.row(axis) += f_d * dt;
    alpha(axis) += alpha_d * dt;

	// Bounding the system variables
    c1.row(axis) = bound(c1.row(axis), -1, 1);
    c2.row(axis) = bound(c2.row(axis), -1, 1);
    sigma1.row(axis) = bound(sigma1.row(axis), 0.001, 1);
    sigma2.row(axis) = bound(sigma2.row(axis), 0.001, 1);
    f.row(axis) = bound(f.row(axis), -10, 10);
    alpha(axis) = bound(alpha(axis), 0, 10);

	// Computes the output
    return (f.row(axis) * W_bar)(0);
}

/* Sign function
 * Returns the sign of a number x, i.e., y = sing(x)
 */
short FNN::sign(double x){
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

/* Gaussian function
 * Returns the value of the Gaussian function with center c and variance sigma, i.e., y = Gaussian(x)
 */
double FNN::Gaussian(double c, double sigma, double x){
    return exp(-pow(x - c, 2)/pow(sigma, 2));
}

/* Bound function
 * Bounds a number x between min and max values
 */
double FNN::bound(double v, double min, double max){
    return (v < min) ? min : ((v > max) ? max : v);
}

/* Bound vector function
 * Bounds all numbers in a vector v between min and max values
 */
VectorXd FNN::bound(VectorXd v, double min, double max){
    for(int i = 0; i < v.size(); ++i)
        v(i) = bound(v(i), min, max);
    return v;
}

// Main function
int main(int argc, char** argv){
    cout << "[FNN] FNN position controller is running..." << endl;

    FNN* controller = new FNN(argc, argv);

    controller->run();
}
