/** *************************** data_record.cpp ***************************
 *
 * This codes subscribes to all the topics to be recorded and stores them in a text file
 * It tries to determine what type of data is being recorded - motor failure / fast flight / normal
 * It also knows which trajectory is being followed
 *
 * Then it renames the text file accordingly and saves it.
 *
 *************************************************************************/

#include<controllers/data_record.h>

// Set the maximum record time to 100s by default
double max_rec_time = 100;


// ********************** Callbacks **********************


/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for record_start, max_rec_time from the graphical interface
 */
void dynamicReconfigureCallback(controllers::setDataRecordConfig &config, uint32_t level)
{
    record_start = config.record_start;
    max_rec_time = config.max_rec_time;

    rec_start_flag.data = record_start;
}

/* Start data record callback function
 * Subscribes to the topic /record_start_flag - in case record_start is not defined in this code
 */
void recFlagCallback(const std_msgs::Int32::ConstPtr& rec_msg){
    if (rec_msg->data == 1)
        record_start = 1;
    else
        record_start = 0;
}

/* Reference trajectory type callback function
 * Subscribes to the trajectory type topic - stores the type of trajectory being followed from the integer.
 * This is used later to rename the file in accordance to the trajectory type.
 */
int traj_type;
void trajTypeCallback(const std_msgs::Int32::ConstPtr& traj_msg){
    traj_type = traj_msg->data;
}

/* Command thrust callback function
 * Subscribes to the command thrust topic - used to store the command thrust in case of attitude controller
 * Can also be used to debug in other codes
 */
double comThrust_;
void comThrCallback(const std_msgs::Float64::ConstPtr& thr_msg){
    comThrust_ = thr_msg->data;
}

/* Failed motor number callback function
 * Subscribes to the topic /motor_failure/motor_number - determines which motor has failed.
 */
int motor_num;
void motorNumCallback(const std_msgs::Int32::ConstPtr& motorNum_msg){
    if(motorNum_msg->data == 1)
        motor_num = 1;
    if(motorNum_msg->data == 4)
        motor_num = 4;
}

/* Local position callback function
 * Subscribes to the local position estimates topic
 */
Vector3d actPose_;
void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& pos_msg){
    actPose_ <<  pos_msg->pose.position.x, pos_msg->pose.position.y, pos_msg->pose.position.z;
}

/* Reference trajectory callback function
 * Subscribes to the commanded trajectory topic
 */
Vector4d refPose_;
void refPosCallback(const geometry_msgs::QuaternionStamped::ConstPtr& pos_msg){
    refPose_ <<  pos_msg->quaternion.x, pos_msg->quaternion.y, pos_msg->quaternion.z, pos_msg->quaternion.w;
}

/* Local attitude callback function
 * Subscribes to the local attitude angles estimates topic
 */
Vector3d actAtt_;
void localAttitudeCallback(const geometry_msgs::PoseStamped::ConstPtr& att_msg){
    actAtt_ <<  att_msg->pose.orientation.x, att_msg->pose.orientation.y, att_msg->pose.orientation.z;
}

/* Command attitude callback function
 * Subscribes to the commanded attitude angles topic
 */
Vector3d comAtt_;
void comAttitudeCallback(const geometry_msgs::PoseStamped::ConstPtr& att_msg){
    comAtt_ <<  att_msg->pose.orientation.x, att_msg->pose.orientation.y, att_msg->pose.orientation.z;
}

/* Local velocity callback function
 * Subscribes to the local velocity estimates topic
 */
Vector3d actVel_;
void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
{
    actVel_ << vel_msg->twist.linear.x, vel_msg->twist.linear.y, vel_msg->twist.linear.z;
}

/* Command velocity callback function
 * Subscribes to the commanded velocity topic
 */
Vector3d comVel_;
void comVelCallback(const geometry_msgs::Quaternion::ConstPtr& vel_msg){
    comVel_ <<  vel_msg->x, vel_msg->y, vel_msg->z;
}

/* PD velocity callback function
 * Subscribes to the PD velocity command topic
 */
Vector3d pdVel_;
void pdVelCallback(const geometry_msgs::Quaternion::ConstPtr& vel_msg){
    pdVel_ <<  vel_msg->x, vel_msg->y, vel_msg->z;
}

/* ANN velocity callback function
 * Subscribes to the ANN velocity command topic
 */
Vector3d annVel_;
void annVelCallback(const geometry_msgs::Quaternion::ConstPtr& vel_msg){
    annVel_ <<  vel_msg->x, vel_msg->y, vel_msg->z;
}

/* FNN velocity callback function
 * Subscribes to the FNN velocity command topic
 */
Vector3d fnnVel_;
void fnnVelCallback(const geometry_msgs::Quaternion::ConstPtr& vel_msg){
    fnnVel_ <<  vel_msg->x, vel_msg->y, vel_msg->z;
}


// Constructor
data_record::data_record(int argc, char** argv){
    ros::init(argc, argv, "data_record");
    ros::NodeHandle nh;

    // Publisher
    record_flag_pub = nh.advertise<std_msgs::Int32>("/record_start_flag", 1);

    // Subscribers
    //record_flag_sub = nh.subscribe<std_msgs::Int32>("/record_start_flag", 1, recFlagCallback);
    traj_type_sub = nh.subscribe<std_msgs::Int32>("/trajectory_type", 1, trajTypeCallback);
    motor_num_sub = nh.subscribe<std_msgs::Int32>("/motor_failure/motor_number", 1, motorNumCallback);

    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, localPosCallback);
    ref_traj_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("/y6/command_position", 1,refPosCallback);
    local_att_sub = nh.subscribe<geometry_msgs::PoseStamped>("/attitude_RPY/local", 1, localAttitudeCallback);
    com_att_sub = nh.subscribe<geometry_msgs::PoseStamped>("/attitude_RPY/command", 1, comAttitudeCallback);
    local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1, localVelCallback);
    com_vel_sub = nh.subscribe<geometry_msgs::Quaternion>("/y6/command_velocity", 1, comVelCallback);

    pd_vel_sub = nh.subscribe<geometry_msgs::Quaternion>("/y6/command/velocity_pd", 1,pdVelCallback);
    ann_vel_sub = nh.subscribe<geometry_msgs::Quaternion>("/ann/command_velocity", 1,annVelCallback);
    fnn_vel_sub = nh.subscribe<geometry_msgs::Quaternion>("/fnn/command_velocity", 1,fnnVelCallback);

    com_thr_sub = nh.subscribe<std_msgs::Float64>("/test_thrust",1,comThrCallback);

    // Initializing parameters
    i = 1;
    result = 0;

    t_last = ros::Time::now().toSec();

}

// Destructor
data_record::~data_record(){
    ros::shutdown();
    exit(0);
}

void data_record::run(){
    ros::Rate rate(100);

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<controllers::setDataRecordConfig> server;
    dynamic_reconfigure::Server<controllers::setDataRecordConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Wait for connection from FCU
    while(ros::ok()){
        for(int p = 0; p < 100; p++){
            ros::spinOnce();
            rate.sleep();
        }        
        break;
    }

    // Create a temporary file to store the data
    string path_dir = ros::package::getPath("controllers") + "/Data/";
    string tmpfile = "tempData.txt";
    //outputFile.open(path_dir + tmpfile);
    outputFile.open(path_dir + tmpfile);
    ROS_INFO("File created with name: %s!",tmpfile.c_str());


    // Initialize time variables
    rec_time = 0;
    t = ros::Time::now().toSec();
    t_last = t;

    // Main loop
    while(ros::ok())
    {

        // Debug
/*      std::cout<<"check1"<<std::endl;
 *      std::cout<<i<<std::endl;
 *      std::cout<<rec_time<<std::endl;     */


        // Check if the "record_start" flag is on and the correct trajectory type for which data is to be recorded is selected
        /* Change "traj_type" in the if condition
         *  1 ......... hover
         *  2 ......... user_hover
         *  3 ......... waypoints
         *  4 ......... agile_waypoints
         *  5 ......... circle / slant-circle
         *  6 ......... mapping_waypoints
         * 10 ......... straight_line
         * 13 ......... vertical_cricle
         *
         ************************************/

        if(record_start && traj_type == 6){ // ************** Modify here **************
            if(screen_msg_flag){
                t_last = ros::Time::now().toSec();
                std::cout << "record_data flag is on" << std::endl;
                std::cout << "Starting to record data!" << std::endl;
                screen_msg_flag = 0;
            }

            // To stop recording data: Make "record_flag" = 0 if "rec_time" > "max_rec_time"
            if(rec_time > max_rec_time){
                record_start = 0;
                rec_start_flag.data = 0;
                std::cout<<"record_flag is off; Saving data and exiting!"<<std::endl;
                // Close the file
                outputFile.close();
            }

            /* ------- Starting to write data into file --------- */
            outputFile<<ros::Time::now()<<" "                                                       // column 1:      time
                      <<refPose_(0)<<" "<<refPose_(1)<<" "<<refPose_(2)<<" "                        // column 2-4:    reference trajectory
                      <<refPose_(3)<<" "                                                            // column 5:      desired yaw
                      <<actPose_(0)<<" "<<actPose_(1)<<" "<<actPose_(2)<<" "                        // column 6-8:    local position
                      <<comAtt_(0)<<" "<<comAtt_(1)<<" "<<comAtt_(2)<<" "                           // column 9-11:   command attitude
                      <<actAtt_(0)<<" "<<actAtt_(1)<<" "<<actAtt_(2)<<" "                           // column 12-14:  local attitude
                      <<comVel_(0)<<" "<<comVel_(1)<<" "<<comVel_(2)<<" "                           // column 15-17:  command velocity
                      <<actVel_(0)<<" "<<actVel_(1)<<" "<<actVel_(2)<<" "                           // column 18-20:  actual velocity
                      <<pdVel_(0)<<" "<<pdVel_(1)<<" "<<pdVel_(2)<<" "                              // column 21-23:  pd velocity
                      <<annVel_(0)<<" "<<annVel_(1)<<" "<<annVel_(2)<<" "                           // column 24-26:  ann velocity
                      <<fnnVel_(0)<<" "<<fnnVel_(1)<<" "<<fnnVel_(2)<<" "                           // column 27-29:  fnn velocity
                      <<comThrust_<<endl;                                                           // column 30:     command throttle

            // Update "rec_time" in each loop [obsolete]
            rec_time = ros::Time::now().toSec() - t_last;
            i++;

            ros::spinOnce();
            rate.sleep();
        }

        // Check if "record_start" is off again and print the message
        if(record_start == 0 && !screen_msg_flag){
            std::cout<<"Renaming the created file"<<std::endl;
            screen_msg_flag = 1;
        }

        // Publish the record_start flag
        record_flag_pub.publish(rec_start_flag);


        /* ----- Post process, check data and rename the file ----- */
        // Break the loop if record_start becomes 0 AGAIN
        if(record_start == 0 && i != 1){

            // Define the trajectory type
            if(traj_type == 1)
                trajectory_type = "hover";
            else if(traj_type == 2)
                trajectory_type = "user_hover";
            else if(traj_type == 3)
                trajectory_type = "waypoints";
            else if(traj_type == 4)
                trajectory_type = "agile_waypoints";
            else if(traj_type == 5)
                trajectory_type = "circle";
            else if(traj_type == 6)
                trajectory_type = "mapping_waypoints";
            else if(traj_type == 10)
                trajectory_type = "straight_line";
            else if(traj_type == 13)
                trajectory_type = "vertical_circle";

            // Define the controller type
            if(fnnVel_(0) == 0 && fnnVel_(1) == 0 && fnnVel_(2) == 0 && annVel_(0) == 0 && annVel_(1) == 0 && annVel_(2) == 0)
                controller_type = "_PID_";
            else if(annVel_(0) == 0 && annVel_(1) == 0 && annVel_(2) == 0)
                controller_type = "_FNN-PD_";
            else
                controller_type = "_ANN-PD_";

            // Define position/attitude setpoint
            if(comAtt_(0) == 0 && comAtt_(1) == 0 && comAtt_(2) == 0)
                setpoint_type = "pos-setpoint_";
            else
                setpoint_type = "att-setpoint_";

            // Define motor failure number
            if(motor_num == 1){
                motor_fail_type = "__front-top_";
                experiment_type = "motorFail";
            }
            else if(motor_num == 4){
                motor_fail_type = "__back-bottom_";
                experiment_type = "motorFail";
            }
            else{
                motor_fail_type = "_";
                experiment_type = "fastFlight";
            }

            break;
        }

        ros::spinOnce();
        rate.sleep();

    }

    // The final filename is created based on the above loop
    filename = experiment_type + controller_type + setpoint_type + trajectory_type + motor_fail_type + ".txt";

    // Rename the created txt file
    result = rename((path_dir + tmpfile).c_str(), (path_dir + filename).c_str());
    if ( result == 0 ){
        ROS_WARN( "File successfully renamed: %s",filename.c_str());
        ROS_WARN( "Uncheck the record_start flag before exiting!");
    }
    else
        ROS_ERROR("Error renaming file!");

    // Hold the loop here and stop from exiting
    while(ros::ok())
    {
        record_flag_pub.publish(rec_start_flag);

        ros::spinOnce();
        rate.sleep();
    }
}

// Main function
int main(int argc, char** argv){
    cout << "[record_data] Data logger is running..." << endl;

    data_record* controller = new data_record(argc, argv);

    controller->run();

    return 0;
}
