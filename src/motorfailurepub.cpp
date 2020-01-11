/** *************************** motorfailurepub.cpp ***************************
 *
 * This cpp file takes the user input for the particular motor to fail. It
 * publishes the respective motor number to be failed on a topic which is
 * subscribed by the Gazebo simulator plugin.
 * This code also allows the user to fail the motor for a certain time and
 * recover later while following a trajectory.
 *
 * NOTE: The user can only fail the motor for certain time for a particular
 *       trajectory as specified in the code. Change the trajectory type to
 *       accomplish thr same!
 *
 * ************************************************************************ **/

#include <controllers/motorfailurepub.h>


// ********************** Callbacks **********************


/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for motor_num, t_fail_max [total time of motor fail]
 * from the graphical interface.
 * Fail_motor is a flag which checks if the motors have to failed and unchecking would
 * the motor in flight.
 */
void dynamicReconfigureCallback(controllers::setMotorFailureConfig &config, uint32_t level){
    Fail_motor = config.Fail_motor;
    t_fail_max = config.t_fail_max;
   if (Fail_motor) {
        //motor_num = config.Motor_1;
        if (config.Motor_1)
            motor_num = 1;
        else if (config.Motor_2)
            motor_num = 2;
        else if (config.Motor_3)
            motor_num = 3;
        else if (config.Motor_4)
            motor_num = 4;
        else if (config.Motor_5)
            motor_num = 5;
        else if (config.Motor_6)
            motor_num = 6;
        else
            motor_num = 0;
   }

   else{
       motor_num = 0;
       t_stop = 0;
   }

   //record_start.data = config.record_start;

}

/* Reference trajectory type callback function
 * Subscribes to the trajectory type topic - stores the type of trajectory being followed from the integer.
 * This is used later to rename the file in accordance to the trajectory type.
 */
void trajTypeCallback(const std_msgs::Int32::ConstPtr& trajmsg){
    traj_type = *trajmsg;
}

// Main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motorfailurepub");
  ros::NodeHandle nh;

  ROS_INFO_ONCE("Motor Failure node initialized! \n");

  // Subscriber
  traj_type_sub = nh.subscribe<std_msgs::Int32>("/trajectory_type", 1, trajTypeCallback);


  // Publisher
  //ros::Publisher motor_failure_flag_pub = nh.advertise<std_msgs::Bool>("/gazebo/motor_failure/flag", 1);
  motor_failure_number_pub = nh.advertise<std_msgs::Int32>("/motor_failure/motor_number", 1);
  //record_start_flag_pub = nh.advertise<std_msgs::Int32>("/record_start_flag", 1);

  int count = 0;

  ros::Rate loop_rate(100);

  // Setup dynamic reconfigure (GUI) connections
  dynamic_reconfigure::Server<controllers::setMotorFailureConfig> server;
  dynamic_reconfigure::Server<controllers::setMotorFailureConfig>::CallbackType f;
  f = boost::bind(&dynamicReconfigureCallback, _1, _2);
  server.setCallback(f);

  while (ros::ok())
  {
    motor_failure_flag.data = Fail_motor;

    // Debug
    //motor_failure_num.data = motor_num;
    //std::cout << motor_num[1] << std::endl;

    //motor_failure_flag_pub.publish(motor_failure_flag);

    // Maximum time the motor fails and then recovers
    // Use this if-else only for motor failure based on time
    if(t_fail_max == 0)
        motor_failure_num.data = motor_num;
    else{

        /* If doing time based failure --> change the traj_type based on what trajectory the user wants to follow
         * The motor will turn for the first "t_motor_off" sec and then 1 motor will fail and after 25 sec ("t_fail_max") it recovers.
         * Change the "t_mtoro_off" in the header file to change the motor fail start time and modify "t_fail_max" from GUI
         * to change the actual time the motor reamins failed and then recovers.
         *
         *  /* Check for "traj_type" in the if condition as per following:
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

        if(Fail_motor && t_stop <= t_motor_off && (traj_type.data == 6)) { // ************** Modify here **************
            motor_failure_num.data = 0;
            t_stop += 0.01;
        }
        else if(Fail_motor && t_stop - t_motor_off <= t_fail_max && (traj_type.data == 6)){ // ************** Modify here **************
            motor_failure_num.data = motor_num;
            t_stop += 0.01;
            //t_stop = 0;
        }
        else
            motor_failure_num.data = 0;
    }


    // Directly use this for motor_failure on demand
    motor_failure_number_pub.publish(motor_failure_num);


    if(t_stop - t_motor_off > (t_fail_max + t_motor_off)){
        //record_start.data = 0;
    }
    else if(t_stop >= (t_fail_max + t_motor_off))
        t_stop += 0.01;

    //record_start_flag_pub.publish(record_start);

    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }

  return 0;
}
