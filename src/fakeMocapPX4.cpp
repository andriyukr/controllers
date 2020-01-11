/** *************************** fakeMocapPX4.cpp ***************************
 *
 * This code is a fake Mocap publisher for simulation [only]. Change the launch file.
 * Subscribes to the gazebo state estimates and publishes them to mocap topic
 *
 * ************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <rosgraph_msgs/Clock.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;

// Publishers
Publisher mocap_publisher;

// Subsribers
Subscriber mocap_subscriber;

geometry_msgs::PoseStamped pose_for_mavros;
bool first_received = false;
int index_name = -1;
std::string uav_name;

// ********************** Callbacks **********************

/* UAV estimates callback function (from Gazebo)
 * Subscribes to UAV state estimates, creates and publishes to mocap topic
 */
void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_model_msg){
     pose_for_mavros.header.stamp = ros::Time::now();

     int i = 0;
     if(!first_received){ // For the first time find required model name (index)
       while(!first_received){
         if(!gazebo_model_msg->name[i].compare("iris") or !gazebo_model_msg->name[i].compare("y6")){
           index_name = i;
           first_received = true;
          //  cout<<"Number:  "<<index_name<<" ##### "<<gazebo_model_msg->name[i]<<endl;
         }
         i++;

       }
     }
     pose_for_mavros.pose = gazebo_model_msg->pose[index_name];
     mocap_publisher.publish(pose_for_mavros);
}


class FakeMocapPX4{
public:
    // Constructor
    FakeMocapPX4(int argc, char** argv){
        ros::init(argc, argv, "FakeMocapPX4");
        ros::NodeHandle node_handle;


        // Subscribers
        mocap_subscriber = node_handle.subscribe("/in_odometry", 1, gazeboCallback);
        //mocap_subscriber = node_handle.subscribe("/gazebo/model_states", 1, gazeboCallback);

        // Publishers
        mocap_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/out_odometry", 1);
        //mocap_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1);
    }

    // Destructor
    ~FakeMocapPX4(){
        ros::shutdown();
        exit(0);
    }

    void run(){
        ros::Rate rate(100);
        // Main loop - it runs until stopped/cancelled and the callback publishes mocap data
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv){
    cout << "[FakeMocapPX4] FakeMocapPX4 is running..." << endl;

    FakeMocapPX4* fm = new FakeMocapPX4(argc, argv);

    fm->run();
}
