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

geometry_msgs::PoseStamped pose;
bool first_received = false;
int index_name = -1;
std::string robot_name;

// ********************** Callbacks **********************

/* UAV estimates callback function (from Gazebo)
 * Subscribes to UAV state estimates, creates and publishes to mocap topic
 */
void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_model_msg){
     int i = 0;
     if(!first_received){ // For the first time find required model name (index)
       while(!first_received){
         if(!gazebo_model_msg->name[i].compare(robot_name)){
           index_name = i;
           first_received = true;
           //cout<<"Number:  "<<index_name<<" ##### "<<gazebo_model_msg->name[i]<<endl;
         }
         i++;
       }
     }

     pose.header.stamp = ros::Time::now();
     pose.header.frame_id = "map";
     pose.pose = gazebo_model_msg->pose[index_name];
     mocap_publisher.publish(pose);
}


class FakeMocap{
public:
    // Constructor
    FakeMocap(int argc, char** argv){
        ros::init(argc, argv, "FakeMocap");
        ros::NodeHandle node_handle;


        // Subscribers
        mocap_subscriber = node_handle.subscribe("/in_pose", 1, gazeboCallback, ros::TransportHints().tcpNoDelay()); // ros::TransportHints().tcpNoDelay() needed to avoid communication delay

        // Publishers
        mocap_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/out_pose", 1);

        if(!node_handle.getParam("/fake_mocap/robot", robot_name))
            ROS_WARN_STREAM("[FakeMocap] Parameter 'robot' not defined!");
        ROS_INFO_STREAM("[FakeMocap] Robot name: " << robot_name);
    }

    // Destructor
    ~FakeMocap(){
        ros::shutdown();
        exit(0);
    }

    void run(){
        ros::Rate rate(1000);
        // Main loop - it runs until stopped/cancelled and the callback publishes mocap data
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv){
    ROS_INFO_STREAM("[FakeMocap] FakeMocap is running...");

    FakeMocap* fm = new FakeMocap(argc, argv);

    fm->run();
}
