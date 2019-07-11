#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cpp_test/TrajectoryManager.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>

using namespace std;

int main(int argc, char **argv){

  ros::init(argc, argv, "main_test");

  TrajectoryManager TrajMan;
  ros::Rate loop_rate(1);

  // Variables
  int input = 0;
  ros::Duration(1.0).sleep();

  ROS_INFO("Starting Node");

  while( ros::ok() ){
    cout << "Press Enter to Continue";
    cin.ignore();

    /*poseStamped.pose.position.x = 0.0;
    poseStamped.pose.position.y = 0.0;
    poseStamped.pose.orientation.x = 1.0;
    poseStamped.pose.orientation.y = 0.0;
    poseStamped.pose.orientation.z = 0.0;
    poseStamped.pose.orientation.w = 0.0;
    if (input == 1){
      poseStamped.pose.position.z = 1.0;
      cout << "Set to hover. " << endl;
    }else if (input == 2){
      poseStamped.pose.position.z = 0.0;
      cout << "Set to land. " << endl;
    }else if (input == 3){
      poseStamped.pose.position.y = 3.0;
      poseStamped.pose.position.z = 1.0;
      cout << "Set to land. " << endl;
    }else if (input == 9){
      return 0;
    }
    input = 0;*/

    TrajMan.spin();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
