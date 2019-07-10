#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/EnableMotors.h>

using namespace std;

int main(int argc, char **argv){

  ros::init(argc, argv, "main_test");
  ros::NodeHandle node;
  ros::Rate loop_rate(1);

  geometry_msgs::PoseStamped poseStamped;
  int input = 0;
  ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("/command/pose", 20);
  ros::ServiceClient motor_enable_service_;
  motor_enable_service_ = node.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;

  if(motor_enable_service_.call(srv)){
    ROS_INFO("Motors Enabled.");
  }else{
    ROS_ERROR("Motor Enabling FAILED");
    return 0;
  }

  cout << "Enable motors manually: rosservice call /enable_motors \"enable: true\" " << endl;
  while( ros::ok()){
    cout << "Select options: " << endl;
    cout << "1-[0,0,1]  | 2-[0,0,0]  | 3-[0,3,1]  | 9-Quit"<< endl;
    cin >> input;

    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "world";
    poseStamped.pose.position.x = 0.0;
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
    input = 0;

    pose_pub.publish(poseStamped);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void initial_state(){
  int input = 0;





  /*
  cout << "Select options:" << endl;
  if (motor_state){
    cout << "1-Disable Motors  | 2-Pose Control  | 3-..." << endl;
  }else{
    cout << "1-Enable Motors  | 2-Pose Control  | 3-..." << endl;
  }

  cin >> input;

  if (input == 1){
    if (motor_state){
      rosservice call /enable_motors "enable: true"
    }
  }*/
}
