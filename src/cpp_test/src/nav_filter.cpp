#define STATES 10   // Number of states of Kalman Filter
#define INPUTS 6    // Number of inputs of Kalman Filter
#define MSRMTS 3   // Number of measurements of Kalman Filter

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <cpp_test/NavFilter.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "my_nav_filter");

  Eigen::Matrix<double,STATES,1> state_prediction;

  int node_freq = 10;   // Hz
  NavFilter filter(node_freq);

  ROS_INFO("::: NAV. NODE INITIALISED");

  ros::Rate loop_rate(node_freq);

  while(ros::ok())
  {

    state_prediction = filter.propagateFilter();
    //std::cout << state_prediction.transpose() << std::endl;
    //pubTFframe(state_prediction);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
