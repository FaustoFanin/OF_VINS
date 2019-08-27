#define STATES 16   // Number of states of Kalman Filter
#define INPUTS 6    // Number of inputs of Kalman Filter
#define MSRMTS 10   // Number of measurements of Kalman Filter

#include <ros/ros.h>
#include <sstream>
#include <chrono>

#include <std_msgs/String.h>
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
#include <nav_msgs/Odometry.h>

#include <cpp_test/NavFilter.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "my_nav_filter");

  Eigen::Matrix<double,STATES,1> state_prediction;

  int node_freq = 10;   // Hz

  NavFilter filter;

  ROS_INFO("::: NAV. NODE INITIALISED");

  ros::Rate loop_rate(node_freq);

  ros::spin();

  /*while(ros::ok()){

    //state_prediction = filter.propagateFilter();
    //std::cout << state_prediction.transpose() << std::endl;
    //pubTFframe(state_prediction);
    //filter.publishState();

    ROS_INFO("Callbacks in last second - Position:%2d, Velocity:%2d, IMU:%3d", filter.posCallbacks, filter.velCallbacks, filter.imuCallbacks);
    filter.posCallbacks = 0;
    filter.velCallbacks = 0;
    filter.imuCallbacks = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  return 0;
}
