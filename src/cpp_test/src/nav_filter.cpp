#define STATES 16   // Number of states of Kalman Filter
#define MSRMTS 16   // Number of measurements of Kalman Filter

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <cpp_test/NavFilter.h>


/*
void pubTFframe(Eigen::VectorXf const &state_vec){
  // Taken and modified from: http://wiki.ros.org/tf2/Tutorials/Writing
  static tf2_ros::TransformBroadcaster broadcaster;

  geometry_msgs::TransformStamped tfStamped;

  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);

  tfStamped.header.stamp = ros::Time::now();
  tfStamped.header.frame_id = "world";
  tfStamped.child_frame_id = turtle_name;
  tfStamped.transform.translation.x = msg->x;
  tfStamped.transform.translation.y = msg->y;
  tfStamped.transform.translation.z = 0.0;
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();

  broadcaster.sendTransform(tfStamped);
}
*/
/*
<plugin name="quadrotor_imu_sim" filename="libhector_gazebo_ros_imu.so">
  <updateRate>100.0</updateRate>
  <bodyName>base_link</bodyName>
  <frameId>$(arg base_link_frame)</frameId>
  <topicName>raw_imu</topicName>
  <rpyOffset>0 0 0</rpyOffset> <!-- deprecated -->
  <gaussianNoise>0</gaussianNoise>  <!-- deprecated -->
  <accelDrift>0.1 0.1 0.1</accelDrift>
  <accelGaussianNoise>0.35 0.35 0.3</accelGaussianNoise>
  <rateDrift>0.1 0.1 0.1</rateDrift>
  <rateGaussianNoise>0.05 0.05 0.015</rateGaussianNoise>
</plugin>
*/


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
    std::cout << state_prediction.transpose() << std::endl;
    //pubTFframe(state_prediction);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
