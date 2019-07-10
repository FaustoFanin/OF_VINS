#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "test1");

  ros::NodeHandle n;

  ros::Publisher publisher = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    publisher.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
