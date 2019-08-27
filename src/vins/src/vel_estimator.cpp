#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Point.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/TwistWithCovariance.h>

#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cpp_test/OpticalFlow.h>

/*
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image){
  // created shared pointer Image
  sensor_msgs::Image::Ptr small_image = boost::make_shared<sensor_msgs::Image>();

  // copy image properties
  small_image->header = old_image->header;
  small_image->height = old_image->height / DIVISOR;
  small_image->width = old_image->width / DIVISOR;
  small_image->encoding = old_image->encoding;
  small_image->is_bigendian = old_image->is_bigendian;
  small_image->step = old_image->step / DIVISOR;

  small_image->data.resize(small_image->width * small_image->height);

  // copy every DIVISORth byte
  // subpixels will be merged if multiple bytes per pixel
  uint new_index = 0;
  for(uint row = 0; row < small_image->height; row++) {
    int row_offset = row*old_image->step*DIVISOR;
    for(uint col = 0; col < small_image->width; col++) {
      int old_index = row_offset + col*DIVISOR;
      small_image->data[new_index++] = old_image->data[old_index];
    }
  }

  pub.publish(small_image);
}
*/


int main(int argc, char **argv){

  ros::init(argc, argv, "vel_estimator");
  OpticalFlow flows;

  ROS_INFO("::: VELOCITY ESTIMATER NODE INITIALISED");

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    flows.flowSpin();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
