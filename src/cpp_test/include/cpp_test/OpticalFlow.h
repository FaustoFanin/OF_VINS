#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/TwistWithCovariance.h>

#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::UMat DenseOF(const sensor_msgs::ImageConstPtr& raw_image, const sensor_msgs::ImageConstPtr& raw_image_prev){
  cv_bridge::CvImagePtr cv_ptr, cv_ptr_prev;

  try{
    cv_ptr = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
    cv_ptr_prev = cv_bridge::toCvCopy(raw_image_prev, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    //return;   // What do I return here?
  }

  cv::UMat OF_img;

  /*cv::optflow::calcOpticalFlowSF(cv_ptr_prev->image, cv_ptr->image,
                                 OF_img,
                                 3, 2, 4);//, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10); // Type is CV_32FC2
                                 */ // Slow as testicles
  cv::calcOpticalFlowFarneback(cv_ptr_prev->image, cv_ptr->image, OF_img, 0.5, 3, 15, 3, 5, 1.2, 0);

  return OF_img;
}

geometry_msgs::TwistWithCovariance VelAlg(cv::UMat& OF_up, cv::UMat& OF_fw, cv::UMat& OF_dw, cv::UMat& OF_bw, cv::UMat& OF_ll, cv::UMat& OF_rr){
  geometry_msgs::TwistWithCovariance vel_est;
  int width = OF_up.cols;
  int height = OF_up.rows;

  cv::UMat vel_up, cov_up;
  cv::UMat vel_fw, cov_fw;
  cv::UMat vel_dw, cov_dw;
  cv::UMat vel_bw, cov_bw;
  cv::UMat vel_ll, cov_ll;
  cv::UMat vel_rr, cov_rr;

  // Do transformation from rectangular to spherical?

  // Calculate average vectors
  cv::meanStdDev(OF_up, vel_up, cov_up);
  cv::meanStdDev(OF_fw, vel_fw, cov_fw);
  cv::meanStdDev(OF_dw, vel_dw, cov_dw);
  cv::meanStdDev(OF_bw, vel_bw, cov_bw);
  cv::meanStdDev(OF_ll, vel_ll, cov_ll);
  cv::meanStdDev(OF_rr, vel_rr, cov_rr);
  // ^ This should return a 1x1 2-channel Mat where the 1st channel is horizontal flow and the second is vertical flow

  float magic_scaling_num = 1.0;  // Figure this out
  // Estimate velocity and covariance
  vel_est.twist.linear.x = (vel_up.getMat(cv::ACCESS_READ).at<float>(1) + vel_ll.getMat(cv::ACCESS_READ).at<float>(0) + vel_dw.getMat(cv::ACCESS_READ).at<float>(1) - vel_rr.getMat(cv::ACCESS_READ).at<float>(0))/4.0;

  vel_est.twist.linear.y = (-vel_up.getMat(cv::ACCESS_READ).at<float>(0) - vel_fw.getMat(cv::ACCESS_READ).at<float>(0) + vel_dw.getMat(cv::ACCESS_READ).at<float>(0) + vel_bw.getMat(cv::ACCESS_READ).at<float>(0))/4.0;

  vel_est.twist.linear.z = (-vel_fw.getMat(cv::ACCESS_READ).at<float>(1) - vel_ll.getMat(cv::ACCESS_READ).at<float>(1) - vel_bw.getMat(cv::ACCESS_READ).at<float>(1) - vel_rr.getMat(cv::ACCESS_READ).at<float>(1))/4.0;

  vel_est.twist.angular.x = 0;
  vel_est.twist.angular.y = 0;
  vel_est.twist.angular.z = 0;

  /*vel_est.covariance = {1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0,
                        0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 1};*/  // Initliase to eye(6)
  vel_est.covariance.fill(0);


  vel_est.covariance[0] = (cov_up.getMat(cv::ACCESS_READ).at<float>(1) + cov_ll.getMat(cv::ACCESS_READ).at<float>(0) + cov_dw.getMat(cv::ACCESS_READ).at<float>(1) + cov_rr.getMat(cv::ACCESS_READ).at<float>(0))/4.0;
  vel_est.covariance[7] = (cov_up.getMat(cv::ACCESS_READ).at<float>(0) + cov_fw.getMat(cv::ACCESS_READ).at<float>(0) + cov_dw.getMat(cv::ACCESS_READ).at<float>(0) + cov_bw.getMat(cv::ACCESS_READ).at<float>(0))/4.0;
  vel_est.covariance[14] = (cov_fw.getMat(cv::ACCESS_READ).at<float>(1) + cov_ll.getMat(cv::ACCESS_READ).at<float>(0) + cov_bw.getMat(cv::ACCESS_READ).at<float>(1) + cov_rr.getMat(cv::ACCESS_READ).at<float>(1))/4.0;
  vel_est.covariance[21] = 0;
  vel_est.covariance[28] = 0;
  vel_est.covariance[35] = 0;

  return vel_est;
}
