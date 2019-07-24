#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Point.h>

#include <cpp_test/OpticalFlow.h>
#include <image_transport/image_transport.h>

cv_bridge::CvImagePtr raw_up, raw_up_old;
cv_bridge::CvImagePtr raw_fw, raw_fw_old;
cv_bridge::CvImagePtr raw_dw, raw_dw_old;
cv_bridge::CvImagePtr raw_bw, raw_bw_old;
cv_bridge::CvImagePtr raw_ll, raw_ll_old;
cv_bridge::CvImagePtr raw_rr, raw_rr_old;

bool up_rec = false;
bool fw_rec = false;
bool dw_rec = false;
bool bw_rec = false;
bool ll_rec = false;
bool rr_rec = false;

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
void upCallback(const sensor_msgs::ImageConstPtr& raw_image){
  try{
    raw_up = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
void fwCallback(const sensor_msgs::ImageConstPtr& raw_image){
  try{
    raw_fw = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
void dwCallback(const sensor_msgs::ImageConstPtr& raw_image){
  try{
    raw_dw = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
void bwCallback(const sensor_msgs::ImageConstPtr& raw_image){
  try{
    raw_bw = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
void llCallback(const sensor_msgs::ImageConstPtr& raw_image){
  try{
    raw_ll = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
void rrCallback(const sensor_msgs::ImageConstPtr& raw_image){
  try{
    raw_rr = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "vel_estimator");

  ros::NodeHandle node;
  image_transport::ImageTransport im_tp(node);

  geometry_msgs::TwistWithCovariance vel_est;
  cv::UMat OF_up,OF_fw,OF_dw,OF_bw,OF_ll,OF_rr;

  image_transport::Subscriber sub_up = im_tp.subscribe("/top_cam/camera/image", 10, upCallback);
  image_transport::Subscriber sub_fw = im_tp.subscribe("/front_cam/camera/image", 10, fwCallback);
  image_transport::Subscriber sub_dw = im_tp.subscribe("/bottom_cam/camera/image", 10, dwCallback);
  image_transport::Subscriber sub_bw = im_tp.subscribe("/back_cam/camera/image", 10, bwCallback);
  image_transport::Subscriber sub_ll = im_tp.subscribe("/left_cam/camera/image", 10, llCallback);
  image_transport::Subscriber sub_rr = im_tp.subscribe("/right_cam/camera/image", 10, rrCallback);

  ros::Publisher vel_pub = node.advertise<geometry_msgs::TwistWithCovariance>("/raw_vel", 20);

  ROS_INFO("::: VELOCITY ESTIMATER NODE INITIALISED");

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    if(raw_up && raw_fw && raw_dw && raw_bw && raw_ll && raw_rr){
      // Perform OF alg
      if(raw_up_old && raw_fw_old && raw_dw_old && raw_bw_old && raw_ll_old && raw_rr_old){

        ROS_INFO("Calculating Dense OF");
        OF_up = DenseOF(raw_up, raw_up_old);
        OF_fw = DenseOF(raw_fw, raw_fw_old);
        OF_bw = DenseOF(raw_bw, raw_bw_old);
        OF_dw = DenseOF(raw_dw, raw_dw_old);
        OF_ll = DenseOF(raw_ll, raw_ll_old);
        OF_rr = DenseOF(raw_rr, raw_rr_old);

        // Do vel alg
        vel_est = VelAlg(OF_up,OF_fw,OF_dw,OF_bw,OF_ll,OF_rr);

        // Publish to topic
        vel_pub.publish(vel_est);
      }

      ROS_INFO("Copying old images");
      raw_up_old = raw_up;
      raw_fw_old = raw_fw;
      raw_dw_old = raw_dw;
      raw_bw_old = raw_bw;
      raw_rr_old = raw_ll;
      raw_ll_old = raw_rr;

      ROS_INFO("Resetting pointers");
      raw_up.reset();
      raw_fw.reset();
      raw_dw.reset();
      raw_bw.reset();
      raw_ll.reset();
      raw_rr.reset();
    }else{
      // Didn't receive all 6 images
      ROS_INFO("Didn't receive all 6");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
