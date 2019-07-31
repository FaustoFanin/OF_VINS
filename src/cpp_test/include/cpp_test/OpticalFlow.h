#include <cv_bridge/cv_bridge.h>
class OpticalFlow{
  private:
    ros::NodeHandle node;
    image_transport::ImageTransport im_tp;

    cv_bridge::CvImagePtr raw_up, raw_up_old;
    cv_bridge::CvImagePtr raw_fw, raw_fw_old;
    cv_bridge::CvImagePtr raw_dw, raw_dw_old;
    cv_bridge::CvImagePtr raw_bw, raw_bw_old;
    cv_bridge::CvImagePtr raw_ll, raw_ll_old;
    cv_bridge::CvImagePtr raw_rr, raw_rr_old;

    geometry_msgs::TwistWithCovariance vel_est;
    cv::UMat OF_up,OF_fw,OF_dw,OF_bw,OF_ll,OF_rr;

    std::vector<cv::UMat> flow_vec;
    cv::UMat mag, ang;
    cv::UMat hsv, hsv_h, hsv_s, hsv_v;
    std::vector<cv::UMat> hsv_vec;
    cv::UMat of_colour;
    cv::UMat blur, thres;

    image_transport::Subscriber sub_up;
    image_transport::Subscriber sub_fw;
    image_transport::Subscriber sub_dw;
    image_transport::Subscriber sub_bw;
    image_transport::Subscriber sub_ll;
    image_transport::Subscriber sub_rr;

    image_transport::Publisher of_pub;
    ros::Publisher vel_pub;

    bool up_rec = false;
    bool fw_rec = false;
    bool dw_rec = false;
    bool bw_rec = false;
    bool ll_rec = false;
    bool rr_rec = false;

  public:
    OpticalFlow(): im_tp(node), flow_vec(2), hsv_vec(3){
      sub_up = im_tp.subscribe("/top_cam/camera/image", 10, &OpticalFlow::upCallback, this);
      sub_fw = im_tp.subscribe("/front_cam/camera/image", 10, &OpticalFlow::fwCallback, this);
      sub_dw = im_tp.subscribe("/bottom_cam/camera/image", 10, &OpticalFlow::dwCallback, this);
      sub_bw = im_tp.subscribe("/back_cam/camera/image", 10, &OpticalFlow::bwCallback, this);
      sub_ll = im_tp.subscribe("/left_cam/camera/image", 10, &OpticalFlow::llCallback, this);
      sub_rr = im_tp.subscribe("/right_cam/camera/image", 10, &OpticalFlow::rrCallback, this);

      of_pub = im_tp.advertise("/image_converter/output_video", 1);

      vel_pub = node.advertise<geometry_msgs::TwistWithCovariance>("/raw_vel", 20);

      hsv_s = cv::UMat(cv::Size(320,320), CV_8UC1, 255);

    }

    void flowSpin(){
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

          cv_bridge::CvImagePtr of_out(new cv_bridge::CvImage);

          of_out->encoding = "bgr8";
          of_out->header.stamp = ros::Time::now();
          of_out->header.frame_id = "/image_converter/output_video";
          ROS_INFO("Converting image");
          of_out->image = convertToHSV(OF_rr);
          of_pub.publish(of_out->toImageMsg());

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

      return;
    }

    cv::UMat DenseOF(const cv_bridge::CvImagePtr& raw_image_ptr, const cv_bridge::CvImagePtr& raw_image_prev_ptr){
      cv::UMat img_grey, img_prev_grey;
      cv::UMat OF_img;

      cv::cvtColor(raw_image_ptr->image, img_grey, cv::COLOR_RGB2GRAY);
      cv::cvtColor(raw_image_prev_ptr->image, img_prev_grey, cv::COLOR_RGB2GRAY);
      /*cv::optflow::calcOpticalFlowSF(cv_ptr_prev->image, cv_ptr->image,
                                     OF_img,
                                     3, 2, 4);//, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10); // Type is CV_32FC2
                                     */ // Slow as testicles
      //cv::calcOpticalFlowFarneback(raw_image_prev_ptr->image, raw_image_ptr->image, OF_img, 0.5, 3, 15, 3, 5, 1.2, 0);
      cv::calcOpticalFlowFarneback(img_prev_grey, img_grey, OF_img, 0.5, 3, 15, 3, 5, 1.2, 0);

      return OF_img;
    }

    cv::Mat convertToHSV(cv::UMat& of){
      split(of, flow_vec);
      cartToPolar(flow_vec[0], flow_vec[1], mag, ang, true);
      ang.convertTo(hsv_h, CV_8UC1, 0.5);
      normalize(mag, hsv_v, 0, 255, cv::NORM_MINMAX, CV_8UC1);

      hsv_vec[0] = hsv_h; // Orientation
      hsv_vec[1] = hsv_s; // 255 matrix
      //hsv_vec[2] = hsv_v; // Magnitude

      //adaptiveThreshold(hsv_v, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
      GaussianBlur(hsv_v, blur, cv::Size(11,11), 0);
      threshold(blur, thres, 0, 255, cv::THRESH_TOZERO+cv::THRESH_OTSU);
      //threshold(blur, thres, 90, 255, THRESH_TOZERO);

      hsv_vec[2] = thres; // Magnitude
      merge(hsv_vec, hsv);
      cvtColor(hsv, of_colour, cv::COLOR_HSV2BGR);

      return of_colour.getMat(cv::ACCESS_WRITE);
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
      //std::cout << vel_up << std::endl;
      // ^ This should return a 1x1 2-channel Mat where the 1st channel is horizontal flow and the second is vertical flow

      // OF output is [pixel displacement/frame]
      // 10 frames / second
      // ~1m/320 pixels
      float magic_scaling_num = 10.0/320.0;  // Figure this out
      // Estimate velocity and covariance
      std::cout << vel_up.getMat(cv::ACCESS_READ).at<float>(1) << vel_ll.getMat(cv::ACCESS_READ).at<float>(0) << vel_dw.getMat(cv::ACCESS_READ).at<float>(1) << vel_rr.getMat(cv::ACCESS_READ).at<float>(0) << std::endl;

      vel_est.twist.linear.x = magic_scaling_num*(vel_up.getMat(cv::ACCESS_READ).at<float>(1) + vel_ll.getMat(cv::ACCESS_READ).at<float>(0) + vel_dw.getMat(cv::ACCESS_READ).at<float>(1) - vel_rr.getMat(cv::ACCESS_READ).at<float>(0))/4.0;

      vel_est.twist.linear.y = magic_scaling_num*(-vel_up.getMat(cv::ACCESS_READ).at<float>(0) - vel_fw.getMat(cv::ACCESS_READ).at<float>(0) + vel_dw.getMat(cv::ACCESS_READ).at<float>(0) + vel_bw.getMat(cv::ACCESS_READ).at<float>(0))/4.0;

      vel_est.twist.linear.z = magic_scaling_num*(-vel_fw.getMat(cv::ACCESS_READ).at<float>(1) - vel_ll.getMat(cv::ACCESS_READ).at<float>(1) - vel_bw.getMat(cv::ACCESS_READ).at<float>(1) - vel_rr.getMat(cv::ACCESS_READ).at<float>(1))/4.0;

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

};
