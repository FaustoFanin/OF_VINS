#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <image_transport/image_transport.h>

#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cpp_test/OpticalFlow.h>
using namespace cv;

int main(int, char**){
  VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  UMat of;
  UMat frame_raw, prev_frame_raw;
  UMat frame, prev_frame;
  UMat frame_s, prev_frame_s;
  std::vector<UMat> flow_vec(2);
  UMat mag, ang;
  UMat hsv, hsv_h, hsv_s, hsv_v;
  std::vector<UMat> hsv_vec(3);
  UMat of_colour;

  UMat vel, cov;
  UMat blur, thres;

  cap >> prev_frame_raw;
  cvtColor(prev_frame_raw, prev_frame, COLOR_RGB2GRAY);
  resize(prev_frame, prev_frame_s, Size(320,320), 0, 0);

  hsv_s = UMat(Size(320,320), prev_frame_s.type(), 255);
  namedWindow("edges",1);

  for(;;){

    cap >> frame_raw;

    cvtColor(frame_raw, frame, COLOR_RGB2GRAY);

    resize(frame,frame_s,Size(320,320), 0, 0);

    //optflow::calcOpticalFlowSF(prev_frame_s, frame_s, of, 3, 2, 4);   // Slow AF
    calcOpticalFlowFarneback(prev_frame_s, frame_s, of, 0.5, 3, 15, 3, 5, 1.2, 0);    // Good starting point

    frame_s.copyTo(prev_frame_s);

    meanStdDev(of, vel, cov);
    //std::cout << vel.getMat(ACCESS_READ).at<float>(1) << std::endl;


    // Visualisation stuff

    split(of, flow_vec);
    cartToPolar(flow_vec[0], flow_vec[1], mag, ang, true);
    ang.convertTo(hsv_h, CV_8UC1, 0.5);

    normalize(mag, hsv_v, 0, 255, NORM_MINMAX, CV_8UC1);

    hsv_vec[0] = hsv_h; // Orientation
    hsv_vec[1] = hsv_s; // 255 matrix
    //hsv_vec[2] = hsv_v; // Magnitude

    //adaptiveThreshold(hsv_v, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    GaussianBlur(hsv_v, blur, Size(11,11), 0);
    threshold(blur, thres, 0, 255, THRESH_TOZERO+THRESH_OTSU);
    //threshold(blur, thres, 90, 255, THRESH_TOZERO);

    hsv_vec[2] = thres; // Magnitude
    merge(hsv_vec, hsv);
    cvtColor(hsv, of_colour, COLOR_HSV2BGR);

    imshow("edges", of_colour);
    if(waitKey(30) >= 0) break;
  }

  return 0;
}
