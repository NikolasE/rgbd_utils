using namespace std;
#include <iostream>
#include "pinch_recognition/cloud_gmm.h"
#include <opencv-2.3.1/opencv2/highgui/highgui.hpp>
#include "pinch_recognition/pinch_detection.h"


PixelEnvironmentModel model;
cv_bridge::CvImagePtr cv_ptr;
cv::Mat foreground,foreground_prob, stable, inited;

int frame_cnt = 0;
ros::Publisher pub_model;
cv::Mat rgb_image;

float error_factor;
int tb_val = 5;
int tb_val_2 = 30;
float sigma_cnt;
cv::Mat user_mask;
cv::Mat dists;
cv::Mat sigmas;

Grasp_detector grasp_detector;
Eigen::Affine3f kinect_trafo;

std::vector<cv::Point2i> points;


void ImgCB(const sensor_msgs::ImageConstPtr& img_ptr){
  cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
  rgb_image = cv_ptr->image;

  cv::Mat darker = rgb_image*0.5;
  cv::Mat masked; rgb_image.copyTo(masked);
  darker.copyTo(masked,255-foreground_prob);

  cv::imshow("rgb",masked);
}


void CloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){

  frame_cnt++;

  Cloud current;
  pcl::fromROSMsg(*cloud_ptr, current);


  if (frame_cnt < 30){
    model.update(current);
    return;
  }


  // model.update(current);

  // model.getStableMeasurements(stable);

  model.getForeground_dist(current,error_factor,foreground);

  //  model.getInitialized(inited);
  //  cv::imshow("inited",inited);

  // model.update(current,&foreground);



  //  model.getSigmas(sigmas,true);
  //  cv::imshow("vars",sigmas);


  if (grasp_detector.detectionAreaSet()){
    grasp_detector.newFrame(foreground);

    grasp_detector.detectGrasps(&rgb_image);

    // model.getDistance(current,dists);

    grasp_detector.detectPointingGesture(dists,&rgb_image);


    cv::namedWindow("detections");
    cv::imshow("detections",rgb_image);

  }








  model.getForeground_prob(current,sigma_cnt, foreground_prob);

  //  int nonzero = cv::countNonZero(foreground_prob);
  // 10% fuer 1 sigma
  // 2.5% fuer 2 sigma
  // 0.4% fuer 3 sigma
  //ROS_INFO("found %i nonzero pixels (%.1f %%)",nonzero,nonzero*100.0/current.size());

  //  cv::erode(foreground_prob,foreground_prob,cv::Mat(),cv::Point(-1,-1),2);
  //  cv::dilate(foreground_prob,foreground_prob,cv::Mat(),cv::Point(-1,-1),2);


  cv::imshow("foreground",foreground);

  cv::imshow("foreground_prob",foreground_prob);
  //  cv::imshow("stable",stable);

  cv::waitKey(10);


  if (pub_model.getNumSubscribers() > 0){
    Cloud result;
    model.getCloudRepresentation(current,result,1);

    Cloud::Ptr msg = result.makeShared();
    msg->header.frame_id = "/openni_rgb_optical_frame";
    msg->header.stamp = ros::Time::now();
    pub_model.publish(msg);
    // ROS_INFO("sending %zu points",result.size());
  }


}


void on_trackbar( int, void* ){
  error_factor = tb_val / 1000.0;
  ROS_INFO("error_factor: %f",error_factor);
}

void on_trackbar2( int, void* ){
  sigma_cnt = tb_val_2 / 10.0;
  ROS_INFO("sigma_cnt: %f",sigma_cnt);
}


void onMouse(int event, int x, int y, int, void* ptr)
{

  if (event == CV_EVENT_LBUTTONDOWN)
    points.push_back(cv::Point(x,y));

  if (points.size() == 4){
    user_mask = cv::Mat(480,640,CV_8UC1);
    user_mask.setTo(0);
    cv::fillConvexPoly(user_mask, points, cv::Scalar::all(255));

    grasp_detector.setDetectionArea(user_mask);

    cv::namedWindow("user mask");
    cv::imshow("user mask", user_mask );
    cv::waitKey(10);
    cv::imwrite("mask.png",user_mask);

    points.clear();
  }

}

int main(int argc, char ** argv){

  ros::init(argc,argv,"pincg_test");
  ros::NodeHandle nh;

  cv::namedWindow("rgb");
  cv::namedWindow("foreground_prob");
  //  cv::namedWindow("stable");
  //  cv::namedWindow("inited");
  cv::namedWindow("foreground");
  //  sprintf( "tb", "Alpha x %d", alpha_slider_max );

  //cv::namedWindow("vars");

  cv::createTrackbar( "foreground", "foreground", &tb_val, 200, on_trackbar );
  cv::createTrackbar( "foreground", "foreground_prob", &tb_val_2, 70, on_trackbar2 );
  on_trackbar(tb_val,NULL);
  on_trackbar2(tb_val_2,NULL);


  pub_model = nh.advertise<Cloud>("model", 1);
  ros::Subscriber sub_cam_info = nh.subscribe("/camera/rgb/points", 1, CloudCB);
  ros::Subscriber sub_color = nh.subscribe("/camera/rgb/image_color", 1, ImgCB);




  bool kinect_trafo_valid = loadAffineTrafo(kinect_trafo,"kinect_trafo.txt");

  if (!kinect_trafo_valid){
    ROS_INFO("fail");
    return -1;
  }




  model.init(640,480,10);

  cv::setMouseCallback("rgb", onMouse);


  user_mask = cv::imread("mask.png",0);

  if (user_mask.cols > 0){

    ROS_INFO("mask has %i pixels",cv::countNonZero(user_mask));

    grasp_detector.setDetectionArea(user_mask);
    model.setMask(user_mask);
  }



  ros::spin();

  return 23;
}
