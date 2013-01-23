using namespace std;
#include <iostream>
#include "rgbd_utils/cloud_gmm.h"
#include <opencv-2.3.1/opencv2/highgui/highgui.hpp>
#include "rgbd_utils/pinch_detection.h"


PixelEnvironmentModel model;
cv_bridge::CvImagePtr cv_ptr;
cv::Mat foreground,foreground_prob, stable, inited;

int frame_cnt = 0;
ros::Publisher pub_model;
cv::Mat rgb_image;

float error_factor;
int tb_val = 15;
int tb_val_2 = 0;
float sigma_cnt;
cv::Mat user_mask;
cv::Mat dists;
cv::Mat sigmas;

Detector detector;
Eigen::Affine3f kinect_trafo;

int frame_check_cnt = 0;
int found_object_cnt = 0;

bool kinect_trafo_valid;

bool eval_running;

std::vector<cv::Point2i> points;


void ImgCB(const sensor_msgs::ImageConstPtr& img_ptr){
  cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
  rgb_image = cv_ptr->image;

  cv::Mat darker = rgb_image*0.5;
  cv::Mat masked; rgb_image.copyTo(masked);
  darker.copyTo(masked,255-foreground);

  cv::imshow("rgb",masked);
}


void on_trackbar( int, void* ){
  error_factor = tb_val / 1000.0;
  ROS_INFO("error_factor: %f",error_factor);
}

void on_trackbar2( int val, void* ){

  if (val == 0){
    frame_check_cnt = found_object_cnt = 0;
    eval_running = false;
  }else{
    eval_running = true;
  }


}


void CloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){

  frame_cnt++;

  Cloud current;
  pcl::fromROSMsg(*cloud_ptr, current);


  int training_cnt = 100;

  if (frame_cnt < training_cnt){
    model.update(current);
    return;
  }

  if (frame_cnt == training_cnt){

    //    model.getSigmas(sigmas,true);
    //    cv::namedWindow("sigmas");
    //    cv::imshow("sigmas",sigmas);

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


  // model.update(current);

  // model.getStableMeasurements(stable);

  model.getForeground_dist(current,error_factor,foreground);

  //  model.getInitialized(inited);
  //  cv::imshow("inited",inited);

  // model.update(current,&foreground);



  //  model.getSigmas(sigmas,true);
  //  cv::imshow("vars",sigmas);


  if (detector.detectionAreaSet()){
    detector.newFrame(foreground);


    detector.analyseScene(&rgb_image);

    // ROS_INFO("Found %zu grasps and %zu objects", grasp_detector.grasp_detections.size(),grasp_detector.object_detections.size());

    //    grasp_detector.detectGrasps(&rgb_image);
    //    grasp_detector.detectPointingGesture(dists,&rgb_image);


    if (eval_running){
      uint n = detector.object_detections.size();

      if (n<2){
        frame_check_cnt++;
        found_object_cnt+=n;
      }

      ROS_INFO("looking for objects: %i / %i",found_object_cnt,frame_check_cnt);

      if (frame_check_cnt == 100){
        eval_running = false;
        on_trackbar2(0,NULL);
      }

    }


    printTrafo(kinect_trafo);


    // get position of every grasp:
    for(uint i=0; i<detector.grasp_detections.size(); ++i){
      pcl_Point mean = getCenter(detector.grasp_detections[i].edge,current);


      ROS_INFO("mean (no trafo) %i: %f %f %f",i,mean.x,mean.y,mean.z);


      if (kinect_trafo_valid){
        mean = getTransformedPoint(mean, kinect_trafo);
        detector.grasp_detections[i].position = mean;
      }

      ROS_INFO("mean %i: %f %f %f",i,mean.x,mean.y,mean.z);
    }


    detector.showDetectionAreaEdge(rgb_image);

    cv::namedWindow("detections");
    cv::imshow("detections",rgb_image);

  }








  //  model.getForeground_prob(current,sigma_cnt, foreground_prob);

  //  int nonzero = cv::countNonZero(foreground_prob);
  // 10% fuer 1 sigma
  // 2.5% fuer 2 sigma
  // 0.4% fuer 3 sigma
  //ROS_INFO("found %i nonzero pixels (%.1f %%)",nonzero,nonzero*100.0/current.size());

  //  cv::erode(foreground_prob,foreground_prob,cv::Mat(),cv::Point(-1,-1),2);
  //  cv::dilate(foreground_prob,foreground_prob,cv::Mat(),cv::Point(-1,-1),2);


  cv::imshow("foreground",foreground);
  //  cv::imshow("foreground_prob",foreground_prob);


  cv::waitKey(10);




}





void onMouse(int event, int x, int y, int, void* ptr)
{

  if (event == CV_EVENT_LBUTTONDOWN)
    points.push_back(cv::Point(x,y));

  if (points.size() == 4){
    user_mask = cv::Mat(480,640,CV_8UC1);
    user_mask.setTo(0);
    cv::fillConvexPoly(user_mask, points, cv::Scalar::all(255));

    detector.setDetectionArea(user_mask);

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
  //  cv::namedWindow("foreground_prob");
  //  cv::namedWindow("stable");
  //  cv::namedWindow("inited");
  cv::namedWindow("foreground");
  //  sprintf( "tb", "Alpha x %d", alpha_slider_max );

  //cv::namedWindow("vars");

  cv::createTrackbar( "foreground", "foreground", &tb_val, 200, on_trackbar );
  cv::createTrackbar( "check", "foreground", &tb_val_2, 1, on_trackbar2 );
  on_trackbar(tb_val,NULL);
  on_trackbar2(tb_val_2,NULL);


  pub_model = nh.advertise<Cloud>("model", 1);
  ros::Subscriber sub_cam_info = nh.subscribe("/camera/rgb/points", 1, CloudCB);
  ros::Subscriber sub_color = nh.subscribe("/camera/rgb/image_color", 1, ImgCB);




  kinect_trafo_valid = loadAffineTrafo(kinect_trafo,"kinect_trafo.txt");

  if (!kinect_trafo_valid){
    ROS_INFO("No Kinect trafo");
    // return -1;
  }




  model.init(640,480,10);

  cv::setMouseCallback("rgb", onMouse);


  user_mask = cv::imread("mask.png",0);

  if (user_mask.cols > 0){

    ROS_INFO("mask has %i pixels",cv::countNonZero(user_mask));

    detector.setDetectionArea(user_mask);
    model.setMask(user_mask);
  }



  ros::spin();

  return 23;
}
