using namespace std;
#include <iostream>
#include "rgbd_utils/gaussian_model.h"
#include <opencv-2.3.1/opencv2/highgui/highgui.hpp>
#include "rgbd_utils/pinch_detection.h"
#include "rgbd_utils/ants.h"


cv_bridge::CvImagePtr cv_ptr;
cv::Mat foreground,foreground_prob, stable, inited;

int frame_cnt = 0;
ros::Publisher pub_model, pub_hand;
cv::Mat rgb_image;

float error_factor;
int tb_val = 15;
int tb_val_2 = 0;
float sigma_cnt;
cv::Mat user_mask;
cv::Mat dists,last_static_dists,dist_thres;
cv::Mat sigmas;

cv::Mat last_static_norm, current_norm;


Eigen::Affine3f kinect_trafo;

int frame_check_cnt = 0;
int found_object_cnt = 0;

bool kinect_trafo_valid;

bool eval_running;

std::vector<cv::Point2i> points;


PixelEnvironmentModel model;
Detector detector;
Object_tracker<Grasp,Track<Grasp> > grasp_tracker;
Object_tracker<Playing_Piece,Track<Playing_Piece> > piece_tracker;
Object_tracker<Fingertip,Track<Fingertip> > fingertip_tracker;



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
  // ROS_INFO("error_factor: %f",error_factor);
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

  Cloud current_cloud;
  pcl::fromROSMsg(*cloud_ptr, current_cloud);


  int training_cnt = 100;

  if (frame_cnt < training_cnt){
    model.update(current_cloud);
    return;
  }

  if (frame_cnt == training_cnt){

    if (pub_model.getNumSubscribers() > 0){
      Cloud result;
      model.getCloudRepresentation(current_cloud,result,1);

      Cloud::Ptr msg = result.makeShared();
      msg->header.frame_id = "/openni_rgb_optical_frame";
      msg->header.stamp = ros::Time::now();
      pub_model.publish(msg);
      // ROS_INFO("sending %zu points",result.size());
    }

  }


  // timing_start("frame"); // 15ms

  model.getForeground_dist(current_cloud,error_factor,foreground);
  model.getNorm(current_cloud,current_norm);

  //  model.getInitialized(inited);
  //  cv::imshow("inited",inited);

  // model.update(current,&foreground);



  //  model.getSigmas(sigmas,true);
  //  cv::imshow("vars",sigmas);


  //  timing_start("foo");
  //  if (kinect_trafo_valid)
  //    pcl::getTransformedPointCloud(current,kinect_trafo,current);
  //  timing_end("foo");


  if (detector.detectionAreaSet()){
    detector.newFrame(foreground,current_norm,&current_cloud);
    detector.analyseScene(&rgb_image);
//    detector.analyseScene();
    //detector.showDetectionAreaEdge(rgb_image);
  }


//  cout << "XXX Detections: " << endl;
//  cout << "Grasps: " << detector.grasp_detections.size() << endl;
//  cout << "Objets: " << detector.object_detections.size() << endl;
//  cout << "Fingertips: " << detector.finger_detections.size() << endl;

//  cout << "OOO Tracks: " << endl;

//  cout << "Grasp: " << grasp_tracker.tracks.size() << endl;
//  cout << "Objects: " << piece_tracker.tracks.size() << endl;
//  cout << "Fingertips: " << fingertip_tracker.tracks.size() << endl;


  grasp_tracker.update_tracks(detector.grasp_detections);
  fingertip_tracker.update_tracks(detector.finger_detections);
  if ( !detector.handVisibleInLastFrame()){ // only update objects if hand was not visible (since objects could be occluded)
    piece_tracker.update_tracks(detector.object_detections);
  }


  // timing_end("frame");


  // visualize tracks
  Cloud grasps;
  for (GraspTrack_it it = grasp_tracker.tracks.begin(); it != grasp_tracker.tracks.end(); ++it){
    if (it->second.state == Track_Active){
      it->second.visualizeOnImage(rgb_image,getColor(it->first));
      pcl_Point center = it->second.last_detection().position_world;
      grasps.push_back(center);
      // ROS_INFO("Found grasp at: %f %f %f", center.x,center.y, center.z);

      // cout <<  center.x << "  " << center.y << "  " <<  center.z << endl;

      float angle;
      it->second.last_detection().getAngle_PCA_2D(angle);
      //ROS_INFO("Angle: %f", angle);

//      cout << angle << endl;

      Eigen::Affine3f trafo;
      it->second.last_detection().getAngle_PCA_3D(current_cloud,angle,&trafo);


    }
  }


  if (pub_hand.getNumSubscribers()){
    ROS_INFO("SENDING MAKRER");
    Cloud::Ptr msg = grasps.makeShared();
    msg->header = cloud_ptr->header;
    pub_hand.publish(msg);
  }


  for (PieceTrack_it it = piece_tracker.tracks.begin(); it != piece_tracker.tracks.end(); ++it){
    if (it->second.state == Track_Active){
      it->second.visualizeOnImage(rgb_image,getColor(it->first));
      pcl_Point center = it->second.last_detection().position_world;
      // ROS_INFO("Found Piece (%i) at: %f %f %f", it->first, center.x,center.y, center.z);
      // cout <<  center.x << "  " << center.y << "  " <<  center.z << endl;
    }
  }


  for (FingerTrack_it it = fingertip_tracker.tracks.begin(); it != fingertip_tracker.tracks.end(); ++it){
    // if (it->second.state == Track_Active)
    it->second.visualizeOnImage(rgb_image,getColor(it->first),TT_FINGERTIP);
    pcl_Point center = it->second.last_detection().position_world;
    // ROS_INFO("Found finger at: %f %f %f", center.x,center.y, center.z);
    // cout <<  center.x << "  " << center.y << "  " <<  center.z << endl;
  }



  cv::namedWindow("detections");
  cv::imshow("detections",rgb_image);



  //  model.getForeground_prob(current,sigma_cnt, foreground_prob);

  //  int nonzero = cv::countNonZero(foreground_prob);
  // 10% fuer 1 sigma
  // 2.5% fuer 2 sigma
  // 0.4% fuer 3 sigma
  //ROS_INFO("found %i nonzero pixels (%.1f %%)",nonzero,nonzero*100.0/current.size());


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

//  cv::namedWindow("dist");
//  cv::namedWindow("dist_thres");

  cv::createTrackbar( "foreground", "foreground", &tb_val, 200, on_trackbar );
  cv::createTrackbar( "check", "foreground", &tb_val_2, 1, on_trackbar2 );
  on_trackbar(tb_val,NULL);
  on_trackbar2(tb_val_2,NULL);


  pub_model = nh.advertise<Cloud>("model", 1);
  pub_hand = nh.advertise<Cloud>("grasps", 1);


  ros::Subscriber sub_cam_info = nh.subscribe("/camera/rgb/points", 1, CloudCB);
  ros::Subscriber sub_color = nh.subscribe("/camera/rgb/image_color", 1, ImgCB);




  kinect_trafo_valid = loadAffineTrafo(kinect_trafo,"kinect_trafo.txt");

  if (!kinect_trafo_valid){
    ROS_INFO("No Kinect trafo");
  }else{
  detector.setTransformation(kinect_trafo);
  }


  model.init(640,480,10);

  cv::setMouseCallback("rgb", onMouse);


  user_mask = cv::imread("mask.png",0);

  if (user_mask.cols > 0){

    // ROS_INFO("mask has %i pixels",cv::countNonZero(user_mask));

    detector.setDetectionArea(user_mask);
    model.setMask(user_mask);
  }



  ros::spin();

  return 23;
}
