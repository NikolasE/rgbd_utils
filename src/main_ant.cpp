using namespace std;
#include <iostream>

#include "rgbd_utils/ants.h"
#include "rgbd_utils/path_paramsConfig.h"
#include <dynamic_reconfigure/server.h>

Path_planner planner;
ros::Publisher pub_model,pub_marker;
cv::Mat h2;
visualization_msgs::Marker marker;

bool do_planning = false;

void doPlanning(){

  if (!do_planning)
    return;


  if (planner.apply_smoothing){
    cv::Mat smoothed;
    h2.copyTo(smoothed);
    cv::GaussianBlur(smoothed, smoothed, cv::Size(5,5),3);
    planner.setHeightMap(smoothed, 0.003);
  }else{
    planner.setHeightMap(h2,0.003);
  }


  int w = h2.cols;
  int h = h2.rows;

  planner.removeEnemies();
  planner.addEnemy(100,10,w/2,h/2);
    planner.addEnemy(100,10,w/2-20,h/2);
   planner.addEnemy(100,30,w/2-20,h/2-10);


  planner.computePolicy(cv::Point(w-10,h-10));
  planner.computePath(cv::Point(10,10));

  double min_, max_;
  cv::minMaxLoc(h2, &min_,&max_);
  cv::Mat scaled = (h2-min_)/(max_-min_);

  std::vector<cv::Point> path = planner.getPath();
  for (uint i=0; i<path.size()-1; ++i){
    cv::line(scaled, path[i],path[i+1],CV_RGB(255,255,255),1);
  }



  planner.createPathMarker(marker);




  cv::resize(scaled, scaled, cv::Size(),3,3);
  cv::imshow("scaled",scaled);
  cv::waitKey(10);
}


void paramCallback(const rgbd_utils::path_paramsConfig& config, uint32_t level){


  planner.setMaxSteepness(config.ant_steepness);
  planner.setHeightCostFactor(config.ant_heightcost_factor);
  planner.setUphillfactor(config.ant_uphill_factor);

  planner.setFourNeighbours(config.ant_use_four == 1);
  planner.setSmoothing(config.ant_use_smoothing == 1);
  planner.setHillSideCostFactor(config.ant_hillside_factor);
  planner.setPathLengthFactor(config.ant_path_length_factor);
  planner.enemy_factor = config.ant_enemy_factor;

  doPlanning();
}






int main(int argc, char ** argv){

  ros::init(argc,argv,"ant_test");
  ros::NodeHandle nh;
  
  pub_marker = nh.advertise<visualization_msgs::Marker>("/ant/path", 1);
  pub_model =  nh.advertise<Cloud>("/ant/model", 100);


  dynamic_reconfigure::Server<rgbd_utils::path_paramsConfig> srv;
  dynamic_reconfigure::Server<rgbd_utils::path_paramsConfig>::CallbackType f;
  f = boost::bind(paramCallback, _1, _2);
  srv.setCallback(f);




  cv::namedWindow("scaled");
  cv::Mat height = cv::imread(argv[1],0);


  height.convertTo(h2,CV_32FC1,1/250.0);


//  int w = h2.cols;
//  int h = h2.rows;

  do_planning = true;


  //  ROS_INFO("sending model with %i points", c.size());


  doPlanning();





  ros::Rate  r(10);

  while (ros::ok()){
    ros::spinOnce();

    Cloud::Ptr msg = planner.model.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now ();

    //    planner.publishPath(pub_marker);

    pub_model.publish(msg);
    pub_marker.publish(marker);
    r.sleep();
  }







  cv::waitKey(0);



  
  
  
}
