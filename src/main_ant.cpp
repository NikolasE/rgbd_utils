using namespace std;
#include <iostream>

#include "rgbd_utils/path_planning.h"
#include "rgbd_utils/path_paramsConfig.h"
#include <dynamic_reconfigure/server.h>

Path_planner planner;
ros::Publisher pub_model,pub_marker,pub_marker_enemy, pub_marker_range;
cv::Mat h2;
visualization_msgs::Marker marker_path;
visualization_msgs::Marker marker_enemies;
visualization_msgs::Marker marker_range;

bool do_planning = false;

float dist_threshold = 50;

void normalize(cv::Mat &img){
  double m,x;
  cv::minMaxLoc(img,&m,&x);
  img = (img-m)/(x-m);
}




float cell_size_m_ = 0.003;

void doPlanning(){

  if (!do_planning)
    return;

  planner.printParameters();

  if (planner.apply_smoothing){
    cv::Mat smoothed;
    h2.copyTo(smoothed);
    cv::GaussianBlur(smoothed, smoothed, cv::Size(5,5),3);
    planner.setHeightMap(smoothed, cell_size_m_);
  }else{
    planner.setHeightMap(h2, cell_size_m_);
  }


  int w = h2.cols;
  int h = h2.rows;

//  planner.removeEnemies();
//  planner.addEnemy(10,20,w-10,h/2);
//  planner.addEnemy(10,20,15,h/2+5);


  //      planner.addEnemy(10,10,w/2+30,h/2);
  //      planner.addEnemy(10,15,w/20,h/2+20);

  //  planner.addEnemy(10,20, 60,50);
  //  planner.addEnemy(10,20, 80,60);
  //  planner.addEnemy(10,20, 100,40);

  double min_1, max_1;
  cv::minMaxLoc(planner.enemy_cost, &min_1,&max_1);
//  cv::Mat scaled1 = (h2-min_1)/(max_1-min_1);

  //  ROS_INFO("enemy cost range: %f %f", min_1, max_1);

  cv::namedWindow("enemy");
  cv::Mat scaled1 = (planner.enemy_cost-min_1)/(max_1-min_1);
  cv::resize(scaled1, scaled1, cv::Size(),3/planner.scale,3/planner.scale);
  cv::imshow("enemy",scaled1);


  cv::Point start(w-1,h-1);
  cv::Point goal(30,60);


  timing_start("policy");
  planner.computePolicy(goal);
  timing_end("policy");


  cv::Mat dist_map;
  planner.getDistanceImage(dist_map);


  vector<cv::Point> contour;
  planner.getRangeOfMotion(dist_map,dist_threshold,contour);


  // planner.createRangeMarker(marker_range, contour);


  normalize(dist_map);
  cv::resize(dist_map,dist_map,cv::Size(), 3,3);
  cv::namedWindow("dist_map");
  cv::imshow("dist_map",dist_map);

  planner.computePath(start);


//  cv::Mat dists;
//  planner.getDistanceMap(dists,true);
//  cv::resize(dists, dists, cv::Size(),3,3);
//  cv::namedWindow("dist");
//  cv::imshow("dist",dists);

  // ROS_INFO("start: %i %i, end: %i %i", goal.x,goal.y, start.x,start.y);


  double min_, max_;
  cv::minMaxLoc(h2, &min_,&max_);
  cv::Mat scaled = (h2-min_)/(max_-min_);

  std::vector<cv::Point> path = planner.getPath();
  for (uint i=0; i<path.size()-1; ++i){
    cv::line(scaled, path[i],path[i+1],CV_RGB(255,255,255),1);
  }

  cv::resize(scaled, scaled, cv::Size(),3,3);
  cv::imshow("scaled",scaled);

  cv::waitKey(20);

   planner.createPathMarker(marker_path);
  // planner.createEnemyMarker(marker_enemies);

}


void paramCallback(const rgbd_utils::path_paramsConfig& config, uint32_t level){


  planner.setMaxSteepness(config.ant_steepness);
  planner.setHeightCostFactor(config.ant_heightcost_factor*100);
  planner.setUphillfactor(config.ant_uphill_factor);

  planner.setFourNeighbours(config.ant_use_four == 1);
  planner.setSmoothing(config.ant_use_smoothing == 1);
  planner.setHillSideCostFactor(config.ant_hillside_factor);
  // planner.setPathLengthFactor(config.ant_path_length_factor);
  planner.enemy_factor = config.ant_enemy_factor;

  planner.setScale(config.scale);


  dist_threshold = config.dist_treshold;

  doPlanning();
}



int main(int argc, char ** argv){

  ros::init(argc,argv,"ant_test");
  ros::NodeHandle nh;
  
  pub_marker = nh.advertise<visualization_msgs::Marker>("/ant/path", 1);
  pub_marker_enemy = nh.advertise<visualization_msgs::Marker>("/ant/enemies", 1);
  pub_marker_range = nh.advertise<visualization_msgs::Marker>("/ant/range", 1);


  pub_model =  nh.advertise<Cloud>("/ant/model", 100);


  dynamic_reconfigure::Server<rgbd_utils::path_paramsConfig> srv;
  dynamic_reconfigure::Server<rgbd_utils::path_paramsConfig>::CallbackType f;
  f = boost::bind(paramCallback, _1, _2);
  srv.setCallback(f);



  cv::namedWindow("scaled");
  cv::Mat height;
  if (argc == 3){
    h2 = cv::Mat(atoi(argv[1]),atoi(argv[2]),CV_32FC1);
    h2.setTo(0);
  }
  if (argc == 2){
    height = cv::imread(argv[1],0);
    height.convertTo(h2,CV_32FC1,1/255.0);
  }

  if (argc < 2){
    height = cv::imread("/usr/gast/engelhan/ros/master_thesis/projector_calibration/img/height.png",0);
    height.convertTo(h2,CV_32FC1,1/255.0);
  }

 // planner.setScale(0.5);

  //


  //  int w = h2.cols;
  //  int h = h2.rows;

  do_planning = true;



  // EVALUATION OF RUNNING TIME:

  //  int W[11]={10,20,50,75,100,125,150,175,180, 190, 200};
  //  int H[11]={10,20,50,75,100,125,150,175,180, 190, 200};
  // int l = 11;
  // cout << "# E*Vlog(V) E V time" << endl;

  //  for (int i=0; i<l; ++i)
  //    for (int j=0; j<l; ++j){
  //      // ROS_INFO("Size: %i %i",W[i],H[j]);
  //      h2 = cv::Mat(W[i],H[j],CV_32FC1);
  //      h2.setTo(0);

  ////      planner.use_four_neighbours = true;
  ////      doPlanning();
  //      planner.use_four_neighbours = false;
  //      doPlanning();
  //    }

  //  return 0;




  doPlanning();


  ros::Rate  r(2);




  while (ros::ok()){
    ros::spinOnce();

    Cloud::Ptr msg = planner.model.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now ();


//    if (planner.enemy_factor>0){
//      for (uint i=0; i<planner.enemies.size(); ++i){
//        planner.createEnemyMarker(marker_enemies,i);
//        pub_marker_enemy.publish(marker_enemies);
//      }
//    }

    pub_model.publish(msg);
    pub_marker_range.publish(marker_range);
    pub_marker.publish(marker_path);

    r.sleep();
  }



  
  
}
